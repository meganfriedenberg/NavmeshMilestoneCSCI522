#define NOMINMAX

#include "NavMesh.h"

#include "PrimeEngine/APIAbstraction/Effect/EffectManager.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"
#include "PrimeEngine/Render/ShaderActions/SetPerFrameConstantsShaderAction.h"

namespace PE {
	namespace Components {
		PE_IMPLEMENT_CLASS1(NavMesh, SceneNode);
		PE_IMPLEMENT_CLASS1(Cell, Component);

		void Cell::Construct(PE::GameContext& context, PE::MemoryArena arena)
		{
			Handle h("CELL", sizeof(Cell));
			Cell* pCell = new(h) Cell(context, arena, h);
			pCell->addDefaultComponents();
		}

		void Cell::addDefaultComponents()
		{
			Component::addDefaultComponents();
		}

		// Navmesh Implementation


		Handle NavMesh::s_hInstance;

		void NavMesh::Construct(PE::GameContext& context, PE::MemoryArena arena)
		{
			Handle h("NAV_MESH", sizeof(NavMesh));
			NavMesh* pNavMesh = new(h) NavMesh(context, arena, h);
			pNavMesh->addDefaultComponents();
			SetInstance(h);
		}

		void NavMesh::addDefaultComponents()
		{
			SceneNode::addDefaultComponents();
		}

		// Graph implementations

		// For some reason, some of the vertex positions are off by about 0.01 instead of being exactly equal even when shared
		bool PE::Components::NavMesh::isCloseEnough(float a, float b)
		{
			return false;
			// this function ended up not being necessary after drawing triangles out by hand based on positionBuffer's coords
		}

		Vector3 PE::Components::NavMesh::getTriangleCenter(Vector3 a, Vector3 b, Vector3 c)
		{
			Vector3 result = (a + b + c) / 3;
			return result;
		}

		bool PE::Components::NavMesh::isConnected(Cell* a, Cell* b)
		{
			bool firstVert = (a->verts[0] == b->verts[0]) || (a->verts[0] == b->verts[1]) || (a->verts[0] == b->verts[2]);
			bool secondVert = (a->verts[1] == b->verts[0]) || (a->verts[1] == b->verts[1]) || (a->verts[1] == b->verts[2]);
			bool thirdVert = (a->verts[2] == b->verts[0]) || (a->verts[2] == b->verts[1]) || (a->verts[2] == b->verts[2]);

			return (firstVert && secondVert) || (firstVert && thirdVert) || (secondVert && thirdVert);
		}

		void PE::Components::NavMesh::connectGraph()
		{
			for (int i = 1; i < m_components.m_size - 1; i++) // Check m_components[1], size - 1 so don't go off the edge
			{
				Cell* pCurrCell = getCell(i);

				// If already connected in graph, continue, but none in the first example will have 3 triangles, only 2
				if (pCurrCell->numCells == 3){
					continue;
				}

				for (int j = i + 1; j < m_components.m_size; j++) // Start one in front, but can go to last triangle
				{
					Cell* potentialCell = getCell(j);

					// Add the edge to the triangles if they share it
					if (potentialCell->numCells < 3 && isConnected(pCurrCell, potentialCell))
					{
						pCurrCell->adjCells[pCurrCell->numCells] = potentialCell;
						potentialCell->adjCells[potentialCell->numCells] = pCurrCell;

						pCurrCell->numCells++;
						potentialCell->numCells++;
						if (potentialCell->numCells == 3)
						{
							totalFullyConnectedTriangles++;
						}

						if (pCurrCell->numCells == 3)
						{
							totalFullyConnectedTriangles++;
							break; // filled in, so stop traversing
						}

					}

				}

			}
			totalCells = m_components.m_size - 1;

		}

		Cell* PE::Components::NavMesh::getCurrentCell(Vector3 pos) // source used: https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
		{
			// compute the current cell by using barycentric coordintes (recommended by Terry Lu)

			// page 47 and 48 from the above linked source, used this formula
			for (int i = 1; i < m_components.m_size; i++) // literally check every single triangle
			{
				Cell* pCell = getCell(i);

				// Compute vectors
				Vector3 v0 = pCell->verts[2] - pCell->verts[0];
				Vector3 v1 = pCell->verts[1] - pCell->verts[0];
				Vector3 v2 = pos - pCell->verts[0];

				// Compute dot products
				float dot00 = v0.dotProduct(v0);
				float dot01 = v0.dotProduct(v1);
				float dot02 = v0.dotProduct(v2);
				float dot11 = v1.dotProduct(v1);
				float dot12 = v1.dotProduct(v2);

				// Compute barcentric coordinates
				float inverseDenominator = 1 / (dot00 * dot11 - dot01 * dot01);
				float u = (dot11 * dot02 - dot01 * dot12) * inverseDenominator;
				float w = (dot00 * dot12 - dot01 * dot02) * inverseDenominator;

				// If in cell, return pointer
				if (u >= 0 && w >= 0 && u + w < 1) // it follows that u + w < 1 because u + w + v = 1.0, where (u,w,v) are the barycentric coordinates
					return pCell;
			}

			// If not in cell, return nullptr	
			return nullptr;

		}

		// given a "goal" as a vector3 and a "start", find a path by the simple stupid funnel algo
		std::vector<Vector3> PE::Components::NavMesh::findPath(Vector3 startPos, Vector3 goalPos, int& pathLength)
		{
			Cell* cellStart = getCurrentCell(startPos);
			Cell* cellEnd = getCurrentCell(goalPos);

			if (cellStart == nullptr || cellEnd == nullptr)
			{
				return std::vector<Vector3>(); // return an empty vector, can't find a path between
			}
			else if (cellStart == cellEnd)
			{
				std::vector<Vector3> simplePath;
				simplePath.push_back(startPos);
				simplePath.push_back(goalPos);
				return simplePath;

			}

			std::vector<Vector3> complexPath;

			std::vector<Cell*>complexCellPath = findCellPath(cellStart, cellEnd, goalPos); // calls A* to find a cell path
			if (complexCellPath.size() > 0) // double check that A* DID return a path, it always should, but just in case
			{
				int totalPortals = 1; // will be at least one
				//complexCellPath.push_back(nullptr); // so build portals knows when to STOP, this might be an issue
				std::vector<Vector3> constructedPortals = buildPortals(complexCellPath, totalPortals, startPos, goalPos);

				simpleStupidFunnelAlgorithm(complexPath, constructedPortals, totalPortals, pathLength);
			}
			else
			{
				OutputDebugStringA("COULD NOT FIND PATH VIA A*\n");
			}


			return complexPath;
		}

		std::vector<Cell*> PE::Components::NavMesh::findCellPath(Cell* pStart, Cell* pGoal, Vector3 end)
		{

			// need to call A* for pathfinding
			std::unordered_map<Cell*, Cell*> pathFrom;
			std::unordered_map<Cell*, int> totalCosts;

			AStar(pathFrom, totalCosts, pStart, pGoal, end);

			std::vector<Cell*> mCellPath(totalCosts[pGoal]+2, nullptr);

			// Rebuild cell pathing
			Cell* currentCell = pGoal;
			while (currentCell != pStart)
			{
				mCellPath[totalCosts[currentCell]] = currentCell;
				currentCell = pathFrom[currentCell];
			}
			mCellPath[0] = currentCell;


			return mCellPath;
		}

		void PE::Components::NavMesh::AStar(std::unordered_map<Cell*, Cell*>& pathFrom, std::unordered_map<Cell*, int>& totalCosts, Cell* pStart, Cell* pGoal, Vector3 end)
		{

			// Initialize data
			PriorityQueue<Cell*, float> openSet;
			openSet.put(pStart, 0);

			// init the breadcrumbs path back
			pathFrom[pStart] = pStart;
			totalCosts[pStart] = 0;

			while (!openSet.empty())
			{
				// See if the next cell is the goal otherwise, keep going
				Cell* pCurrentCell = openSet.get();
				if (pCurrentCell == pGoal) break;

				// Add all neighboring cells to the queue
				for (Cell* next : pCurrentCell->adjCells)
				{
					if (next == nullptr) continue;

					float newCost = totalCosts[pCurrentCell] + 1;
					// If previously undiscovered OR it is now cheaper, add to queue and record
					if (totalCosts.find(next) == totalCosts.end() || newCost < totalCosts[next])
					{
						totalCosts[next] = newCost;
						float priority = newCost * getHeuristic(next, end);
						openSet.put(next, priority);
						pathFrom[next] = pCurrentCell;
					}
				}
			}

		}
		// just whichever vertex is closer to the goal, use that one
		float PE::Components::NavMesh::getHeuristic(Cell* a, Vector3 b)
		{
			float vertCost1 = (a->verts[0].m_x - b.m_x) * (a->verts[0].m_x - b.m_x) + (a->verts[0].m_z - b.m_z) * (a->verts[0].m_z - b.m_z);
			float vertCost2 = (a->verts[1].m_x - b.m_x) * (a->verts[1].m_x - b.m_x) + (a->verts[1].m_z - b.m_z) * (a->verts[1].m_z - b.m_z);
			float vertCost3 = (a->verts[2].m_x - b.m_x) * (a->verts[2].m_x - b.m_x) + (a->verts[2].m_z - b.m_z) * (a->verts[2].m_z - b.m_z);

			return min(min(vertCost1, vertCost2), vertCost3);
		}
		
		// for simple stupid funnel algo
		float PE::Components::NavMesh::triarea2(Vector3 a, Vector3 b, Vector3 c) // from http://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html?m=1
		{
			const float ax = b.m_x - a.m_x;
			const float ay = b.m_z - a.m_z;
			const float bx = c.m_x - a.m_x;
			const float by = c.m_z - a.m_z;
			return bx * ay - ax * by;
		}

		// reference: http://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html?m=1
		std::vector<Vector3> PE::Components::NavMesh::buildPortals(std::vector<Cell*> cellPath, int& totalPortals, Vector3 startPos, Vector3 goalPos)
		{
			// the cellPath from A* already has the full path, we just have to traverse it and break when the goal is reached
			// init the portal storing structure
			std::vector<Vector3> portalList(cellPath.size() * 2);
			portalList[0] = startPos; // left start
			portalList[1] = startPos; // right start

			Cell* left = cellPath[0];
			Cell* right = cellPath[1];
			while (right != nullptr) // appended a nullptr prior so we know when we're done, in case we need to revisit the goalCell
			{
				// see which vertices are shared before trying to identify a portal line
				bool firstVert = (left->verts[0] == right->verts[0]) || (left->verts[0] == right->verts[1]) || (left->verts[0] == right->verts[2]);
				bool secondVert = (left->verts[1] == right->verts[0]) || (left->verts[1] == right->verts[1]) || (left->verts[1] == right->verts[2]);
				bool thirdVert = (left->verts[2] == right->verts[0]) || (left->verts[2] == right->verts[1]) || (left->verts[2] == right->verts[2]);

				// use winding order like from the picture
				// firstVert is the vertex on the "left" while thirdVert is the other side of the triangle (secondVert is "top" of the triangle)
				if (firstVert && secondVert)
				{
					portalList[totalPortals * 2 + 0] = left->verts[0];
					portalList[totalPortals * 2 + 1] = left->verts[1];
				}
				else if (firstVert && thirdVert)
				{
					portalList[totalPortals * 2 + 0] = left->verts[2];
					portalList[totalPortals * 2 + 1] = left->verts[0];
				}
				else if (secondVert && thirdVert)
				{
					portalList[totalPortals * 2 + 0] = left->verts[1];
					portalList[totalPortals * 2 + 1] = left->verts[2];
				}

				// "sweep" from left to right, go to next cell
				totalPortals++;
				left = right;
				right = cellPath[totalPortals];
			}

			// need to add the goal in since the algo will break right before adding the goal in
			portalList[totalPortals * 2 + 0] = goalPos;
			portalList[totalPortals * 2 + 1] = goalPos;
			totalPortals++;

			return portalList;
		}

		// based off of: http://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html?m=1
		void NavMesh::simpleStupidFunnelAlgorithm(std::vector<Vector3>& path, std::vector<Vector3>& portals, int numPortals, int& pathLength)
		{
			int npts = 0;
			Vector3 portalApex;
			Vector3 portalLeft;
			Vector3 portalRight;
			int apexIndex = 0;
			int leftIndex = 0;
			int rightIndex = 0;

			portalApex = portals[0];
			portalLeft = portals[0];
			portalRight = portals[1]; // should this be  [1] or [2]???

			// Add the start point
			path.push_back(portalApex);
			npts++;

			for (int i = 1; i < numPortals && npts < 512; ++i)
			{
				Vector3 left = portals[i * 2 + 0];
				Vector3 right = portals[i * 2 + 1];

				// Update right vertex
				if (triarea2(portalApex, portalRight, right) <= 0.0f)
				{
					if ((portalApex == portalRight) || triarea2(portalApex, portalLeft, right) > 0.0f)
					{
						// Tighten the funnel
						portalRight = right;
						rightIndex = i;
					}
					else
					{
						// Right over left, insert left to path and restart scan from portal left point.
						path.push_back(portalLeft);
						npts++;

						// Make current left the new apex.
						portalApex = portalLeft;
						apexIndex = leftIndex;

						// Reset portal
						portalRight = portalApex;
						rightIndex = apexIndex;

						// Restart scan
						i = apexIndex;
						continue;
					}
				}

				// Update the left vertex.
				if (triarea2(portalApex, portalLeft, left) >= 0.0f)
				{
					if (portalApex == portalLeft || triarea2(portalApex, portalRight, left) < 0.0f)
					{
						// Tighten the funnel
						portalLeft = left;
						leftIndex = i;
					}
					else
					{
						// Left over right, insert right to path and restart scan from portal right point.
						path.push_back(portalRight);
						npts++;

						// Make current right the new apex.
						portalApex = portalRight;
						apexIndex = rightIndex;

						// Reset portal
						portalLeft = portalApex;
						leftIndex = apexIndex;

						// Restart scan
						i = apexIndex;
						continue;
					}
				}
			} // end for

			pathLength = npts;

		}
	}; // namespace Components
}; // namespace PE