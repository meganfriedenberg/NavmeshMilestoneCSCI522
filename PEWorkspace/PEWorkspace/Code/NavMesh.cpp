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

						if (pCurrCell->numCells == 3)
						{
							break; // filled in, so stop traversing
						}
					}

				}

			}

		}

		std::vector<Cell*> PE::Components::NavMesh::findCellPath(Cell* pStart, Cell* pGoal, Vector3 end)
		{

			// need to call A* for pathfinding
			std::unordered_map<Cell*, Cell*> pathFrom;
			std::unordered_map<Cell*, int> totalCosts;

			std::vector<Cell*> mCellPath(totalCosts[pGoal]+2, nullptr);

			// Rebuild cell pathing
			std::vector<Cell*> cellPath(totalCosts[pGoal] + 2, nullptr);
			Cell* currentCell = pGoal;
			while (currentCell != pStart)
			{
				cellPath[totalCosts[currentCell]] = currentCell;
				currentCell = pathFrom[currentCell];
			}
			cellPath[0] = currentCell;


			return mCellPath;
		}

		void PE::Components::NavMesh::AStar(std::unordered_map<Cell*, Cell*>&, std::unordered_map<Cell*, int>&, Cell* pStart, Cell* pGoal, Vector3 end)
		{



		}

		float PE::Components::NavMesh::getHeuristic(Cell* a, Vector3 b)
		{
			return 0.0f;
		}

	}; // namespace Components
}; // namespace PE