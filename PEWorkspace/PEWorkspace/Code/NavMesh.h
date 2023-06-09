#ifndef __PYENGINE_2_0_NAVMESH_H__
#define __PYENGINE_2_0_NAVMESH_H__

// API Abstraction
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

// Outer-Engine includes
#include <assert.h>
#include <vector>

// Inter-Engine includes
#include "PrimeEngine/MemoryManagement/Handle.h"
#include "PrimeEngine/PrimitiveTypes/PrimitiveTypes.h"
#include "PrimeEngine/Events/Component.h"
#include "PrimeEngine/Utils/Array/Array.h"
#include "PrimeEngine/APIAbstraction/Effect/Effect.h"


// Sibling/Children includes
#include "PrimeEngine/Scene/SceneNode.h"

#include <unordered_map>
#include <queue>

template<typename T, typename priority_t>
struct PriorityQueue {
	typedef std::pair<priority_t, T> PQElement;
	std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

	inline bool empty() const {
		return elements.empty();
	}

	inline void put(T item, priority_t priority) {
		elements.emplace(priority, item);
	}

	T get() {
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};

namespace PE {
	namespace Components {

		struct Cell : public Component
		{
			PE_DECLARE_CLASS(Cell);

			static void Construct(PE::GameContext& context, PE::MemoryArena arena);

			// Constructor -------------------------------------------------------------
			// same
			Cell(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself) : Component(context, arena, hMyself)
			{
				m_components.reset(512);
			}

			virtual ~Cell() {}

			// component
			virtual void addDefaultComponents();
			// Individual events -------------------------------------------------------
			// this method will set up some global gpu constants like game time, frame time
			// it will also set light source gpu constants

			Vector3 verts[3]; // what makes up this triangle
			Cell* adjCells[3]; // what are the 3 adjacent triangles
			int numCells = 0;
		};

		struct NavMesh : public SceneNode
		{
			PE_DECLARE_CLASS(NavMesh);

			static void Construct(PE::GameContext& context, PE::MemoryArena arena);

			// Constructor -------------------------------------------------------------
			// same
			NavMesh(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself) : SceneNode(context, arena, hMyself)
			{
				m_components.reset(512);
			}

			virtual ~NavMesh() {}

			// component
			virtual void addDefaultComponents();

			// Singleton instancing
			static NavMesh* Instance() { return s_hInstance.getObject<NavMesh>(); }
			static Handle InstanceHandle() { return s_hInstance; }

			static void SetInstance(Handle h) { s_hInstance = h; }

			// Graph Functions
			void connectGraph();
			bool isConnected(Cell* a, Cell* b);
			bool isCloseEnough(float a, float b);
			Vector3 getTriangleCenter(Vector3 a, Vector3 b, Vector3 c);

			// Pathfinding Functions
			std::vector<Vector3> findPath(Vector3 start, Vector3 end, int& length);
			std::vector<Cell*> findCellPath(Cell* pStartingCell, Cell* pFinalCell, Vector3 end);
			void AStar(std::unordered_map<Cell*, Cell*>&, std::unordered_map<Cell*, int>&, Cell*, Cell*, Vector3);
			Cell* getCurrentCell(Vector3 pos); // return the cell closest to this position
			float getHeuristic(Cell* a, Vector3 b);
			float triarea2(Vector3 a, Vector3 b, Vector3 c);
			std::vector<Vector3> buildPortals(std::vector<Cell*> cellPath, int& numPortals, Vector3 startPos, Vector3 goalPos);
			void simpleStupidFunnelAlgorithm(std::vector<Vector3>& path, std::vector<Vector3>& portals, int numPortals, int& pathLength);

			int totalCells;


			inline Cell* getCell(int index)
			{
				Handle hCell = m_components[index];
				return hCell.getObject<Cell>();
			}

		private:
			static Handle s_hInstance;
			int totalFullyConnectedTriangles = 0;

		};

	}; // namespace Components
}; // namespace PE
#endif // file guard