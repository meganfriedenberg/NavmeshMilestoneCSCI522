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

		/**************************
		 * NAVMESH IMPLEMENTATION *
		 **************************/


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
			//Component::addDefaultComponents();
			SceneNode::addDefaultComponents();
			//Component::addDefaultComponents();
		}
	}; // namespace Components
}; // namespace PE