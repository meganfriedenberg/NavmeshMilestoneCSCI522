
// API Abstraction
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

// Inter-Engine includes
#include "PrimeEngine/MemoryManagement/Handle.h"
#include "PrimeEngine/PrimitiveTypes/PrimitiveTypes.h"
#include "PrimeEngine/Events/Component.h"


#include "PrimeEngine/Utils/Networkable.h"
#include "PrimeEngine/Scene/MeshInstance.h"

#include "PrimeEngine/Utils/Array/Array.h"
#include "PrimeEngine/APIAbstraction/Effect/Effect.h"


// Sibling/Children includes
#include "PrimeEngine/Scene/SceneNode.h"

namespace PE {
	namespace Components {
		struct CollisionResult
		{
			Vector3 mDirection;
			float mDepthSq;
			Vector3 mHitPoint;
			bool m_hasCollision;
		};
		struct LineSegment
		{
			Vector3 m_startPoint;
			Vector3 m_endPoint;
		};
		struct WallManager : public SceneNode
		{
			PE_DECLARE_CLASS(WallManager);

			static void Construct(PE::GameContext& context, PE::MemoryArena arena);

			// Constructor -------------------------------------------------------------
			// same
			WallManager(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself) : SceneNode(context, arena, hMyself)
			{
				m_components.reset(512);
				mComponents.reset(256);
			}

			virtual ~WallManager() {}

			void addWallBody(Handle h);

			// component
			virtual void addDefaultComponents();

			bool RayCast(const LineSegment& segment, Vector3* pHitPoint, bool drawDebugLine);
			bool Intersect(const LineSegment& segment, struct WallComponent* box, CollisionResult& collResult, Vector3* pHitPoint);

			//static Handle s_hInstance;

			//static void SetInstance(Handle h) { s_hInstance = h; }
			//static WallManager* Instance() { return s_hInstance.getObject<WallManager>(); }
			//static Handle InstanceHandle() { return s_hInstance; }

			// Array of all PhysicsComponents
			Array<Handle> mComponents;


		};
		struct WallComponent : public Component
		{
			PE_DECLARE_CLASS(WallComponent);

			static void Construct(PE::GameContext& context, PE::MemoryArena arena, MeshInstance* parent);

			// Constructor -------------------------------------------------------------
			// same
			WallComponent(PE::GameContext& context, PE::MemoryArena arena, PE::Handle hMyself, MeshInstance* parent) : Component(context, arena, hMyself)
			{
				mManager = context.getWallManager();
				mParent = parent;
				//context.getPhysicsManager()->mComponents.add(this); // add to the physicsManager on construction
				mManager->addWallBody(hMyself);

			}

			virtual ~WallComponent() {}

			SceneNode* getParentsSceneNode();


			// component
			virtual void addDefaultComponents();

			WallManager* mManager = nullptr;

			MeshInstance* mParent = nullptr;

			struct SceneNode* mSceneNode = nullptr;

			struct SceneNode* mRotateScene = nullptr;

			Vector3 minPos;
			Vector3 maxPos;


		};
	}; // end Components
}; // end PE