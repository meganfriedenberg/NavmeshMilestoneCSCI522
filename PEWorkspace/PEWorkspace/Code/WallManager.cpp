#define NOMINMAX

#include "WallManager.h"

#include "PrimeEngine/APIAbstraction/Effect/EffectManager.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"
#include "PrimeEngine/Render/ShaderActions/SetPerFrameConstantsShaderAction.h"

#include "PrimeEngine/Scene/DebugRenderer.h"

namespace PE {
	namespace Components {
		PE_IMPLEMENT_CLASS1(WallManager, SceneNode);
		PE_IMPLEMENT_CLASS1(WallComponent, Component);

		void WallComponent::Construct(PE::GameContext& context, PE::MemoryArena arena, MeshInstance* parent)
		{
			Handle h("WALL_COMPONENT", sizeof(WallComponent));
			WallComponent* pWallComponent = new(h) WallComponent(context, arena, h, parent);
			pWallComponent->addDefaultComponents();
		}

		void WallComponent::addDefaultComponents()
		{
			//Component::addDefaultComponents();
		}

		// WallManager Implementation


		//Handle WallComponent::s_hInstance;

		void WallManager::Construct(PE::GameContext& context, PE::MemoryArena arena)
		{
			Handle h("WALL_MANAGER", sizeof(WallManager));
			WallManager* pNavMesh = new(h) WallManager(context, arena, h);
			pNavMesh->addDefaultComponents();
			//SetInstance(h);
		}

		void WallManager::addDefaultComponents()
		{
			SceneNode::addDefaultComponents();
		}

		void PE::Components::WallManager::addWallBody(Handle h)
		{
			mComponents.add(h);
		}

		bool WallManager::RayCast(const LineSegment& segment, Vector3* pHitPoint, bool drawDebugLine)
		{
			Vector3 hitPoint;
			bool intersect = false;
			CollisionResult collResult;
			if (drawDebugLine)
			{
				DebugRenderer::Instance()->createLineTwoPoints(segment.m_startPoint, segment.m_endPoint, Vector3(1.0f, 0.0f, 0.0f), 0.0f, 10.0f);
			}
			for (size_t i = 0; i < mComponents.m_size; i++) {
				WallComponent* pWallInst = mComponents[i].getObject<WallComponent>();
				if (Intersect(segment, pWallInst, collResult, &hitPoint)) {
					intersect = true;
					if ((segment.m_startPoint - hitPoint).lengthSqr() < (segment.m_startPoint - *pHitPoint).lengthSqr()) {
						*pHitPoint = hitPoint;
					}
				}
			}
			return intersect;
		}

        bool WallManager::Intersect(const LineSegment& segment, WallComponent* box, CollisionResult& collResult, Vector3* pHitPoint)
        {
            float t = 0.0f;
            float tmin = 0.0f;
            float tmax = FLT_MAX;
            Vector3 segDir = (segment.m_endPoint - segment.m_startPoint);

            Vector3 minWorld = box->mSceneNode->m_base * box->mParent->getMesh()->minPoint;
            Vector3 maxWorld = box->mSceneNode->m_base * box->mParent->getMesh()->maxPoint;

            // x
            if (pemath::isZero(segDir.m_x)) {
                if (segment.m_startPoint.m_x < minWorld.m_x || segment.m_startPoint.m_x > maxWorld.m_x) {
                    collResult.m_hasCollision = false;
                    return false;
                }
            }
            else {
                float ood = 1.0f / segDir.m_x;
                float t1 = (minWorld.m_x - segment.m_startPoint.m_x) * ood;
                float t2 = (maxWorld.m_x - segment.m_startPoint.m_x) * ood;
                if (t1 > t2) std::swap(t1, t2);
                tmin = max(tmin, t1);
                tmax = min(tmax, t2);
                if (tmin > tmax) {
                    collResult.m_hasCollision = false;
                    return false;
                }
            }
            // y
            if (pemath::isZero(segDir.m_y)) {
                if (segment.m_startPoint.m_y < minWorld.m_y || segment.m_startPoint.m_y > maxWorld.m_y) {
                    collResult.m_hasCollision = false;
                    return false;
                }
            }
            else {
                float ood = 1.0f / segDir.m_y;
                float t1 = (minWorld.m_y - segment.m_startPoint.m_y) * ood;
                float t2 = (maxWorld.m_y - segment.m_startPoint.m_y) * ood;
                if (t1 > t2) std::swap(t1, t2);
                tmin = max(tmin, t1);
                tmax = min(tmax, t2);
                if (tmin > tmax) {
                    collResult.m_hasCollision = false;
                    return false;
                }
            }
            // z
            if (pemath::isZero(segDir.m_z)) {
                if (segment.m_startPoint.m_z < minWorld.m_z || segment.m_startPoint.m_z > maxWorld.m_z) {
                    collResult.m_hasCollision = false;
                    return false;
                }
            }
            else {
                float ood = 1.0f / segDir.m_z;
                float t1 = (minWorld.m_z - segment.m_startPoint.m_z) * ood;
                float t2 = (maxWorld.m_z - segment.m_startPoint.m_z) * ood;
                if (t1 > t2) std::swap(t1, t2);
                tmin = max(tmin, t1);
                tmax = min(tmax, t2);
                if (tmin > tmax) {
                    collResult.m_hasCollision = false;
                    return false;
                }
            }

            if (tmin > 1.0f) {
                collResult.m_hasCollision = false;
                return false;
            }

            *pHitPoint = segment.m_startPoint + (segDir * tmin);
            collResult.m_hasCollision = true;
            return true;
        }
	}; // end Components
}; // end PE