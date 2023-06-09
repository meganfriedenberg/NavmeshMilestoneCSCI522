#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

#include "PrimeEngine/Lua/LuaEnvironment.h"
#include "PrimeEngine/Scene/DebugRenderer.h"
#include "../ClientGameObjectManagerAddon.h"
#include "../CharacterControlContext.h"
#include "SoldierNPCMovementSM.h"
#include "SoldierNPCAnimationSM.h"
#include "SoldierNPCBehaviorSM.h"
#include "SoldierNPC.h"
#include "PrimeEngine/Scene/SceneNode.h"
#include "PrimeEngine/Render/IRenderer.h"

#include "NavMesh.h"
#include "WallManager.h"

#include "PrimeEngine/Scene/CameraSceneNode.h"
#include "PrimeEngine/Scene/CameraManager.h"
using namespace PE::Components;
using namespace PE::Events;
using namespace CharacterControl::Events;

namespace CharacterControl{

namespace Components{

PE_IMPLEMENT_CLASS1(SoldierNPCBehaviorSM, Component);

SoldierNPCBehaviorSM::SoldierNPCBehaviorSM(PE::GameContext &context, PE::MemoryArena arena, PE::Handle hMyself, PE::Handle hMovementSM) 
: Component(context, arena, hMyself)
, m_hMovementSM(hMovementSM)
{

}

void SoldierNPCBehaviorSM::start()
{
	randColor1 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 1.0f));
	randColor2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 1.0f));
	randColor3 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 1.0f));

	if (m_havePatrolWayPoint)
	{
		m_state = WAITING_FOR_WAYPOINT; // will update on next do_UPDATE()
	}
	else
	{
		m_state = IDLE; // stand in place

		PE::Handle h("SoldierNPCMovementSM_Event_STOP", sizeof(SoldierNPCMovementSM_Event_STOP));
		SoldierNPCMovementSM_Event_STOP *pEvt = new(h) SoldierNPCMovementSM_Event_STOP();

		m_hMovementSM.getObject<Component>()->handleEvent(pEvt);
		// release memory now that event is processed
		h.release();
		
	}	
}

void SoldierNPCBehaviorSM::addDefaultComponents()
{
	Component::addDefaultComponents();

	PE_REGISTER_EVENT_HANDLER(SoldierNPCMovementSM_Event_TARGET_REACHED, SoldierNPCBehaviorSM::do_SoldierNPCMovementSM_Event_TARGET_REACHED);
	PE_REGISTER_EVENT_HANDLER(Event_UPDATE, SoldierNPCBehaviorSM::do_UPDATE);

	PE_REGISTER_EVENT_HANDLER(Event_PRE_RENDER_needsRC, SoldierNPCBehaviorSM::do_PRE_RENDER_needsRC);
}

void SoldierNPCBehaviorSM::do_SoldierNPCMovementSM_Event_TARGET_REACHED(PE::Events::Event *pEvt)
{
	PEINFO("SoldierNPCBehaviorSM::do_SoldierNPCMovementSM_Event_TARGET_REACHED\n");


	if (m_state == NAVMESH)
	{
		m_state = NAVMESH;

		currIndex++;

		if (currIndex >= currPath.size() || currIndex == pathLen)
		{
			panicPathing = true;
			updatePath(false);
		}

		if (currIndex < currPath.size())
		{
			OutputDebugStringA("navMove\n");
			PE::Handle h("SoldierNPCMovementSM_Event_MOVE_TO", sizeof(SoldierNPCMovementSM_Event_MOVE_TO));
			Events::SoldierNPCMovementSM_Event_MOVE_TO* pEvt = new(h) SoldierNPCMovementSM_Event_MOVE_TO(currPath[currIndex]);
			pEvt->m_running = false; // TODO change this


			m_hMovementSM.getObject<Component>()->handleEvent(pEvt);
			// release memory now that event is processed
			h.release();
		}

	}




	if (m_state == PATROLLING_WAYPOINTS)
	{
		ClientGameObjectManagerAddon *pGameObjectManagerAddon = (ClientGameObjectManagerAddon *)(m_pContext->get<CharacterControlContext>()->getGameObjectManagerAddon());
		if (pGameObjectManagerAddon)
		{
			// search for waypoint object
			WayPoint *pWP = pGameObjectManagerAddon->getWayPoint(m_curPatrolWayPoint);
			if (pWP && StringOps::length(pWP->m_nextWayPointName) > 0)
			{
				// have next waypoint to go to
				pWP = pGameObjectManagerAddon->getWayPoint(pWP->m_nextWayPointName);
				if (pWP)
				{
					StringOps::writeToString(pWP->m_name, m_curPatrolWayPoint, 32);

					m_state = PATROLLING_WAYPOINTS;
					OutputDebugStringA("here??\n");
					PE::Handle h("SoldierNPCMovementSM_Event_MOVE_TO", sizeof(SoldierNPCMovementSM_Event_MOVE_TO));
					Events::SoldierNPCMovementSM_Event_MOVE_TO *pEvt = new(h) SoldierNPCMovementSM_Event_MOVE_TO(pWP->m_base.getPos());

					m_hMovementSM.getObject<Component>()->handleEvent(pEvt);
					// release memory now that event is processed
					h.release();
				}
			}
			else
			{
				m_state = IDLE;
				// no need to send the event. movement state machine will automatically send event to animation state machine to play idle animation
			}
		}
	}
}

// this event is executed when thread has RC
void SoldierNPCBehaviorSM::do_PRE_RENDER_needsRC(PE::Events::Event *pEvt)
{
	Event_PRE_RENDER_needsRC *pRealEvent = (Event_PRE_RENDER_needsRC *)(pEvt);
	if (m_havePatrolWayPoint)
	{
		char buf[80];
		sprintf(buf, "Patrol Waypoint: %s",m_curPatrolWayPoint);
		SoldierNPC *pSol = getFirstParentByTypePtr<SoldierNPC>();
		PE::Handle hSoldierSceneNode = pSol->getFirstComponentHandle<PE::Components::SceneNode>();
		Matrix4x4 base = hSoldierSceneNode.getObject<PE::Components::SceneNode>()->m_worldTransform;
		
		DebugRenderer::Instance()->createTextMesh(
			buf, false, false, true, false, 0,
			base.getPos(), 0.01f, pRealEvent->m_threadOwnershipMask);
		
		{
			//we can also construct points ourself
			bool sent = false;
			ClientGameObjectManagerAddon *pGameObjectManagerAddon = (ClientGameObjectManagerAddon *)(m_pContext->get<CharacterControlContext>()->getGameObjectManagerAddon());
			if (pGameObjectManagerAddon)
			{
				WayPoint *pWP = pGameObjectManagerAddon->getWayPoint(m_curPatrolWayPoint);
				if (pWP)
				{
					Vector3 target = pWP->m_base.getPos();
					Vector3 pos = base.getPos();
					Vector3 color(1.0f, 1.0f, 0);
					Vector3 linepts[] = {pos, color, target, color};
					
					//DebugRenderer::Instance()->createLineMesh(true, base,  &linepts[0].m_x, 2, 0);// send event while the array is on the stack
					sent = true;
				}
			}
			//if (!sent) // if for whatever reason we didnt retrieve waypoint info, send the event with transform only
				//DebugRenderer::Instance()->createLineMesh(true, base, NULL, 0, 0);// send event while the array is on the stack
		}
	}
	if (currPath.size() > 0)
	{
		SoldierNPC* pSol = getFirstParentByTypePtr<SoldierNPC>();
		PE::Handle hSoldierSceneNode = pSol->getFirstComponentHandle<PE::Components::SceneNode>();
		Matrix4x4 base = hSoldierSceneNode.getObject<PE::Components::SceneNode>()->m_worldTransform;
		//DebugRenderer::Instance()->createLineTwoPoints(base.getPos(), currPath[0], Vector3(1.0f, 1.0f, 1.0f), 0, 1.0f);
		for (int i = 0; i < currPath.size()-1; i++)
		{
			DebugRenderer::Instance()->createLineTwoPoints(currPath[i], currPath[i+1], Vector3(randColor1, randColor2, randColor3), 0, 1.0f);
		}
	}
}

void SoldierNPCBehaviorSM::do_UPDATE(PE::Events::Event *pEvt)
{
	SoldierNPC* pSol = getFirstParentByTypePtr<SoldierNPC>();
	PE::Handle hSoldierSceneNode = pSol->getFirstComponentHandle<PE::Components::SceneNode>();
	Matrix4x4 base = hSoldierSceneNode.getObject<PE::Components::SceneNode>()->m_worldTransform;

	// get the cell the player is over
	CameraSceneNode* pCamera = CameraManager::Instance()->getActiveCamera()->getCamSceneNode();
	playerPos = pCamera->m_base.getPos();
	playerPos.m_y = 0.0f; // navmesh is flat, camera height doesn't matter, but if it DOES matter, change this

	float playerDistanceFromSoldier = (base.getPos() - playerPos).lengthSqr();

	checkPlayerVisibility(base.getPos(), base.getN(), playerPos);
	if ((m_pContext)->getNavMesh()->getCurrentCell(playerPos) == nullptr)
	{
		isPlayerSeen = false;
	}

	if (m_state == WAITING_FOR_WAYPOINT)
	{
		m_state = NAVMESH;

		updatePath(false);

		OutputDebugStringA("waitedMoved\n");
		PE::Handle h("SoldierNPCMovementSM_Event_MOVE_TO", sizeof(SoldierNPCMovementSM_Event_MOVE_TO));
		Events::SoldierNPCMovementSM_Event_MOVE_TO* pEvt = new(h) SoldierNPCMovementSM_Event_MOVE_TO(currPath[currIndex]);

		pEvt->m_running = isPlayerSeen;

		m_hMovementSM.getObject<Component>()->handleEvent(pEvt);
		// release memory now that event is processed
		h.release();


		//if (m_havePatrolWayPoint)
		//{
		//	ClientGameObjectManagerAddon *pGameObjectManagerAddon = (ClientGameObjectManagerAddon *)(m_pContext->get<CharacterControlContext>()->getGameObjectManagerAddon());
		//	if (pGameObjectManagerAddon)
		//	{
		//		// search for waypoint object
		//		WayPoint *pWP = pGameObjectManagerAddon->getWayPoint(m_curPatrolWayPoint);
		//		if (pWP)
		//		{
		//			m_state = PATROLLING_WAYPOINTS;
		//			PE::Handle h("SoldierNPCMovementSM_Event_MOVE_TO", sizeof(SoldierNPCMovementSM_Event_MOVE_TO));
		//			Events::SoldierNPCMovementSM_Event_MOVE_TO *pEvt = new(h) SoldierNPCMovementSM_Event_MOVE_TO(pWP->m_base.getPos());

		//			m_hMovementSM.getObject<Component>()->handleEvent(pEvt);
		//			// release memory now that event is processed
		//			h.release();
		//		}
		//	}
		//}
		//else
		//{
		//	// should not happen, but in any case, set state to idle
		//	m_state = IDLE;

		//	PE::Handle h("SoldierNPCMovementSM_Event_STOP", sizeof(SoldierNPCMovementSM_Event_STOP));
		//	SoldierNPCMovementSM_Event_STOP *pEvt = new(h) SoldierNPCMovementSM_Event_STOP();

		//	m_hMovementSM.getObject<Component>()->handleEvent(pEvt);
		//	// release memory now that event is processed
		//	h.release();
		//}
	}
	if (m_state == NAVMESH) // if for some reason this gets locked at the start from out of order execution
	{
		if (currIndex >= currPath.size())
		{
			updatePath(false);
		}
		else if (isPlayerSeen)
		{
			updatePath(true);
			OutputDebugStringA("moving\n");
			if (currIndex >= currPath.size())
			{
				panicPathing = true;
				updatePath(false);
			}
			if (currIndex < currPath.size())
			{
				PE::Handle h("SoldierNPCMovementSM_Event_MOVE_TO", sizeof(SoldierNPCMovementSM_Event_MOVE_TO));
				Events::SoldierNPCMovementSM_Event_MOVE_TO* pEvt = new(h) SoldierNPCMovementSM_Event_MOVE_TO(currPath[currIndex]);

				pEvt->m_running = isPlayerSeen;

				m_hMovementSM.getObject<Component>()->handleEvent(pEvt);
				// release memory now that event is processed
				h.release();
			}

		}
	}
	if (m_state == IDLE)
	{
		m_state = WAITING_FOR_WAYPOINT;
	}
}

void CharacterControl::Components::SoldierNPCBehaviorSM::updatePath(bool isChasingPlayer)
{
	SoldierNPC* pSol = getFirstParentByTypePtr<SoldierNPC>();
	PE::Handle hSoldierSceneNode = pSol->getFirstComponentHandle<PE::Components::SceneNode>();
	Matrix4x4 base = hSoldierSceneNode.getObject<PE::Components::SceneNode>()->m_worldTransform;

	// get the cell the player is over
	CameraSceneNode* pCamera = CameraManager::Instance()->getActiveCamera()->getCamSceneNode();
	playerPos = pCamera->m_base.getPos();
	playerPos.m_y = 0.0f; // navmesh is flat, camera height doesn't matter, but if it DOES matter, change this
	 
	bool validPlayerPos = true;
	Cell* playerCell = (m_pContext)->getNavMesh()->getCurrentCell(playerPos);
	if (playerCell == nullptr || panicPathing)
	{
		// just randomly walk since the player either doesn't have a valid position, or out of order execution landed us here
		playerCell = ((m_pContext)->getNavMesh()->getCell(rand() % (m_pContext)->getNavMesh()->totalCells + 1));
		validPlayerPos = false;
	}

	// Get the starting Vector3
	Vector3 start = base.getPos();

	
	Cell* soldierCell = (m_pContext)->getNavMesh()->getCurrentCell(start);
	if (soldierCell == nullptr)
	{
		Vector3 newCellPos = findClosestCell(start);
		soldierCell = (m_pContext)->getNavMesh()->getCurrentCell(newCellPos);
	}


	// if you only want A* uncomment this
	
	if (!validPlayerPos || panicPathing || !isPlayerSeen)
	{
		if (currPath.size() < 3 || currIndex == currPath.size())
		{
			std::vector<Cell*> path = (m_pContext)->getNavMesh()->findCellPath(soldierCell, playerCell, playerCell->verts[1]);
			currPath.clear();
			for (int i = 0; i < path.size() - 1; i++)
			{
				Vector3 triangleCenter = (m_pContext)->getNavMesh()->getTriangleCenter(path[i]->verts[0], path[i]->verts[1], path[i]->verts[2]);
				currPath.push_back(triangleCenter);
			}
			currIndex = 1;
		}
		panicPathing = false;
	}
	if (validPlayerPos && isPlayerSeen)
	{
		currPath.clear();
		currPath = (m_pContext)->getNavMesh()->findPath(start, playerPos, pathLen); // then call simple stupid funnel
		currIndex = 1;
	}

	if(!currPath.empty() && currIndex < currPath.size())
		while ((base.getPos() - currPath[currIndex]).lengthSqr() < 0.01f && currIndex < currPath.size() - 1)
		{
			currIndex++;
			if (currIndex >= currPath.size())
			{
				break;
			}
		}


}

void CharacterControl::Components::SoldierNPCBehaviorSM::checkPlayerVisibility(Vector3 soldierPos, Vector3 soldierForward, Vector3 playerPosition)
{
	// see if a player is within 30 degrees of sight

	Vector3 uDirection = playerPosition - soldierPos; // from the soldier to the player
	Vector3 uNormalized = uDirection;
	uNormalized.normalize();

	float angle = acos(uNormalized.dotProduct(soldierForward));

	if (angle < (3.1415f / 6)) // within 30 degrees range
	{
		bool wallCheck = checkWalls(soldierPos, soldierForward, playerPos);
		if (wallCheck)
		{
			CharacterControl::Components::SoldierNPCBehaviorSM::setIsPlayerSeen(true);
			OutputDebugStringA("PLAYER IS SEEN\n");
		}
		else
		{
			CharacterControl::Components::SoldierNPCBehaviorSM::setIsPlayerSeen(false);
			OutputDebugStringA("CAN'T SEE THROUGH WALLS\n");
		}
	}
	else
	{
		CharacterControl::Components::SoldierNPCBehaviorSM::setIsPlayerSeen(false);
	}

}

bool CharacterControl::Components::SoldierNPCBehaviorSM::checkWalls(Vector3 soldierPos, Vector3 soldierForward, Vector3 playerPosition)
{
	WallManager* pWallManager = (m_pContext)->getWallManager();
	LineSegment fromSoldierToPlayer;
	fromSoldierToPlayer.m_startPoint = soldierPos;
	fromSoldierToPlayer.m_endPoint = playerPosition;

	Vector3 hitPoint;

	bool bHitWall = pWallManager->RayCast(fromSoldierToPlayer, &hitPoint, true); // returns true if the ray hits any walls
	

	return !(bHitWall); // return false if the check fails

}

Vector3 CharacterControl::Components::SoldierNPCBehaviorSM::findClosestCell(Vector3 cellPos)
{
	// will return a valid Vector3
	Vector3 newPos = cellPos;
	while ((m_pContext)->getNavMesh()->getCurrentCell(newPos) == nullptr)
	{
		float temp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		temp *= 0.02f;
		temp -= 0.01f;
		newPos.m_x += temp;

		temp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		temp *= 0.02f;
		temp -= 0.01f;
		newPos.m_z += temp;


	}

	return newPos;
}

}}




