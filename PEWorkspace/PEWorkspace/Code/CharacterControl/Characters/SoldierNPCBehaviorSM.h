#ifndef _PE_SOLDIER_NPC_BEHAVIOR_SM_H_
#define _PE_SOLDIER_NPC_BEHAVIOR_SM_H_


#include "PrimeEngine/Events/Component.h"

#include "../Events/Events.h"
#include <vector>

namespace CharacterControl{

namespace Components {

// movement state machine talks to associated animation state machine
struct SoldierNPCBehaviorSM : public PE::Components::Component
{
	PE_DECLARE_CLASS(SoldierNPCBehaviorSM);
	
	enum States
	{
		IDLE, // stand in place
		WAITING_FOR_WAYPOINT, // have a name of waypoint to go to, but it has not been loaded yet
		PATROLLING_WAYPOINTS,
		NAVMESH, // added for milestone
	};


	SoldierNPCBehaviorSM(PE::GameContext &context, PE::MemoryArena arena, PE::Handle hMyself, PE::Handle hMovementSM);

	void start();

	//////////////////////////////////////////////////////////////////////////
	// Component API and Event handlers
	//////////////////////////////////////////////////////////////////////////
	//
	virtual void addDefaultComponents();
	//
	PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_SoldierNPCMovementSM_Event_TARGET_REACHED)
	virtual void do_SoldierNPCMovementSM_Event_TARGET_REACHED(PE::Events::Event *pEvt);
	//
	PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_UPDATE)
	virtual void do_UPDATE(PE::Events::Event *pEvt);

	PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_PRE_RENDER_needsRC)
	void do_PRE_RENDER_needsRC(PE::Events::Event *pEvt);

	PE::Handle m_hMovementSM;

	bool m_havePatrolWayPoint;
	char m_curPatrolWayPoint[32];
	States m_state;

	// added below for milestone

	void updatePath(bool isChasingPlayer);
	void checkPlayerVisibility(Vector3 soldierPos, Vector3 soldierForward, Vector3 playerPosition);
	bool checkWalls(Vector3 soldierPos, Vector3 soldierForward, Vector3 playerPosition);
	void setIsPlayerSeen(bool b) { isPlayerSeen = b; }
	Vector3 findClosestCell(Vector3 cellPos);

	std::vector<Vector3> currPath;
	int pathLen;
	int currIndex = 0;
	Vector3 playerPos; // can't make this a cell unfortunately
	bool isPlayerSeen = false; // is the player visible to the soldier? if so, chase it. otherwise, move randomly
	bool panicPathing = false; // if for some reason the state machine gets messed up, randomly select nodes

	float randColor1 = 1.0f;
	float randColor2 = 1.0f;
	float randColor3 = 1.0f;
};

};
};


#endif


