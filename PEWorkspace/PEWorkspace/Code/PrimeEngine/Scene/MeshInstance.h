#ifndef __pe_meshinstance_h__
#define __pe_meshinstance_h__

#define NOMINMAX
// API Abstraction
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

// Outer-Engine includes
#include <assert.h>

// Inter-Engine includes
#include "PrimeEngine/APIAbstraction/Effect/Effect.h"

// Sibling/Children includes
#include "Mesh.h"

namespace PE {
namespace Components {

struct MeshInstance : public Component
{
	PE_DECLARE_CLASS(MeshInstance);

	// Constructor -------------------------------------------------------------
	MeshInstance(PE::GameContext &context, PE::MemoryArena arena, Handle hMyself) ;

	void initFromFile(const char *assetName, const char *assetPackage,
		int &threadOwnershipMask);

	void initFromRegisteredAsset(const PE::Handle &h);

	Mesh* getMesh();

	virtual ~MeshInstance(){}

	virtual void addDefaultComponents();

	bool hasSkinWeights();

    bool m_culledOut;
	Handle m_hAsset;

	int m_skinDebugVertexId;

	const char* m_assetName; // added for M1
	Vector3 minPoint = Vector3(0.0f, 0.0f, 0.0f);
	Vector3 maxPoint = Vector3(0.0f, 0.0f, 0.0f);
};

}; // namespace Components
}; // namespace PE
#endif
