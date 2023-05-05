// API Abstraction
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

#include "MeshManager.h"
// Outer-Engine includes

// Inter-Engine includes
#include "PrimeEngine/FileSystem/FileReader.h"
#include "PrimeEngine/APIAbstraction/GPUMaterial/GPUMaterialSet.h"
#include "PrimeEngine/PrimitiveTypes/PrimitiveTypes.h"
#include "PrimeEngine/APIAbstraction/Texture/Texture.h"
#include "PrimeEngine/APIAbstraction/Effect/EffectManager.h"
#include "PrimeEngine/APIAbstraction/GPUBuffers/VertexBufferGPUManager.h"
#include "PrimeEngine/../../GlobalConfig/GlobalConfig.h"

#include "PrimeEngine/Geometry/SkeletonCPU/SkeletonCPU.h"

#include "PrimeEngine/Scene/RootSceneNode.h"

#include "Light.h"

#include "NavMesh.h" // added for M1
#include "WallManager.h" // added for M3

// Sibling/Children includes

#include "MeshInstance.h"
#include "Skeleton.h"
#include "SceneNode.h"
#include "DrawList.h"
#include "SH_DRAW.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"

namespace PE {
namespace Components{

PE_IMPLEMENT_CLASS1(MeshManager, Component);
MeshManager::MeshManager(PE::GameContext &context, PE::MemoryArena arena, Handle hMyself)
	: Component(context, arena, hMyself)
	, m_assets(context, arena, 256)
{
}

PE::Handle MeshManager::getAsset(const char *asset, const char *package, int &threadOwnershipMask)
{
	char key[StrTPair<Handle>::StrSize];
	sprintf(key, "%s/%s", package, asset);
	
	int index = m_assets.findIndex(key);
	if (index != -1)
	{
		return m_assets.m_pairs[index].m_value;
	}
	Handle h;

	if (StringOps::endswith(asset, "skela"))
	{
		PE::Handle hSkeleton("Skeleton", sizeof(Skeleton));
		Skeleton *pSkeleton = new(hSkeleton) Skeleton(*m_pContext, m_arena, hSkeleton);
		pSkeleton->addDefaultComponents();

		pSkeleton->initFromFiles(asset, package, threadOwnershipMask);
		h = hSkeleton;
	}
	else if (StringOps::endswith(asset, "mesha"))
	{
		MeshCPU mcpu(*m_pContext, m_arena);
		mcpu.ReadMesh(asset, package, "");
		
		PE::Handle hMesh("Mesh", sizeof(Mesh));
		Mesh *pMesh = new(hMesh) Mesh(*m_pContext, m_arena, hMesh);
		pMesh->addDefaultComponents();

		pMesh->loadFromMeshCPU_needsRC(mcpu, threadOwnershipMask);

		// navmesh code
		std::string assetName = std::string(asset);
		if (assetName.find("NavPlane") != std::string::npos)
		{
			IndexBufferCPU* indexBuffer = nullptr;
			PositionBufferCPU* positionBuffer = nullptr;
			VertexBufferGPU* vertexBuff = nullptr;
			Array<PrimitiveTypes::Float32> m_values;


			indexBuffer = mcpu.m_hIndexBufferCPU.getObject<IndexBufferCPU>();
			positionBuffer = mcpu.m_hPositionBufferCPU.getObject<PositionBufferCPU>();

			for (PrimitiveTypes::UInt32 i = 0; i < indexBuffer->m_values.m_size; i+=3) {

				PE::Handle hCell("Cell", sizeof(Cell));
				Cell* pCell = new(hCell) Cell(*m_pContext, m_arena, hCell);

				// filling cell in
				for (int pos = 0; pos < 3; pos++)
				{
					int vertexID = indexBuffer->m_values[i + pos];

					pCell->verts[pos].m_x = positionBuffer->m_values[vertexID * 3 + 0];
					pCell->verts[pos].m_y = positionBuffer->m_values[vertexID * 3 + 1];
					pCell->verts[pos].m_z = positionBuffer->m_values[vertexID * 3 + 2];
				}
				//vert2 = indexBuffer->m_values[i];
				//vert3 = positionBuffer->m_values[vert2 * 3];
				//if (counter == 0)
				//{
				//	vert1 = indexBuffer->m_values[i];
				//}

				m_pContext->getNavMesh()->addComponent(hCell); // adds it in at index 1, not index 0. 
			}

			m_pContext->getNavMesh()->connectGraph();


		}
		else // store bounding box values
		{
			PositionBufferCPU* positionBuffer = nullptr;
			Array<PrimitiveTypes::Float32> m_values;

			positionBuffer = pMesh->m_hPositionBufferCPU.getObject<PositionBufferCPU>();

			Vector3 min = Vector3(FLT_MAX, FLT_MAX, FLT_MAX);
			Vector3 max = Vector3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			int counter = 0;
			for (int i = 0; i < positionBuffer->m_values.m_size; i++) {
				float currNumber = positionBuffer->m_values[i];
				if (counter == 0) // x y z
				{
					if (currNumber > max.getX())
					{
						max.m_x = currNumber;
					}
					if (currNumber < min.getX())
					{
						min.m_x = currNumber;
					}
				}
				if (counter == 1)
				{
					if (currNumber > max.getY())
					{
						max.m_y = currNumber;

					}
					if (currNumber < min.getY())
					{
						min.m_y = currNumber;
					}


				}
				if (counter == 2)
				{
					if (currNumber > max.getZ())
					{
						max.m_z = currNumber;
					}

					if (currNumber < min.getZ())
					{
						min.m_z = currNumber;
					}

				}
				counter++;
				if (counter == 3)
				{
					counter = 0;
				}

			}


			pMesh->firstPoint = Vector3(min.getX(), min.getY(), min.getZ()); // min min min
			pMesh->secondPoint = Vector3(min.getX(), min.getY(), max.getZ()); // min min max
			pMesh->thirdPoint = Vector3(min.getX(), max.getY(), min.getZ()); // min max min
			pMesh->fourthPoint = Vector3(min.getX(), max.getY(), max.getZ()); // min max max

			pMesh->fifthPoint = Vector3(max.getX(), min.getY(), min.getZ()); // max min min
			pMesh->sixthPoint = Vector3(max.getX(), min.getY(), max.getZ()); // max min max
			pMesh->seventhPoint = Vector3(max.getX(), max.getY(), min.getZ()); // SHOULD BE max max min, there was two of fourthPoint (min,max,max), pMesh->seventhPoint = Vector3(min.getX(), max.getY(), max.getZ());
			pMesh->eighthPoint = Vector3(max.getX(), max.getY(), max.getZ()); // max max max

			pMesh->maxPoint = pMesh->eighthPoint;
			pMesh->minPoint = pMesh->firstPoint;

			positionBuffer->m_values.clear();
			m_values.clear();
			// end bounding box code
		}

		//if (assetName.find("pplane") != std::string::npos)
		//{
		//	Handle hPC("Wall_Component", sizeof(WallComponent));
		//	WallComponent* pPC = new(hPC) WallComponent(*m_pContext, m_arena, hPC, pMesh);
		//	pPC->addDefaultComponents();
		//	//pPC->type = true; // not a sphere since no skeleton
		//	//pPC->mSceneNode = pMainSN;
		//	//pPC->mRotateScene = pRotateSN;
		//	addComponent(pPC);
		//}





#if PE_API_IS_D3D11
		// todo: work out how lods will work
		//scpu.buildLod();
#endif
        // generate collision volume here. or you could generate it in MeshCPU::ReadMesh()
        pMesh->m_performBoundingVolumeCulling = true; // will now perform tests for this mesh

		h = hMesh;
	}


	PEASSERT(h.isValid(), "Something must need to be loaded here");

	RootSceneNode::Instance()->addComponent(h);
	m_assets.add(key, h);
	return h;
}

void MeshManager::registerAsset(const PE::Handle &h)
{
	static int uniqueId = 0;
	++uniqueId;
	char key[StrTPair<Handle>::StrSize];
	sprintf(key, "__generated_%d", uniqueId);
	
	int index = m_assets.findIndex(key);
	PEASSERT(index == -1, "Generated meshes have to be unique");
	
	RootSceneNode::Instance()->addComponent(h);
	m_assets.add(key, h);
}

}; // namespace Components
}; // namespace PE
