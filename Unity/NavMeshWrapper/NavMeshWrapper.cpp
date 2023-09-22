#include "NavMeshWrapper.h"
#include <stdio.h>
#include <stdint.h>
#include <memory>
#include <fstream>
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourCommon.h"
#include "fastlz.h"

#ifdef WIN32
#ifndef NDEBUG
	#define ENABLE_LOG
#endif // !NDEBUG
#endif

FILE* fp;
static const int MAX_POLYS = 256;

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};


/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,
};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK = 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM = 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR = 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP = 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED = 0x10,		// Disabled polygon
	SAMPLE_POLYFLAGS_ALL = 0xffff	// All abilities.
};
//------------------------- TempObstacles Begin-----------------------------
static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

static const int MAX_LAYERS = 32;

struct TileCacheData
{
	unsigned char* data;
	int dataSize;
};

static const int TILECACHESET_MAGIC = 'T' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'TSET';
static const int TILECACHESET_VERSION = 1;

struct TileCacheSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams meshParams;
	dtTileCacheParams cacheParams;
};

struct TileCacheTileHeader
{
	dtCompressedTileRef tileRef;
	int dataSize;
};

static const int EXPECTED_LAYERS_PER_TILE = 4;

struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize * 1.05f);
	}

	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
		unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void* const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}

	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
		unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};


struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	size_t capacity;
	size_t top;
	size_t high;

	LinearAllocator(const size_t cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}

	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const size_t cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}

	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}

	virtual void* alloc(const size_t size)
	{
		if (!buffer)
			return 0;
		if (top + size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

struct MeshProcess : public dtTileCacheMeshProcess
{
	inline MeshProcess()
	{
	}

	virtual void process(struct dtNavMeshCreateParams* params,
		unsigned char* polyAreas, unsigned short* polyFlags)
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->polyCount; ++i)
		{
			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
				polyAreas[i] = SAMPLE_POLYAREA_GROUND;

			if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
				polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
				polyAreas[i] == SAMPLE_POLYAREA_ROAD)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}
	}
};

LinearAllocator* m_talloc = new LinearAllocator(32000);
FastLZCompressor* m_tcomp = new FastLZCompressor;
MeshProcess* m_tmproc = new MeshProcess;
//------------------------- TempObstacles End-------------------------------


class NavMeshInstance
{
public:
	NavMeshInstance()
	{
		m_navMesh = nullptr;
		m_navQuery = nullptr;
		m_crowd = nullptr;
		m_tileCache = nullptr;
	}

	~NavMeshInstance()
	{
		if (m_navMesh)
		{
			dtFreeNavMesh(m_navMesh);
			m_navMesh = nullptr;
		}
		if (m_crowd)
		{
			dtFreeCrowd(m_crowd);
			m_crowd = nullptr;
		}
		if (m_navQuery)
		{
			dtFreeNavMeshQuery(m_navQuery);
			m_navQuery = nullptr;
		}
		if (m_tileCache)
		{
			dtFreeTileCache(m_tileCache);
			m_tileCache = nullptr;
		}
	}

public:
	dtNavMesh* m_navMesh;
	dtNavMeshQuery* m_navQuery;
	dtCrowd* m_crowd;
	dtTileCache* m_tileCache;
	float m_straightPath[MAX_POLYS * 3];
	int32_t m_nStraightPath = 0;
};


#ifdef ENABLE_LOG
	#define LOG(fmt, ...) fprintf(fp, fmt"\r\n", ##__VA_ARGS__)
#else
	#define LOG
#endif

#include <list>
std::list<NavMeshInstance*> g_navmesh_insts;

NavMeshInstance* LoadNavMesh(unsigned char* pucValue, unsigned int uiLength)
{
#ifdef ENABLE_LOG
	fp = fopen("navmesh.log", "w+");
#endif
	LOG("--------------------------------------------LoadNavMesh--------------------------------------------");
	dtNavMesh *g_navMesh = dtAllocNavMesh();
	if (!g_navMesh)
	{
		LOG("Could not create Detour navmesh");
		return nullptr;
	}

	dtStatus status;

	unsigned char* data = (unsigned char*)dtAlloc(uiLength, DT_ALLOC_PERM);
	memcpy(data, pucValue, uiLength);
	status = g_navMesh->init(data, uiLength, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		LOG("Could not init Detour navmesh");
		return nullptr;
	}

	dtNavMeshQuery *g_navQuery = dtAllocNavMeshQuery();
	status = g_navQuery->init(g_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		LOG("Could not init Detour navmesh query");
		return nullptr;
	}
	LOG("LoadNavMesh:%d", uiLength);

	dtCrowd *g_crowd = dtAllocCrowd();

	NavMeshInstance* navMeshInstance = new NavMeshInstance();
	navMeshInstance->m_crowd = g_crowd;
	navMeshInstance->m_navMesh = g_navMesh;
	navMeshInstance->m_navQuery = g_navQuery;
	g_navmesh_insts.push_back(navMeshInstance);
	return navMeshInstance;
}

NavMeshInstance *LoadObstaclesMesh(unsigned char* pucValue, unsigned int uiLength)
{
#ifdef ENABLE_LOG
	fp = fopen("navmesh.log", "w+");
#endif
	LOG("--------------------------------------------LoadObstaclesMesh--------------------------------------------");

	// Read header.
	TileCacheSetHeader header;
	int pos = 0;
	memcpy(&header, pucValue + pos, sizeof(TileCacheSetHeader));
	pos += sizeof(TileCacheSetHeader);
	if (header.magic != TILECACHESET_MAGIC)
	{
		return nullptr;
	}
	if (header.version != TILECACHESET_VERSION)
	{
		return nullptr;
	}

	dtNavMesh *g_navMesh = dtAllocNavMesh();
	if (!g_navMesh)
	{
		return nullptr;
	}
	dtStatus status = g_navMesh->init(&header.meshParams);
	if (dtStatusFailed(status))
	{
		return nullptr;
	}

	m_talloc = new LinearAllocator(32000);
	m_tcomp = new FastLZCompressor;
	m_tmproc = new MeshProcess;
	dtTileCache *g_tileCache = dtAllocTileCache();
	if (!g_tileCache)
	{
		fclose(fp);
		return nullptr;
	}
	status = g_tileCache->init(&header.cacheParams, m_talloc, m_tcomp, m_tmproc);
	if (dtStatusFailed(status))
	{
		return nullptr;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		TileCacheTileHeader tileHeader;
		memcpy(&tileHeader, pucValue + pos, sizeof(tileHeader));
		pos += sizeof(tileHeader);
		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		memcpy(data, pucValue + pos, tileHeader.dataSize);
		pos += tileHeader.dataSize;

		dtCompressedTileRef tile = 0;
		dtStatus addTileStatus = g_tileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);
		if (dtStatusFailed(addTileStatus))
		{
			dtFree(data);
		}

		if (tile)
			g_tileCache->buildNavMeshTile(tile, g_navMesh);
	}

	dtNavMeshQuery *g_navQuery = dtAllocNavMeshQuery();
	status = g_navQuery->init(g_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		LOG("Could not init Detour navmesh query");
		return nullptr;
	}
	LOG("LoadObstaclesMesh:%d", uiLength);

	dtCrowd *g_crowd = dtAllocCrowd();


	NavMeshInstance* navMeshInstance = new NavMeshInstance();
	navMeshInstance->m_crowd = g_crowd;
	navMeshInstance->m_navMesh = g_navMesh;
	navMeshInstance->m_navQuery = g_navQuery;
	navMeshInstance->m_tileCache = g_tileCache;

	g_navmesh_insts.push_back(navMeshInstance);

	return navMeshInstance;
}

int FindStraightPath(NavMeshInstance* inst, float startX, float startY, float endX, float endY)
{
	LOG("FindStraightPath Enter:start(%f, %f) end(%f, %f)", startX, startY, endX, endY);
	if (!inst->m_navQuery)
	{
		LOG("navQuery is nullptr");
		return 0;
	}
	
	float sPos[3] = { 0 };
	sPos[0] = -startX;
	sPos[1] = 0.f;
	sPos[2] = startY;

	float ePos[3] = { 0 };
	ePos[0] = -endX;
	ePos[1] = 0.f;
	ePos[2] = endY;

	dtQueryFilter m_filter;
	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);

	float m_polyPickExt[3] = { 3.0f, 4.0f, 3.0f };

	float m_fixedEPos[3];
	dtPolyRef m_polys[MAX_POLYS];
	unsigned char m_straightPathFlags[MAX_POLYS];
	dtPolyRef m_straightPathPolys[MAX_POLYS];

	memset(m_fixedEPos, 0, sizeof(m_fixedEPos));
	memset(m_polys, 0, sizeof(m_polys));
	memset(inst->m_straightPath, 0, sizeof(inst->m_straightPath));
	memset(m_straightPathFlags, 0, sizeof(m_straightPathFlags));
	memset(m_straightPathPolys, 0, sizeof(m_straightPathPolys));

	int32_t m_nPolys = 0;
	int32_t m_nStraightPathOptions = 0;
	dtPolyRef m_startRef = 0;
	dtPolyRef m_endRef = 0;

	inst->m_navQuery->findNearestPoly(sPos, m_polyPickExt, &m_filter, &m_startRef, 0);
	inst->m_navQuery->findNearestPoly(ePos, m_polyPickExt, &m_filter, &m_endRef, 0);
	inst->m_navQuery->findPath(m_startRef, m_endRef, sPos, ePos, &m_filter, m_polys, &m_nPolys, MAX_POLYS);

	inst->m_nStraightPath = 0;

	if (m_nPolys > 0)
	{
		// In case of partial path, make sure the end point is clamped to the last polygon.
		m_fixedEPos[0] = ePos[0];
		m_fixedEPos[1] = ePos[1];
		m_fixedEPos[2] = ePos[2];

		if (m_polys[m_nPolys - 1] != m_endRef)
			inst->m_navQuery->closestPointOnPoly(m_polys[m_nPolys - 1], ePos, m_fixedEPos, 0);

		inst->m_navQuery->findStraightPath(sPos, m_fixedEPos, m_polys, m_nPolys, inst->m_straightPath, m_straightPathFlags,
			m_straightPathPolys, &inst->m_nStraightPath, MAX_POLYS, m_nStraightPathOptions);

		if (inst->m_nStraightPath >= MAX_POLYS)
		{
			inst->m_nStraightPath = MAX_POLYS;
			LOG("straightPath out of bound of max polys.");
		}
	}
	LOG("FindStraightPath End:%d", inst->m_nStraightPath);
	return inst->m_nStraightPath;
}

bool GetPathPoint(NavMeshInstance* inst, int index, float& x, float& y)
{
	LOG("GetPathPoint:%d", index);

	if (index >= inst->m_nStraightPath)
		return false;

	auto startPtr = &inst->m_straightPath[index * 3];
	x = -startPtr[0];
	y = startPtr[2];

	LOG("GetPathPoint:%d %f %f", index, x, y);
	return true;
}

bool PathRaycast(NavMeshInstance* inst, float startX, float startY, float endX, float endY, float& hitX, float& hitY)
{
	hitX = 0;
	hitY = 0;
	float sPos[3] = { 0 };
	sPos[0] = -startX;
	sPos[1] = 0.f;
	sPos[2] = startY;

	float ePos[3] = { 0 };
	ePos[0] = -endX;
	ePos[1] = 0.f;
	ePos[2] = endY;

	dtQueryFilter m_filter;
	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);

	float m_polyPickExt[3] = { 2.0f, 4.0f, 2.0f };

	float m_hitNormal[3];
	memset(m_hitNormal, 0, sizeof(m_hitNormal));
	dtPolyRef m_polys[MAX_POLYS];

	dtPolyRef m_startRef = 0;
	inst->m_navQuery->findNearestPoly(sPos, m_polyPickExt, &m_filter, &m_startRef, 0);

	float t = 0;
	int m_npolys = 0;
	dtStatus status = inst->m_navQuery->raycast(m_startRef, sPos, ePos, &m_filter, &t, m_hitNormal, m_polys, &m_npolys, MAX_POLYS);
	bool success = dtStatusSucceed(status);
	if (sPos[0]==ePos[0]&&sPos[2]==ePos[2])
	{
		if ((m_npolys == 0 || !success))
		{
			// No hit
			hitX = sPos[0];
			hitY = sPos[2];
			return false;
		}
		else
		{
			return true;
		}
	}
	if (t > 1)
	{
		// No hit
		hitX = 0;
		hitY = 0;
		return false;
	}
	else
	{
		// Hit
		hitX = -(sPos[0] + (ePos[0] - sPos[0]) * t);
		hitY = sPos[2] + (ePos[2] - sPos[2]) * t;
		return true;
	}
}

void UnLoadNavMesh(NavMeshInstance* inst)
{
	if (inst)
	{
		auto itr = std::find(g_navmesh_insts.begin(), g_navmesh_insts.end(), inst);
		if (itr != g_navmesh_insts.end())
		{
			g_navmesh_insts.erase(itr);
			g_navmesh_insts.remove(inst);
		}
		delete inst;
	}
#ifdef ENABLE_LOG
	if (fp)
	{
		LOG("--------------------------------------------UnLoadNavMesh--------------------------------------------");
		fclose(fp);
		fp = nullptr;
	}
#endif
}


bool AddObstacles(NavMeshInstance* inst, float x, float y, float z, float radius, float height, unsigned int& id, bool update)
{
	if (inst->m_tileCache == nullptr)
		return false;
	float pos[3] = {-x,y,z};
	dtStatus status = ((dtTileCache*)inst->m_tileCache)->addObstacle(pos, radius, height, (dtObstacleRef*)&id);
	if ((status & DT_BUFFER_TOO_SMALL))
	{
		if (UpdateObstaclesMesh(inst))
		{
			status = ((dtTileCache*)inst->m_tileCache)->addObstacle(pos, radius, height, (dtObstacleRef*)&id);
			update = false;
		}
	}
	bool success = dtStatusSucceed(status);
	if (success)
	{
		if (update)
		{
			return UpdateObstaclesMesh(inst);
		}
	}
	else
	{
		printf("addObstacle fail:%d\n", status);
	}
	return success;
}
bool AddBoxObstacles(NavMeshInstance* inst, float minx, float miny, float minz, float maxx, float maxy, float maxz, unsigned int& id, bool update)
{
	if (inst->m_tileCache == nullptr)
		return false;
	float bmin[3] = { -maxx,miny,minz };
	float bmax[3] = { -minx,maxy,maxz };
	dtStatus status = ((dtTileCache*)inst->m_tileCache)->addBoxObstacle(bmin, bmax, (dtObstacleRef*)&id);
	if ((status & DT_BUFFER_TOO_SMALL))
	{
		if (UpdateObstaclesMesh(inst))
		{
			status = ((dtTileCache*)inst->m_tileCache)->addBoxObstacle(bmin, bmax, (dtObstacleRef*)&id);
			update = false;
		}
	}
	bool success = dtStatusSucceed(status);
	if (success)
	{
		if (update)
		{
			return UpdateObstaclesMesh(inst);
		}
	}
	else
	{
		printf("addObstacle fail:%d\n", status);
	}
	return success;
}
bool RemoveObstacles(NavMeshInstance* inst, unsigned int id, bool update)
{
	if (inst->m_tileCache == nullptr)
		return false;
	dtStatus status = ((dtTileCache*)inst->m_tileCache)->removeObstacle((dtObstacleRef)id);
	bool success = dtStatusSucceed(status);
	if (success)
	{
		if (update)
		{
			return UpdateObstaclesMesh(inst);
		}
	}
	else
	{
		printf("addObstacle fail:%d\n", status);
	}

	return success;
}

bool UpdateObstaclesMesh(NavMeshInstance* inst)
{
	if (inst->m_tileCache == nullptr || inst->m_navQuery == nullptr)
		return false;

	bool upToDate = false;
	dtNavMesh* navMeshQuery = (dtNavMesh*)(((dtNavMeshQuery*)inst->m_navQuery)->getAttachedNavMesh());
	while (!upToDate)
	{
		dtStatus status = ((dtTileCache*)inst->m_tileCache)->update(0, navMeshQuery, &upToDate);
		if (!dtStatusSucceed(status))
		{
			printf("tickUpdate fail:%d\n", status);
			return false;
		}
	}
	return true;
}

bool InitCrowd(NavMeshInstance* inst, int max_agent/* = 128*/, float agent_radius/*=0.7*/)
{
	if (inst->m_tileCache == nullptr || inst->m_navQuery == nullptr || inst->m_crowd == nullptr)
		return false;

	inst->m_crowd->init(max_agent, agent_radius, inst->m_navMesh);

	auto crowd = inst->m_crowd;
	// Make polygons with 'disabled' flag invalid.
	crowd->getEditableFilter(0)->setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);

	// Setup local avoidance params to different qualities.
	dtObstacleAvoidanceParams params;
	// Use mostly default settings, copy from dtCrowd.
	memcpy(&params, crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

	// Low (11)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 1;
	crowd->setObstacleAvoidanceParams(0, &params);

	// Medium (22)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 2;
	crowd->setObstacleAvoidanceParams(1, &params);

	// Good (45)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 3;
	crowd->setObstacleAvoidanceParams(2, &params);

	// High (66)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 3;
	params.adaptiveDepth = 3;

	crowd->setObstacleAvoidanceParams(3, &params);

	return true;
}
bool AddCrowdAgent(NavMeshInstance* inst, float x, float y, float z, float radius, float height, float maxAcceleration, float maxSpeed, unsigned int& id, int update_flag /*= 0*/)
{
	if (inst->m_tileCache == nullptr || inst->m_navQuery == nullptr || inst->m_crowd == nullptr)
		return false;
	auto crowd = inst->m_crowd;
	dtCrowdAgentParams ap;
	memset(&ap, 0, sizeof(ap));
	ap.radius = radius;
	ap.height = height;
	ap.maxAcceleration = maxAcceleration;
	ap.maxSpeed = maxSpeed;
	ap.collisionQueryRange = ap.radius * 12.0f;
	ap.pathOptimizationRange = ap.radius * 30.0f;
	ap.updateFlags = update_flag;
	ap.obstacleAvoidanceType = 3;
	ap.separationWeight = 2;

	float pos[3] = { -x,y,z };
	id = crowd->addAgent(pos, &ap);

	return true;
}

bool RemoveCrowdAgent(NavMeshInstance* inst, unsigned int id)
{
	if (inst->m_tileCache == nullptr || inst->m_navQuery == nullptr || inst->m_crowd == nullptr)
		return false;
	dtCrowd* crowd = inst->m_crowd;
	crowd->removeAgent(id);

	return true;
}

bool UpdateCrowdAgent(NavMeshInstance* inst, float dt)
{
	if (inst->m_tileCache == nullptr || inst->m_navQuery == nullptr || inst->m_crowd == nullptr)
		return false;
	inst->m_crowd->update(dt, nullptr);
	return true;
}

bool GetCrowdAgentPos(NavMeshInstance* inst, int index, float& x, float& y)
{
	if (inst->m_tileCache == nullptr || inst->m_navQuery == nullptr || inst->m_crowd == nullptr)
		return false;

	const dtCrowdAgent* ag = inst->m_crowd->getAgent(index);
	if (ag == nullptr)
		return false;

	auto startPtr = ag->npos;
	x = -startPtr[0];
	y = startPtr[2];

	return true;
}

bool ResetCrowdAgentTarget(NavMeshInstance* inst, int index)
{
	if (inst->m_tileCache == nullptr || inst->m_navQuery == nullptr || inst->m_crowd == nullptr)
		return false;
	inst->m_crowd->resetMoveTarget(index);

	return true;
}
bool SetCrowdAgentTarget(NavMeshInstance* inst, int index, float x, float y)
{
	if (inst->m_tileCache == nullptr || inst->m_navQuery == nullptr || inst->m_crowd == nullptr)
		return false;

	const dtCrowdAgent* ag = inst->m_crowd->getAgent(index);
	if (ag == nullptr)
		return false;

	float ePos[3] = { 0 };
	ePos[0] = -x;
	ePos[1] = 0.f;
	ePos[2] = y;

	dtQueryFilter m_filter;
	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);

	float m_polyPickExt[3] = { 2.0f, 4.0f, 2.0f };

	dtPolyRef m_startRef = 0;
	inst->m_navQuery->findNearestPoly(ePos, m_polyPickExt, &m_filter, &m_startRef, 0);
	inst->m_crowd->requestMoveTarget(index, m_startRef, ePos);
	return true;
}


void ClearNavMesh()
{
	for (auto itr = g_navmesh_insts.begin(); itr != g_navmesh_insts.end(); ++itr)
	{
		NavMeshInstance* inst = *itr;
		if (inst)
		{
			delete inst;
		}
	}
	g_navmesh_insts.clear();
}