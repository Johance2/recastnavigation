#include "NavMeshWrapper.h"
#include <stdio.h>
#include <stdint.h>
#include <memory>
#include <fstream>
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
dtNavMesh *g_navMesh = nullptr;
dtNavMeshQuery *g_navQuery = nullptr;

static const int MAX_POLYS = 256;
float m_straightPath[MAX_POLYS * 3];
int32_t m_nStraightPath = 0;

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

dtTileCache* g_tileCache = nullptr;
LinearAllocator* m_talloc = new LinearAllocator(32000);
FastLZCompressor* m_tcomp = new FastLZCompressor;
MeshProcess* m_tmproc = new MeshProcess;
//------------------------- TempObstacles End-------------------------------



#ifdef ENABLE_LOG
	#define LOG(fmt, ...) fprintf(fp, fmt"\r\n", ##__VA_ARGS__)
#else
	#define LOG
#endif

bool LoadNavMesh(unsigned char* pucValue, unsigned int uiLength)
{
	UnLoadNavMesh();
#ifdef ENABLE_LOG
	fp = fopen("navmesh.log", "w+");
#endif
	LOG("--------------------------------------------LoadNavMesh--------------------------------------------");
	g_navMesh = dtAllocNavMesh();
	if (!g_navMesh)
	{
		LOG("Could not create Detour navmesh");
		return false;
	}

	dtStatus status;

	unsigned char* data = (unsigned char*)dtAlloc(uiLength, DT_ALLOC_PERM);
	memcpy(data, pucValue, uiLength);
	status = g_navMesh->init(data, uiLength, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		LOG("Could not init Detour navmesh");
		return false;
	}

	g_navQuery = dtAllocNavMeshQuery();
	status = g_navQuery->init(g_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		LOG("Could not init Detour navmesh query");
		return false;
	}
	LOG("LoadNavMesh:%d", uiLength);
	return true;
}

bool LoadObstaclesMesh(unsigned char* pucValue, unsigned int uiLength)
{
	UnLoadNavMesh();

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
		return false;
	}
	if (header.version != TILECACHESET_VERSION)
	{
		return false;
	}

	g_navMesh = dtAllocNavMesh();
	if (!g_navMesh)
	{
		return false;
	}
	dtStatus status = g_navMesh->init(&header.meshParams);
	if (dtStatusFailed(status))
	{
		return false;
	}

	m_talloc = new LinearAllocator(32000);
	m_tcomp = new FastLZCompressor;
	m_tmproc = new MeshProcess;
	g_tileCache = dtAllocTileCache();
	if (!g_tileCache)
	{
		fclose(fp);
		return false;
	}
	status = g_tileCache->init(&header.cacheParams, m_talloc, m_tcomp, m_tmproc);
	if (dtStatusFailed(status))
	{
		return false; 
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

	g_navQuery = dtAllocNavMeshQuery();
	status = g_navQuery->init(g_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		LOG("Could not init Detour navmesh query");
		return false;
	}
	LOG("LoadObstaclesMesh:%d", uiLength);

	return true;
}

int FindStraightPath(float startX, float startY, float endX, float endY)
{
	LOG("FindStraightPath Enter:start(%f, %f) end(%f, %f)", startX, startY, endX, endY);
	if (!g_navQuery)
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

	float m_polyPickExt[3] = { 2.0f, 4.0f, 2.0f };

	float m_fixedEPos[3];
	dtPolyRef m_polys[MAX_POLYS];
	unsigned char m_straightPathFlags[MAX_POLYS];
	dtPolyRef m_straightPathPolys[MAX_POLYS];

	memset(m_fixedEPos, 0, sizeof(m_fixedEPos));
	memset(m_polys, 0, sizeof(m_polys));
	memset(m_straightPath, 0, sizeof(m_straightPath));
	memset(m_straightPathFlags, 0, sizeof(m_straightPathFlags));
	memset(m_straightPathPolys, 0, sizeof(m_straightPathPolys));

	int32_t m_nPolys = 0;
	int32_t m_nStraightPathOptions = 0;
	dtPolyRef m_startRef = 0;
	dtPolyRef m_endRef = 0;

	g_navQuery->findNearestPoly(sPos, m_polyPickExt, &m_filter, &m_startRef, 0);
	g_navQuery->findNearestPoly(ePos, m_polyPickExt, &m_filter, &m_endRef, 0);
	g_navQuery->findPath(m_startRef, m_endRef, sPos, ePos, &m_filter, m_polys, &m_nPolys, MAX_POLYS);

	m_nStraightPath = 0;

	if (m_nPolys > 0)
	{
		// In case of partial path, make sure the end point is clamped to the last polygon.
		m_fixedEPos[0] = ePos[0];
		m_fixedEPos[1] = ePos[1];
		m_fixedEPos[2] = ePos[2];

		if (m_polys[m_nPolys - 1] != m_endRef)
			g_navQuery->closestPointOnPoly(m_polys[m_nPolys - 1], ePos, m_fixedEPos, 0);

		g_navQuery->findStraightPath(sPos, m_fixedEPos, m_polys, m_nPolys, m_straightPath, m_straightPathFlags,
			m_straightPathPolys, &m_nStraightPath, MAX_POLYS, m_nStraightPathOptions);

		if (m_nStraightPath >= MAX_POLYS)
		{
			m_nStraightPath = MAX_POLYS;
			LOG("straightPath out of bound of max polys.");
		}
	}
	LOG("FindStraightPath End:%d", m_nStraightPath);
	return m_nStraightPath;
}

bool GetPathPoint(int index, float& x, float& y)
{
	LOG("GetPathPoint:%d", index);

	if (index >= m_nStraightPath)
		return false;

	auto startPtr = &m_straightPath[index * 3];
	x = -startPtr[0];
	y = startPtr[2];

	LOG("GetPathPoint:%d %f %f", index, x, y);
	return true;
}

bool PathRaycast(float startX, float startY, float endX, float endY, float& hitX, float& hitY)
{
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
	g_navQuery->findNearestPoly(sPos, m_polyPickExt, &m_filter, &m_startRef, 0);

	float t = 0;
	int m_npolys = 0;
	g_navQuery->raycast(m_startRef, sPos, ePos, &m_filter, &t, m_hitNormal, m_polys, &m_npolys, MAX_POLYS);
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

void UnLoadNavMesh()
{
	if (g_navMesh)
	{
		dtFreeNavMesh(g_navMesh);
		g_navMesh = nullptr;
	}
	if (g_navQuery)
	{
		dtFreeNavMeshQuery(g_navQuery);
		g_navQuery = nullptr;
	}
	if (g_tileCache)
	{
		dtFreeTileCache(g_tileCache);
		g_tileCache = nullptr;
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


bool AddObstacles(float x, float y, float z, float radius, float height, unsigned int& id, bool update)
{
	if (g_tileCache == nullptr)
		return false;
	float pos[3] = {-x,y,z};
	dtStatus status = ((dtTileCache*)g_tileCache)->addObstacle(pos, radius, height, (dtObstacleRef*)&id);
	if ((status & DT_BUFFER_TOO_SMALL))
	{
		if (UpdateObstaclesMesh())
		{
			status = ((dtTileCache*)g_tileCache)->addObstacle(pos, radius, height, (dtObstacleRef*)&id);
			update = false;
		}
	}
	bool success = dtStatusSucceed(status);
	if (success)
	{
		if (update)
		{
			return UpdateObstaclesMesh();
		}
	}
	else
	{
		printf("addObstacle fail:%d\n", status);
	}
	return success;
}
bool AddBoxObstacles(float minx, float miny, float minz, float maxx, float maxy, float maxz, unsigned int& id, bool update)
{
	if (g_tileCache == nullptr)
		return false;
	float bmin[3] = { -maxx,miny,minz };
	float bmax[3] = { -minx,maxy,maxz };
	dtStatus status = ((dtTileCache*)g_tileCache)->addBoxObstacle(bmin, bmax, (dtObstacleRef*)&id);
	if ((status & DT_BUFFER_TOO_SMALL))
	{
		if (UpdateObstaclesMesh())
		{
			status = ((dtTileCache*)g_tileCache)->addBoxObstacle(bmin, bmax, (dtObstacleRef*)&id);
			update = false;
		}
	}
	bool success = dtStatusSucceed(status);
	if (success)
	{
		if (update)
		{
			return UpdateObstaclesMesh();
		}
	}
	else
	{
		printf("addObstacle fail:%d\n", status);
	}
	return success;
}
bool RemoveObstacles(unsigned int id, bool update)
{
	if (g_tileCache == nullptr)
		return false;
	dtStatus status = ((dtTileCache*)g_tileCache)->removeObstacle((dtObstacleRef)id);
	bool success = dtStatusSucceed(status);
	if (success)
	{
		if (update)
		{
			return UpdateObstaclesMesh();
		}
	}
	else
	{
		printf("addObstacle fail:%d\n", status);
	}

	return success;
}

bool UpdateObstaclesMesh()
{
	if (g_tileCache == nullptr || g_navQuery == nullptr)
		return false;

	bool upToDate = false;
	dtNavMesh* navMeshQuery = (dtNavMesh*)(((dtNavMeshQuery*)g_navQuery)->getAttachedNavMesh());
	while (!upToDate)
	{
		dtStatus status = ((dtTileCache*)g_tileCache)->update(0, navMeshQuery, &upToDate);
		if (!dtStatusSucceed(status))
		{
			printf("tickUpdate fail:%d\n", status);
			return false;
		}
	}
	return true;
}