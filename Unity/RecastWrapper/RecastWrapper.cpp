#include "RecastWrapper.h"
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "MeshLoaderObj.h"
#include "ChunkyTriMesh.h"
#include "InputGeom.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include <math.h>
#include "fastlz.h"

#define ERROR_OUT_OF_MEMORY		1
#define ERROR_OBJ_FILE_ERROR	2
#define ERROR_BUILD_ERROR	-1

enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED,
	SAMPLE_PARTITION_MONOTONE,
	SAMPLE_PARTITION_LAYERS,
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


rcChunkyTriMesh* m_chunkyMesh = nullptr;
rcMeshLoaderObj* m_mesh = nullptr;
rcContext* m_ctx = nullptr;

bool m_keepInterResults = false;
bool m_filterLowHangingObstacles = true;
bool m_filterLedgeSpans = true;
bool m_filterWalkableLowHeightSpans = true;

//------------------------- TempObstacles Begin-----------------------------
static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

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


static const int MAX_LAYERS = 32;

struct TileCacheData
{
	unsigned char* data;
	int dataSize;
};

struct RasterizationContext
{
	RasterizationContext() :
		solid(0),
		triareas(0),
		lset(0),
		chf(0),
		ntiles(0)
	{
		memset(tiles, 0, sizeof(TileCacheData) * MAX_LAYERS);
	}

	~RasterizationContext()
	{
		rcFreeHeightField(solid);
		delete[] triareas;
		rcFreeHeightfieldLayerSet(lset);
		rcFreeCompactHeightfield(chf);
		for (int i = 0; i < MAX_LAYERS; ++i)
		{
			dtFree(tiles[i].data);
			tiles[i].data = 0;
		}
	}

	rcHeightfield* solid;
	unsigned char* triareas;
	rcHeightfieldLayerSet* lset;
	rcCompactHeightfield* chf;
	TileCacheData tiles[MAX_LAYERS];
	int ntiles;
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

dtTileCache* m_tileCache = nullptr;
dtNavMesh* m_navMesh = nullptr;
LinearAllocator *m_talloc = new LinearAllocator(32000);
FastLZCompressor *m_tcomp = new FastLZCompressor;
MeshProcess *m_tmproc = new MeshProcess;
dtNavMeshQuery *m_navQuery = nullptr;

int rasterizeTileLayers(
	const int tx, const int ty,
	const rcConfig& cfg,
	TileCacheData* tiles,
	const int maxTiles)
{
	FastLZCompressor comp;
	RasterizationContext rc;

	const float* verts = m_mesh->getVerts();
	const int nverts = m_mesh->getVertCount();

	auto m_chunkyMesh = new rcChunkyTriMesh;
	if (!m_chunkyMesh)
	{
		printf("buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
		return false;
	}
	if (!rcCreateChunkyTriMesh(m_mesh->getVerts(), m_mesh->getTris(), m_mesh->getTriCount(), 256, m_chunkyMesh))
	{
		printf("buildTiledNavigation: Failed to build chunky mesh.");
		return false;
	}
	const rcChunkyTriMesh* chunkyMesh = m_chunkyMesh;

	// Tile bounds.
	const float tcs = cfg.tileSize * cfg.cs;

	rcConfig tcfg;
	memcpy(&tcfg, &cfg, sizeof(tcfg));

	tcfg.bmin[0] = cfg.bmin[0] + tx * tcs;
	tcfg.bmin[1] = cfg.bmin[1];
	tcfg.bmin[2] = cfg.bmin[2] + ty * tcs;
	tcfg.bmax[0] = cfg.bmin[0] + (tx + 1) * tcs;
	tcfg.bmax[1] = cfg.bmax[1];
	tcfg.bmax[2] = cfg.bmin[2] + (ty + 1) * tcs;
	tcfg.bmin[0] -= tcfg.borderSize * tcfg.cs;
	tcfg.bmin[2] -= tcfg.borderSize * tcfg.cs;
	tcfg.bmax[0] += tcfg.borderSize * tcfg.cs;
	tcfg.bmax[2] += tcfg.borderSize * tcfg.cs;

	// Allocate voxel heightfield where we rasterize our input data to.
	rc.solid = rcAllocHeightfield();
	if (!rc.solid)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(m_ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}

	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	rc.triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!rc.triareas)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
		return 0;
	}

	float tbmin[2], tbmax[2];
	tbmin[0] = tcfg.bmin[0];
	tbmin[1] = tcfg.bmin[2];
	tbmax[0] = tcfg.bmax[0];
	tbmax[1] = tcfg.bmax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		return 0; // empty
	}

	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* tris = &chunkyMesh->tris[node.i * 3];
		const int ntris = node.n;

		memset(rc.triareas, 0, ntris * sizeof(unsigned char));
		rcMarkWalkableTriangles(m_ctx, tcfg.walkableSlopeAngle,
			verts, nverts, tris, ntris, rc.triareas);

		if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb))
			return 0;
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, tcfg.walkableClimb, *rc.solid);
	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, tcfg.walkableHeight, *rc.solid);


	rc.chf = rcAllocCompactHeightfield();
	if (!rc.chf)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, tcfg.walkableRadius, *rc.chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}

	// (Optional) Mark areas.
	//const ConvexVolume* vols = m_geom->getConvexVolumes();
	//for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
	//{
	//	rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts,
	//		vols[i].hmin, vols[i].hmax,
	//		(unsigned char)vols[i].area, *rc.chf);
	//}

	rc.lset = rcAllocHeightfieldLayerSet();
	if (!rc.lset)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'.");
		return 0;
	}
	if (!rcBuildHeightfieldLayers(m_ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
		return 0;
	}

	rc.ntiles = 0;
	for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
	{
		TileCacheData* tile = &rc.tiles[rc.ntiles++];
		const rcHeightfieldLayer* layer = &rc.lset->layers[i];

		// Store header
		dtTileCacheLayerHeader header;
		header.magic = DT_TILECACHE_MAGIC;
		header.version = DT_TILECACHE_VERSION;

		// Tile layer location in the navmesh.
		header.tx = tx;
		header.ty = ty;
		header.tlayer = i;
		dtVcopy(header.bmin, layer->bmin);
		dtVcopy(header.bmax, layer->bmax);

		// Tile info.
		header.width = (unsigned char)layer->width;
		header.height = (unsigned char)layer->height;
		header.minx = (unsigned char)layer->minx;
		header.maxx = (unsigned char)layer->maxx;
		header.miny = (unsigned char)layer->miny;
		header.maxy = (unsigned char)layer->maxy;
		header.hmin = (unsigned short)layer->hmin;
		header.hmax = (unsigned short)layer->hmax;

		dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
			&tile->data, &tile->dataSize);
		if (dtStatusFailed(status))
		{
			return 0;
		}
	}

	// Transfer ownsership of tile data from build context to the caller.
	int n = 0;
	for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
	{
		tiles[n++] = rc.tiles[i];
		rc.tiles[i].data = 0;
		rc.tiles[i].dataSize = 0;
	}

	return n;
}

//------------------------- TempObstacles End-------------------------------

float m_cellSize = 0.3f;
float m_cellHeight = 0.2f;
float m_agentMaxSlope = 45.0f;
float m_agentHeight = 2.0f;
float m_agentMaxClimb = 0.9f;
float m_agentRadius = 0.7f;
float m_edgeMaxLen = 12.0f;
float m_edgeMaxError = 1.3f;
float m_regionMinSize = 8.0f;
float m_regionMergeSize = 20.0f;
float m_vertsPerPoly = 6.0f;
float m_detailSampleDist = 6.0f;
float m_detailSampleMaxError = 1.0f;
int m_tileSize = 48;

int BuildSoloMesh(const char* objPath, const char* binPath, const char* param)
{
	int m_partitionType = SAMPLE_PARTITION_WATERSHED;
	m_ctx = new rcContext();
	m_mesh = new rcMeshLoaderObj;
	if (!m_mesh)
	{
		printf("loadMesh: Out of memory 'mesh'.");
		return ERROR_OUT_OF_MEMORY;
	}
	if (!m_mesh->load(objPath))
	{
		printf("buildTiledNavigation: Could not load '%s'", objPath);
		return 2;
	}

	float meshBMin[3];
	float meshBMax[3];

	rcCalcBounds(m_mesh->getVerts(), m_mesh->getVertCount(), meshBMin, meshBMax);

	const float* bmin = meshBMin;
	const float* bmax = meshBMax;
	const float* verts = m_mesh->getVerts();
	const int nverts = m_mesh->getVertCount();
	const int* tris = m_mesh->getTris();
	const int ntris = m_mesh->getTriCount();

	//
	// Step 1. Initialize build config.
	//
	rcConfig m_cfg;
	memset(&m_cfg, 0, sizeof(m_cfg));
	if (param != nullptr && strlen(param) > 0)
	{
		int err = sscanf(param, "%f %f %f %f %f %f %f %f %f %f %f %f %f",
			&m_cellSize,
			&m_cellHeight,
			&m_agentHeight,
			&m_agentRadius,
			&m_agentMaxClimb,
			&m_agentMaxSlope,
			&m_regionMinSize,
			&m_regionMergeSize,
			&m_edgeMaxLen,
			&m_edgeMaxError,
			&m_vertsPerPoly,
			&m_detailSampleDist,
			&m_detailSampleMaxError);
	}

	m_cfg.cs = m_cellSize;
	m_cfg.ch = m_cellHeight;
	m_cfg.walkableSlopeAngle = m_agentMaxSlope;
	m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);
	m_cfg.walkableClimb = (int)floorf(m_agentMaxClimb / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);
	m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	m_cfg.maxSimplificationError = m_edgeMaxError;
	m_cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	m_cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	rcVcopy(m_cfg.bmin, bmin);
	rcVcopy(m_cfg.bmax, bmax);
	rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);

	//
	// Step 2. Rasterize input polygon soup.
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	auto m_solid = rcAllocHeightfield();
	if (!m_solid)
	{
		printf("buildNavigation: Out of memory 'solid'.");
		return ERROR_OUT_OF_MEMORY;
	}
	if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
	{
		printf("buildNavigation: Could not create solid heightfield.");
		return ERROR_BUILD_ERROR;
	}

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	auto m_triareas = new unsigned char[ntris];
	if (!m_triareas)
	{
		printf("buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return ERROR_OUT_OF_MEMORY;
	}

	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	memset(m_triareas, 0, ntris * sizeof(unsigned char));
	rcMarkWalkableTriangles(nullptr, m_cfg.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
	if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, *m_solid, m_cfg.walkableClimb))
	{
		printf("buildNavigation: Could not rasterize triangles.");
		return ERROR_BUILD_ERROR;
	}

	if (!m_keepInterResults)
	{
		delete[] m_triareas;
		m_triareas = 0;
	}

	//
	// Step 3. Filter walkables surfaces.
	//
	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);


	//
	// Step 4. Partition walkable surface to simple regions.
	//

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	auto m_chf = rcAllocCompactHeightfield();
	if (!m_chf)
	{
		printf("buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
	{
		printf("buildNavigation: Could not build compact data.");
		return false;
	}

	if (!m_keepInterResults)
	{
		rcFreeHeightField(m_solid);
		m_solid = 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
	{
		printf("buildNavigation: Could not erode.");
		return false;
	}

	// (Optional) Mark areas.
	//const ConvexVolume* vols = m_geom->getConvexVolumes();
	//for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
	//	rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);


	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 martitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
	// 2) Monotone partioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 3) Layer partitoining
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than monotone)
	//     if you have large open areas with small obstacles (not a problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized tiles

	if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(m_ctx, *m_chf))
		{
			printf("buildNavigation: Could not build distance field.");
			return false;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			printf("buildNavigation: Could not build watershed regions.");
			return false;
		}
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			printf("buildNavigation: Could not build monotone regions.");
			return false;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea))
		{
			printf("buildNavigation: Could not build layer regions.");
			return false;
		}
	}


	//
	// Step 5. Trace and simplify region contours.
	//

	// Create contours.
	auto m_cset = rcAllocContourSet();
	if (!m_cset)
	{
		printf("buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
	{
		printf("buildNavigation: Could not create contours.");
		return false;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	// Build polygon navmesh from the contours.
	auto m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
	{
		printf("buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
	{
		printf("buildNavigation: Could not triangulate contours.");
		return false;
	}


	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//

	auto m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
	{
		printf("buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
	{
		printf("buildNavigation: Could not build detail mesh.");
		return false;
	}

	if (!m_keepInterResults)
	{
		rcFreeCompactHeightfield(m_chf);
		m_chf = 0;
		rcFreeContourSet(m_cset);
		m_cset = 0;
	}

	// At this point the navigation mesh data is ready, you can access it from m_pmesh.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//

	// The GUI may allow more max points per polygon than Detour can handle.
	// Only build the detour navmesh if we do not exceed the limit.
	if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Update poly flags from areas.
		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
				m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;

			if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}


		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_pmesh->verts;
		params.vertCount = m_pmesh->nverts;
		params.polys = m_pmesh->polys;
		params.polyAreas = m_pmesh->areas;
		params.polyFlags = m_pmesh->flags;
		params.polyCount = m_pmesh->npolys;
		params.nvp = m_pmesh->nvp;
		params.detailMeshes = m_dmesh->meshes;
		params.detailVerts = m_dmesh->verts;
		params.detailVertsCount = m_dmesh->nverts;
		params.detailTris = m_dmesh->tris;
		params.detailTriCount = m_dmesh->ntris;
		//params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
		//params.offMeshConRad = m_geom->getOffMeshConnectionRads();
		//params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
		//params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
		//params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
		//params.offMeshConUserID = m_geom->getOffMeshConnectionId();
		//params.offMeshConCount = m_geom->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_cfg.cs;
		params.ch = m_cfg.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			printf("Could not build Detour navmesh.");
			return false;
		}

		FILE *fp = fopen(binPath, "wb");
		if (!fp)
		{
			printf("Could create file:%s to write bin.", binPath);
			return false;
		}

		fwrite(navData, navDataSize, 1, fp);

		fclose(fp);

		//auto m_navMesh = dtAllocNavMesh();
		//if (!m_navMesh)
		//{
		//	dtFree(navData);
		//	printf("Could not create Detour navmesh");
		//	return false;
		//}

		//dtStatus status;

		//status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		//if (dtStatusFailed(status))
		//{
		//	dtFree(navData);
		//	printf("Could not init Detour navmesh");
		//	return false;
		//}

		//auto m_navQuery = dtAllocNavMeshQuery();
		//status = m_navQuery->init(m_navMesh, 2048);
		//if (dtStatusFailed(status))
		//{
		//	printf("Could not init Detour navmesh query");
		//	return false;
		//}




	}

	printf(">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);

	return 0;
}

int BuildTempObstacles(const char* objPath, const char* binPath, const char* param)
{
	dtStatus status;

	int m_partitionType = SAMPLE_PARTITION_WATERSHED;
	m_ctx = new rcContext();
	m_mesh = new rcMeshLoaderObj;
	if (!m_mesh)
	{
		printf("loadMesh: Out of memory 'mesh'.");
		return ERROR_OUT_OF_MEMORY;
	}
	if (!m_mesh->load(objPath))
	{
		printf("buildTiledNavigation: Could not load '%s'", objPath);
		return 2;
	}

	
	if (param != nullptr && strlen(param) > 0)
	{
		int err = sscanf(param, "%f %f %f %f %f %f %f %f %f %f %f %f %f %d",
			&m_cellSize,
			&m_cellHeight,
			&m_agentHeight,
			&m_agentRadius,
			&m_agentMaxClimb,
			&m_agentMaxSlope,
			&m_regionMinSize,
			&m_regionMergeSize,
			&m_edgeMaxLen,
			&m_edgeMaxError,
			&m_vertsPerPoly,
			&m_detailSampleDist,
			&m_detailSampleMaxError,
			&m_tileSize);
	}

	// Init cache
	float meshBMin[3];
	float meshBMax[3];
	rcCalcBounds(m_mesh->getVerts(), m_mesh->getVertCount(), meshBMin, meshBMax);
	const float* bmin = meshBMin;
	const float* bmax = meshBMax;
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)48;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;

	// Max tiles and max polys affect how the tile IDs are caculated.
	// There are 22 bits available for identifying a tile and a polygon.
	int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw * th * EXPECTED_LAYERS_PER_TILE)), 14);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;
	int m_maxTiles = 1 << tileBits;
	int m_maxPolysPerTile = 1 << polyBits;
	//gridSize = tw * th;

	// Generation params.
	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.cs = m_cellSize;
	cfg.ch = m_cellHeight;
	cfg.walkableSlopeAngle = m_agentMaxSlope;
	cfg.walkableHeight = (int)ceilf(m_agentHeight / cfg.ch);
	cfg.walkableClimb = (int)floorf(m_agentMaxClimb / cfg.ch);
	cfg.walkableRadius = (int)ceilf(m_agentRadius / cfg.cs);
	cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	cfg.maxSimplificationError = m_edgeMaxError;
	cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	cfg.tileSize = (int)m_tileSize;
	cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
	cfg.width = cfg.tileSize + cfg.borderSize * 2;
	cfg.height = cfg.tileSize + cfg.borderSize * 2;
	cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
	rcVcopy(cfg.bmin, bmin);
	rcVcopy(cfg.bmax, bmax);

	// Tile cache params.
	dtTileCacheParams tcparams;
	memset(&tcparams, 0, sizeof(tcparams));
	rcVcopy(tcparams.orig, bmin);
	tcparams.cs = m_cellSize;
	tcparams.ch = m_cellHeight;
	tcparams.width = (int)m_tileSize;
	tcparams.height = (int)m_tileSize;
	tcparams.walkableHeight = m_agentHeight;
	tcparams.walkableRadius = m_agentRadius;
	tcparams.walkableClimb = m_agentMaxClimb;
	tcparams.maxSimplificationError = m_edgeMaxError;
	tcparams.maxTiles = tw * th * EXPECTED_LAYERS_PER_TILE;
	tcparams.maxObstacles = 32*32;

	dtFreeTileCache(m_tileCache);

	m_tileCache = dtAllocTileCache();
	if (!m_tileCache)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate tile cache.");
		return false;
	}
	status = m_tileCache->init(&tcparams, m_talloc, m_tcomp, m_tmproc);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init tile cache.");
		return false;
	}

	dtFreeNavMesh(m_navMesh);

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams params;
	memset(&params, 0, sizeof(params));
	rcVcopy(params.orig, bmin);
	params.tileWidth = m_tileSize * m_cellSize;
	params.tileHeight = m_tileSize * m_cellSize;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;

	status = m_navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return false;
	}

	m_navQuery = dtAllocNavMeshQuery();
	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return false;
	}
	// Preprocess tiles.

	m_ctx->resetTimers();

	int m_cacheLayerCount = 0;
	int m_cacheCompressedSize = 0;
	//int m_cacheRawSize = 0;

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			TileCacheData tiles[MAX_LAYERS];
			memset(tiles, 0, sizeof(tiles));
			int ntiles = rasterizeTileLayers(x, y, cfg, tiles, MAX_LAYERS);

			for (int i = 0; i < ntiles; ++i)
			{
				TileCacheData* tile = &tiles[i];
				status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
				if (dtStatusFailed(status))
				{
					dtFree(tile->data);
					tile->data = 0;
					continue;
				}

				m_cacheLayerCount++;
				m_cacheCompressedSize += tile->dataSize;
				//m_cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
			}
		}
	}
	// Build initial meshes
	m_ctx->startTimer(RC_TIMER_TOTAL);
	for (int y = 0; y < th; ++y)
		for (int x = 0; x < tw; ++x)
			m_tileCache->buildNavMeshTilesAt(x, y, m_navMesh);
	m_ctx->stopTimer(RC_TIMER_TOTAL);


	if (!m_tileCache) return false;

	FILE* fp = fopen(binPath, "wb");
	if (!fp)
	{
		dtFreeTileCache(m_tileCache);
		m_tileCache = nullptr;
		return false;
	}

	// Store header.
	TileCacheSetHeader header;
	header.magic = TILECACHESET_MAGIC;
	header.version = TILECACHESET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < m_tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = m_tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.cacheParams, m_tileCache->getParams(), sizeof(dtTileCacheParams));
	memcpy(&header.meshParams, m_navMesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(TileCacheSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < m_tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = m_tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		TileCacheTileHeader tileHeader;
		tileHeader.tileRef = m_tileCache->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);

	dtFreeTileCache(m_tileCache);
	m_tileCache = nullptr;


	return true;
}


int BuildBlockData(const char* binPath, const char* blockPath, int width, int height, bool bDebugFile /*= false*/)
{
	unsigned char* navData = 0;
	FILE* fp = fopen(binPath, "rb");
	if (!fp)
		return false;
	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return false;
	}
	long navDataSize = ftell(fp);
	if (navDataSize < 0)
	{
		fclose(fp);
		return false;
	}
	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return false;
	}
	navData = (unsigned char*)dtAlloc(navDataSize, dtAllocHint::DT_ALLOC_PERM);
	if (!navData)
	{
		fclose(fp);
		return false;
	}
	size_t readLen = fread(navData, navDataSize, 1, fp);
	fclose(fp);

	auto m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		dtFree(navData);
		printf("Could not create Detour navmesh");
		return false;
	}

	dtStatus status;

	status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		dtFree(navData);
		printf("Could not init Detour navmesh");
		return false;
	}

	auto m_navQuery = dtAllocNavMeshQuery();
	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		printf("Could not init Detour navmesh query");
		return false;
	}

	float pos[3];
	float m_polyPickExt[3] = { 50.6f, 50.0f, 50.6f };
	dtQueryFilter m_filter;
	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);
	dtPolyRef dtPy = 0;
	float nearPos[3];
	char *block = new char[width * height];
	FILE* fp2 = 0;
	if (bDebugFile)
	{
		fp2 = fopen((std::string(blockPath)+".txt").c_str(), "wt");
	}
	for (int j = height-1; j >= 0; j--)
	{
		for (int i = 0; i < width; i++)
		{
			pos[0] = -i;
			pos[1] = 0;
			pos[2] = j;
			bool bIsOverPoy = false;
			dtStatus status = m_navQuery->findNearestPoly(pos, m_polyPickExt, &m_filter, &dtPy, nearPos, &bIsOverPoy);
			if (bIsOverPoy)
			{
				if (status == DT_SUCCESS)
				{
					float hitDist, hitNormal[3], hitPos[3];
					status = m_navQuery->findDistanceToWall(dtPy, pos, 20, &m_filter, &hitDist, hitPos, hitNormal);
					if (status == DT_SUCCESS)
					{
						bIsOverPoy = hitDist >= 0.6;
					}
					else
					{
						bIsOverPoy = false;
					}
				}
			}
			int nIndex = j * width + i;
			block[nIndex] = bIsOverPoy;
			if (fp2)
			{
				fprintf(fp2, "%d", bIsOverPoy ? 1 : 0);
			}
		}
		if (fp2)
		{
			fprintf(fp2, "\n");
		}
	}
	if (fp2)
	{
		fclose(fp2);
	}

	fp = fopen(blockPath, "wb");
	fwrite(&width, sizeof(width), 1, fp);
	fwrite(&height, sizeof(height), 1, fp);
	fwrite(block, width*height, 1, fp);
	fclose(fp);
	delete[] block;

	return 0;
}

int BuildBlockDataObstacles(const char* binPath, const char* blockPath, int width, int height, bool bDebugFile /*= false*/)
{
	unsigned char* pucValue = 0;
	FILE* fp = fopen(binPath, "rb");
	if (!fp)
		return false;
	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return false;
	}
	long uiLength = ftell(fp);
	if (uiLength < 0)
	{
		fclose(fp);
		return false;
	}
	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return false;
	}
	pucValue = (unsigned char*)dtAlloc(uiLength, dtAllocHint::DT_ALLOC_PERM);
	if (!pucValue)
	{
		fclose(fp);
		return false;
	}
	size_t readLen = fread(pucValue, uiLength, 1, fp);
	fclose(fp);
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

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		return false;
	}
	dtStatus status = m_navMesh->init(&header.meshParams);
	if (dtStatusFailed(status))
	{
		return false;
	}

	m_talloc = new LinearAllocator(32000);
	m_tcomp = new FastLZCompressor;
	m_tmproc = new MeshProcess;
	m_tileCache = dtAllocTileCache();
	if (!m_tileCache)
	{
		fclose(fp);
		return false;
	}
	status = m_tileCache->init(&header.cacheParams, m_talloc, m_tcomp, m_tmproc);
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
		dtStatus addTileStatus = m_tileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);
		if (dtStatusFailed(addTileStatus))
		{
			dtFree(data);
		}

		if (tile)
			m_tileCache->buildNavMeshTile(tile, m_navMesh);
	}

	float test_pos[3] = { -32, 0, 32 };
	unsigned int id;
	m_tileCache->addObstacle(test_pos, 10, 10, &id);
	bool upToDate = false;
	dtNavMesh* navMeshQuery = (dtNavMesh*)(((dtNavMeshQuery*)m_navQuery)->getAttachedNavMesh());
	m_tileCache->update(0, navMeshQuery, &upToDate);
	//float pos[3];
	float m_polyPickExt[3] = { 50.6f, 50.0f, 50.6f };
	dtQueryFilter m_filter;
	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);
	dtPolyRef dtPy = 0;
	float nearPos[3];
	char* block = new char[width * height];
	FILE* fp2 = 0;
	if (bDebugFile)
	{
		fp2 = fopen((std::string(blockPath) + ".txt").c_str(), "wt");
	}
	for (int j = height - 1; j >= 0; j--)
	{
		for (int i = 0; i < width; i++)
		{
			test_pos[0] = -i;
			test_pos[1] = 0;
			test_pos[2] = j;
			bool bIsOverPoy = false;
			dtStatus status = m_navQuery->findNearestPoly(test_pos, m_polyPickExt, &m_filter, &dtPy, nearPos, &bIsOverPoy);
			if (bIsOverPoy)
			{
				if (status == DT_SUCCESS)
				{
					float hitDist, hitNormal[3], hitPos[3];
					status = m_navQuery->findDistanceToWall(dtPy, test_pos, 20, &m_filter, &hitDist, hitPos, hitNormal);
					if (status == DT_SUCCESS)
					{
						bIsOverPoy = hitDist >= 0.6;
					}
					else
					{
						bIsOverPoy = false;
					}
				}
			}
			int nIndex = j * width + i;
			block[nIndex] = bIsOverPoy;
			if (fp2)
			{
				fprintf(fp2, "%d", bIsOverPoy ? 1 : 0);
			}
		}
		if (fp2)
		{
			fprintf(fp2, "\n");
		}
	}
	if (fp2)
	{
		fclose(fp2);
	}

	fp = fopen(blockPath, "wb");
	fwrite(&width, sizeof(width), 1, fp);
	fwrite(&height, sizeof(height), 1, fp);
	fwrite(block, width * height, 1, fp);
	fclose(fp);
	delete[] block;

	return 0;
}