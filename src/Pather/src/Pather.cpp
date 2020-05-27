
#include <cstring>
#include <math.h>
#include <Pather.h>
#include "DetourAlloc.h"
#include "DetourNavMesh.h"
#include "DetourStatus.h"
#include "Recast.h"
#include "RecastDump.h"
#include <DetourCommon.h>
#include <InputGeom.h>

#include "DetourNavMeshBuilder.h"


using namespace std;

class NavMesh__pImpl
{
	VM_DECL_API(NavMesh)
public:
	NavMesh__pImpl(NavMesh* api) :q_ptr(api) {}
	std::shared_ptr<InputGeom> m_geom;
	std::shared_ptr<dtNavMesh> m_navMesh;
	std::shared_ptr<dtNavMeshQuery> m_navQuery;
	std::shared_ptr<dtCrowd> m_crowd;
	
	std::shared_ptr<rcContourSet> m_cset;
	std::shared_ptr<rcPolyMesh> m_pmesh;
	std::shared_ptr<rcPolyMeshDetail> m_dmesh;
	std::shared_ptr<rcHeightfield> m_solid;
	std::shared_ptr<rcCompactHeightfield> m_chf;
	std::unique_ptr<unsigned char[]> m_triareas;
	ExternalSettings m_es;
	NavMeshDesc Desc;
};

static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

static float frand()
{
	//	return ((float)(rand() & 0xffff)/(float)0xffff);
	return (float)rand() / (float)RAND_MAX;
}



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


void NavMesh::Simulate(float dt)
{
	VM_IMPL(NavMesh)
	auto nav = _->m_navMesh;
	auto crowd = _->m_crowd;
	if (!nav || !crowd) 
		return;

	crowd->update(dt, nullptr);

	//m_crowdSampleCount.addSample((float)crowd->getVelocitySampleCount());
	//m_crowdTotalTime.addSample(getPerfTimeUsec(endTime - startTime) / 1000.0f);

}

NavMesh::NavMesh(std::shared_ptr<InputGeom> geom, const ExternalSettings& es) :
	d_ptr(new NavMesh__pImpl(this))
{
	d_ptr->m_geom = std::move(geom);
	d_ptr->m_es = es;
}

NavMesh::NavMesh(std::shared_ptr<dtNavMesh> navMesh) :d_ptr(new NavMesh__pImpl(this))
{
	d_ptr->m_navMesh = std::move(navMesh);
	d_ptr->m_navQuery = std::shared_ptr<dtNavMeshQuery>(dtAllocNavMeshQuery(), [](dtNavMeshQuery* p) {dtFreeNavMeshQuery(p); });;
	d_ptr->m_navQuery->init(d_ptr->m_navMesh.get(), 2048);
}


std::shared_ptr<NavMesh> CreateNavMesh(std::shared_ptr<rcContext> ctx,
	std::shared_ptr<InputGeom> geom,
	const ExternalSettings& es,
	const NavMeshDesc& desc)
{
	auto p = std::shared_ptr<NavMesh>(new NavMesh(geom, es));
	p->SetExternalSettings(es);
	p->Build(ctx, desc);
	p->InitQuery();
	p->InitCrowd();
	return p;

}

std::shared_ptr<NavMesh> LoadNavMesh(const std::string& fileName)
{
	FILE* fp = fopen(fileName.c_str(), "rb");
	if (!fp) return 0;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return nullptr;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return nullptr;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return nullptr;
	}

	auto mesh = shared_ptr<dtNavMesh>(dtAllocNavMesh(), [](dtNavMesh* p) {dtFreeNavMesh(p); });
	if (!mesh)
	{
		fclose(fp);
		return nullptr;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return nullptr;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			return nullptr;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			fclose(fp);
			return nullptr;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	auto p = shared_ptr<NavMesh>(new NavMesh(mesh));
	p->InitQuery();
	p->InitCrowd();

	return p;
}

bool NavMesh::SaveAs(const std::string& fileName)
{
	VM_IMPL(NavMesh);

	auto mesh = (const dtNavMesh*)_->m_navMesh.get();
	if (!_->m_navMesh)
		return false;

	FILE* fp = fopen(fileName.c_str(), "wb");
	if (!fp)
		return false;


	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;

	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const auto tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < _->m_navMesh->getMaxTiles(); ++i)
	{
		const auto tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}
	fclose(fp);
}

int NavMesh::AddAgent(const Point3f& pos)
{
	VM_IMPL(NavMesh);
	auto crowd = _->m_crowd.get();
	assert(crowd);

	const auto& desc = _->Desc;


	dtCrowdAgentParams ap;
	memset(&ap, 0, sizeof(ap));
	ap.radius = desc.AgentRadius;
	ap.height = desc.AgentHeight;
	ap.maxAcceleration = 8.0f;
	ap.maxSpeed = 3.5f;
	ap.collisionQueryRange = ap.radius * 12.0f;
	ap.pathOptimizationRange = ap.radius * 30.0f;
	ap.updateFlags = 0;
	//if (m_toolParams.m_anticipateTurns)
	ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	//if (m_toolParams.m_optimizeVis)
	ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	//if (m_toolParams.m_optimizeTopo)
	ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	//if (m_toolParams.m_obstacleAvoidance)
	ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	//if (m_toolParams.m_separation)
	ap.updateFlags |= DT_CROWD_SEPARATION;
	ap.obstacleAvoidanceType = 0; // (unsigned char)m_toolParams.m_obstacleAvoidanceType;
	ap.separationWeight = 2.0f; // m_toolParams.m_separationWeight;

	auto idx = crowd->addAgent(pos.ConstData(), &ap);
	//if (idx != -1)
	//{
	//	if (m_targetRef)
	//		crowd->requestMoveTarget(idx, m_targetRef, m_targetPos);

	//}
	return idx;

}

void NavMesh::RemoveAgent(int idx)
{
	VM_IMPL(NavMesh);
	auto crowd = _->m_crowd.get();
	assert(crowd);
	crowd->removeAgent(idx);
}


void NavMesh::MoveAllAgentsTo(const Point3f& target)
{
	VM_IMPL(NavMesh);
	auto navquery = _->m_navQuery.get();
	auto crowd = _->m_crowd.get();
	assert(navquery && crowd);
	auto filter = crowd->getFilter(0);
	const float* halfExtents = crowd->getQueryExtents();

	Point3f targetPos;
	dtPolyRef targetRef;

	navquery->findNearestPoly(target.ConstData(), halfExtents, filter, &targetRef, targetPos.Data());

	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		auto ag = crowd->getAgent(i);
		if (!ag->active)
			continue;
		crowd->requestMoveTarget(i, targetRef, targetPos.ConstData());
	}


}

void NavMesh::MoveAgentTo(int idx, const Point3f& target)
{
	VM_IMPL(NavMesh);
	auto navquery = _->m_navQuery.get();
	auto crowd = _->m_crowd.get();
	assert(navquery && crowd);
	auto filter = crowd->getFilter(0);
	const float* halfExtents = crowd->getQueryExtents();

	Point3f targetPos;
	dtPolyRef targetRef;

	navquery->findNearestPoly(target.ConstData(),
		halfExtents, filter, &targetRef, targetPos.Data());

	auto ag = crowd->getAgent(idx);

	if (!ag || !ag->active)
		return;
	crowd->requestMoveTarget(idx, targetRef, targetPos.ConstData());

}

Point3f NavMesh::GetAgentCurrentPosition(int idx)
{
	VM_IMPL(NavMesh);
	auto navquery = _->m_navQuery.get();
	auto crowd = _->m_crowd.get();
	assert(navquery && crowd);
	auto filter = crowd->getFilter(0);


	auto ag = crowd->getAgent(idx);

	if (!ag || !ag->active)
		return Point3f();

	return Point3f(ag->npos[0],ag->npos[1],ag->npos[2]);

}


std::vector<Point3f> NavMesh::GetRandomPosition(int count)
{
	VM_IMPL(NavMesh);
	dtQueryFilter filter;
	filter.setIncludeFlags(POLYFLAGS_ALL ^ POLYFLAGS_DISABLED);
	filter.setExcludeFlags(0);

	std::vector<Point3f> res;
	int np = 0;
	for (int i = 0; i < count; i++)
	{
		float pt[3];
		dtPolyRef ref;
		
		dtStatus status = _->m_navQuery->findRandomPoint(&filter, frand, &ref, pt);
		if (dtStatusSucceed(status))
		{
			Point3f p;
			dtVcopy(p.Data(), pt);
			res.push_back(p);
			np++;
		}
	}
	return res;
}


std::vector<Point3f> NavMesh::GetRandomPositionAroundCircle(int count, const Point3f& center, float radius)
{
	VM_IMPL(NavMesh);

	std::vector<Point3f> res;
	int np = 0;

	dtQueryFilter filter;
	filter.setIncludeFlags(POLYFLAGS_ALL ^ POLYFLAGS_DISABLED);
	filter.setExcludeFlags(0);

	dtPolyRef startRef;

	const float pickExtent[3] = { 2,4,2 };

	_->m_navQuery->findNearestPoly(center.ConstData(), 
		pickExtent, 
		&filter, 
		&startRef, 0);

	for (int i = 0; i < count; i++)
	{
		float pt[3];
		dtPolyRef ref;

		dtStatus status = _->m_navQuery->findRandomPointAroundCircle(startRef, center.ConstData(), radius, &filter, frand, &ref, pt);
		if (dtStatusSucceed(status))
		{
			Point3f p;
			dtVcopy(p.Data(), pt);
			res.push_back(p);
			np++;
		}
	}

	return res;
}


void NavMesh::SetExternalSettings(const ExternalSettings& es)
{
	VM_IMPL(NavMesh)
		_->m_es = es;
}


bool NavMesh::Build(std::shared_ptr<rcContext> ctx, const NavMeshDesc& desc)
{
	VM_IMPL(NavMesh);



	if (!_->m_geom || !_->m_geom->getMesh())
	{
		ctx.get()->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}

	_->Desc = desc;

	//cleanup();

	const float* bmin = _->m_geom->getNavMeshBoundsMin();
	const float* bmax = _->m_geom->getNavMeshBoundsMax();
	const float* verts = _->m_geom->getMesh()->getVerts();
	const int nverts = _->m_geom->getMesh()->getVertCount();
	const int* tris = _->m_geom->getMesh()->getTris();
	const int ntris = _->m_geom->getMesh()->getTriCount();

	//
	// Step 1. Initialize build config.
	//
	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));

	cfg.cs = desc.CellSize;
	cfg.ch = desc.CellHeight;
	cfg.walkableSlopeAngle = desc.AgentMaxSlope;
	cfg.walkableHeight = (int)ceilf(desc.AgentHeight / cfg.ch);
	cfg.walkableClimb = (int)floorf(desc.AgentMaxClimb / cfg.ch);
	cfg.walkableRadius = (int)ceilf(desc.AgentRadius / cfg.cs);
	cfg.maxEdgeLen = (int)(desc.EdgeMaxLen / desc.CellSize);
	cfg.maxSimplificationError = desc.EdgeMaxError;
	cfg.minRegionArea = (int)rcSqr(desc.RegionMinSize);		// Note: area = size*size
	cfg.mergeRegionArea = (int)rcSqr(desc.RegionMergeSize);	// Note: area = size*size
	cfg.maxVertsPerPoly = (int)desc.VertsPerPoly;
	cfg.detailSampleDist = desc.DetailSampleDist < 0.9f ? 0 : desc.CellSize * desc.DetailSampleDist;
	cfg.detailSampleMaxError = desc.CellHeight * desc.DetailSampleMaxError;

	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	rcVcopy(cfg.bmin, bmin);
	rcVcopy(cfg.bmax, bmax);
	rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);


	// Reset build times gathering.
	ctx.get()->resetTimers();

	// Start the build process.	
	ctx.get()->startTimer(RC_TIMER_TOTAL);

	ctx.get()->log(RC_LOG_PROGRESS, "Building navigation:");
	ctx.get()->log(RC_LOG_PROGRESS, " - %d x %d cells", cfg.width, cfg.height);
	ctx.get()->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, ntris / 1000.0f);

	//
	// Step 2. Rasterize input polygon soup.
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	_->m_solid = std::shared_ptr<rcHeightfield>(rcAllocHeightfield(), [](rcHeightfield* p) {rcFreeHeightField(p); });
	if (!_->m_solid)
	{
		ctx.get()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return false;
	}
	if (!rcCreateHeightfield(ctx.get(), *_->m_solid, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch))
	{
		ctx.get()->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	_->m_triareas = std::make_unique<unsigned char[]>(ntris);
	if (!_->m_triareas)
	{
		ctx.get()->log(RC_LOG_ERROR, "buildNavigation: Out of memory '_->m_triareas' (%d).", ntris);
		return false;
	}

	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	memset(_->m_triareas.get(), 0, ntris * sizeof(unsigned char));
	rcMarkWalkableTriangles(ctx.get(), cfg.walkableSlopeAngle, verts, nverts, tris, ntris, _->m_triareas.get());
	if (!rcRasterizeTriangles(ctx.get(), verts, nverts, tris, _->m_triareas.get(), ntris, *_->m_solid, cfg.walkableClimb))
	{
		ctx.get()->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return false;
	}

	if (!_->m_es.KeepInterResult)
	{
		_->m_triareas = nullptr;
	}

	//
	// Step 3. Filter walkables surfaces.
	//

	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (_->m_es.FilterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(ctx.get(), cfg.walkableClimb, *_->m_solid);
	if (_->m_es.FilterLedgeSpan)
		rcFilterLedgeSpans(ctx.get(), cfg.walkableHeight, cfg.walkableClimb, *_->m_solid);
	if (_->m_es.FilterWalkableLowHeighSpan)
		rcFilterWalkableLowHeightSpans(ctx.get(), cfg.walkableHeight, *_->m_solid);


	//
	// Step 4. Partition walkable surface to simple regions.
	//

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	_->m_chf = shared_ptr<rcCompactHeightfield>(rcAllocCompactHeightfield(), [](rcCompactHeightfield* p) {rcFreeCompactHeightfield(p); });
	if (!_->m_chf)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(ctx.get(), cfg.walkableHeight, cfg.walkableClimb, *_->m_solid, *_->m_chf))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}

	if (!_->m_es.KeepInterResult)
	{
		_->m_solid = nullptr;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(ctx.get(), cfg.walkableRadius, *_->m_chf))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}

	// (Optional) Mark areas.
	const ConvexVolume* vols = _->m_geom->getConvexVolumes();
	for (int i = 0; i < _->m_geom->getConvexVolumeCount(); ++i)
		rcMarkConvexPolyArea(ctx.get(), vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *_->m_chf);


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

	if (_->m_es.PartitionType == Sample_Partition_Watershed)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(ctx.get(), *_->m_chf))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(ctx.get(), *_->m_chf, 0, cfg.minRegionArea, cfg.mergeRegionArea))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return false;
		}
	}
	else if (_->m_es.PartitionType == Sample_Partition_Monoton)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(ctx.get(), *_->m_chf, 0, cfg.minRegionArea, cfg.mergeRegionArea))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return false;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(ctx.get(), *_->m_chf, 0, cfg.minRegionArea))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return false;
		}
	}

	//
	// Step 5. Trace and simplify region contours.
	//

	// Create contours.
	_->m_cset = std::shared_ptr<rcContourSet>(rcAllocContourSet(), [](rcContourSet* p) {rcFreeContourSet(p); });

	if (!_->m_cset)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(ctx.get(), *_->m_chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *_->m_cset))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	// Build polygon navmesh from the contours.
	_->m_pmesh = std::shared_ptr<rcPolyMesh>(rcAllocPolyMesh(), [](rcPolyMesh* p) {rcFreePolyMesh(p); });
	if (!_->m_pmesh)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(ctx.get(), *_->m_cset, cfg.maxVertsPerPoly, *_->m_pmesh))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//

	_->m_dmesh = std::shared_ptr<rcPolyMeshDetail>(rcAllocPolyMeshDetail(), [](rcPolyMeshDetail* p) {rcFreePolyMeshDetail(p); });
	if (!_->m_dmesh)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	if (!rcBuildPolyMeshDetail(ctx.get(), *_->m_pmesh, *_->m_chf, cfg.detailSampleDist, cfg.detailSampleMaxError, *_->m_dmesh))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}

	if (!_->m_es.KeepInterResult)
	{
		_->m_chf = nullptr;
		_->m_cset = nullptr;
	}

	// At this point the navigation mesh data is ready, you can access it from _->m_pmesh.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//

	// The GUI may allow more max points per polygon than Detour can handle.
	// Only build the detour navmesh if we do not exceed the limit.
	if (cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Update poly flags from areas.

		for (int i = 0; i < _->m_pmesh->npolys; ++i)
		{
			if (_->m_pmesh->areas[i] == RC_WALKABLE_AREA)
				_->m_pmesh->areas[i] = POLYAREA_GROUND;

			if (_->m_pmesh->areas[i] == POLYAREA_GROUND ||
				_->m_pmesh->areas[i] == POLYAREA_GRASS ||
				_->m_pmesh->areas[i] == POLYAREA_ROAD)
			{
				_->m_pmesh->flags[i] = POLYFLAGS_WALK;
			}
			else if (_->m_pmesh->areas[i] == POLYAREA_WATER)
			{
				_->m_pmesh->flags[i] = POLYFLAGS_SWIM;
			}
			else if (_->m_pmesh->areas[i] == POLYAREA_DOOR)
			{
				_->m_pmesh->flags[i] = POLYFLAGS_WALK | POLYFLAGS_DOOR;
			}
		}


		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = _->m_pmesh->verts;
		params.vertCount = _->m_pmesh->nverts;
		params.polys = _->m_pmesh->polys;
		params.polyAreas = _->m_pmesh->areas;
		params.polyFlags = _->m_pmesh->flags;
		params.polyCount = _->m_pmesh->npolys;
		params.nvp = _->m_pmesh->nvp;
		params.detailMeshes = _->m_dmesh->meshes;
		params.detailVerts = _->m_dmesh->verts;
		params.detailVertsCount = _->m_dmesh->nverts;
		params.detailTris = _->m_dmesh->tris;
		params.detailTriCount = _->m_dmesh->ntris;
		params.offMeshConVerts = _->m_geom->getOffMeshConnectionVerts();
		params.offMeshConRad = _->m_geom->getOffMeshConnectionRads();
		params.offMeshConDir = _->m_geom->getOffMeshConnectionDirs();
		params.offMeshConAreas = _->m_geom->getOffMeshConnectionAreas();
		params.offMeshConFlags = _->m_geom->getOffMeshConnectionFlags();
		params.offMeshConUserID = _->m_geom->getOffMeshConnectionId();
		params.offMeshConCount = _->m_geom->getOffMeshConnectionCount();
		params.walkableHeight = desc.AgentHeight;
		params.walkableRadius = desc.AgentRadius;
		params.walkableClimb = desc.AgentMaxClimb;
		rcVcopy(params.bmin, _->m_pmesh->bmin);
		rcVcopy(params.bmax, _->m_pmesh->bmax);
		params.cs = cfg.cs;
		params.ch = cfg.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}

		_->m_navMesh = std::shared_ptr<dtNavMesh>(dtAllocNavMesh(), [](dtNavMesh* p) {dtFreeNavMesh(p); });
		if (!_->m_navMesh)
		{
			dtFree(navData);
			ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return false;
		}

		dtStatus status;

		status = _->m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}
		_->m_navQuery = std::shared_ptr<dtNavMeshQuery>(dtAllocNavMeshQuery(), [](dtNavMeshQuery* p) {dtFreeNavMeshQuery(p); });

		status = _->m_navQuery->init(_->m_navMesh.get(), 2048);
		if (dtStatusFailed(status))
		{
			ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
			return false;
		}
	}

	ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*ctx, ctx.get()->getAccumulatedTime(RC_TIMER_TOTAL));
	ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", _->m_pmesh->nverts, _->m_pmesh->npolys);

	auto m_totalBuildTimeMs = ctx.get()->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;


	//if (m_tool)
	//	m_tool->init(this);
	//initToolStates(this);

	return true;

}

void NavMesh::InitCrowd()
{
	VM_IMPL(NavMesh);

	_->m_crowd = std::shared_ptr<dtCrowd>(dtAllocCrowd(), [](dtCrowd* p) {dtFreeCrowd(p); });

	auto nav = _->m_navMesh.get();
	auto crowd = _->m_crowd.get();

	assert(nav && crowd);

	const auto& desc = _->Desc;

	crowd->init(10, desc.AgentRadius, nav);

	// Make polygons with 'disabled' flag invalid.
	crowd->getEditableFilter(0)->setExcludeFlags(POLYFLAGS_DISABLED);

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

}

void NavMesh::InitQuery()
{
	VM_IMPL(NavMesh);
	_->m_navQuery = std::shared_ptr<dtNavMeshQuery>(dtAllocNavMeshQuery(), [](dtNavMeshQuery* p) {dtFreeNavMeshQuery(p); });
	_->m_navQuery->init(_->m_navMesh.get(), 2048);
}

