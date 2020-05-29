
#include <Query.h>

#include <DetourCommon.h>

#include <Pather.h>


class NavQuery__pImpl
{
	VM_DECL_API(NavQuery)
public:
	NavQuery__pImpl(NavQuery* api) :q_ptr(api) {}
	std::shared_ptr<dtNavMesh> m_navMesh;
	std::shared_ptr<dtNavMeshQuery> m_navQuery;
};

static float frand()
{
	return (float)rand() / (float)RAND_MAX;
}


std::vector<Point3f> NavQuery::GetRandomPosition(int count)
{
	VM_IMPL(NavQuery);
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

std::vector<Point3f> NavQuery::GetRandomPositionAroundCircle(int count, const Point3f& center, float radius)
{
	VM_IMPL(NavQuery);
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
NavQuery::NavQuery(std::shared_ptr<dtNavMesh> navMesh, int maxNode)
{
	VM_IMPL(NavQuery);
	_->m_navMesh = std::move(navMesh);
	_->m_navQuery = std::shared_ptr<dtNavMeshQuery>(dtAllocNavMeshQuery(), [](dtNavMeshQuery* p) {dtFreeNavMeshQuery(p); });
	_->m_navQuery->init(_->m_navMesh.get(), maxNode);
}
