
#include <Crowd.h>

#include "Pather.h"


class NavCrowd__pImpl
{
	VM_DECL_API(NavCrowd);
public:
	NavCrowd__pImpl(NavCrowd * api):q_ptr(api){}
	std::shared_ptr<dtCrowd> m_crowd;
	std::shared_ptr<dtNavMesh> m_navMesh;
};


void NavCrowd::Simulate(float dt)
{
	VM_IMPL(NavCrowd)
	auto crowd = _->m_crowd;
	assert(crowd);
	if (!crowd) return;
	crowd->update(dt, nullptr);
}

int NavCrowd::AddAgent(const Point3f & pos, const AgentDesc& ap)
{
	VM_IMPL(NavCrowd)
	auto crowd = _->m_crowd.get();
	assert(crowd);
	auto idx = crowd->addAgent(pos.ConstData(), &ap);
	return idx;
}

void NavCrowd::RemoveAgent(int idx)
{
	VM_IMPL(NavCrowd)
	auto crowd = _->m_crowd.get();
	assert(crowd);
	crowd->removeAgent(idx);
}

vm::Point3f NavCrowd::GetAgentCurrentPosition(int idx)
{
	VM_IMPL(NavCrowd);
	auto crowd = _->m_crowd.get();
	assert(crowd);
	auto filter = crowd->getFilter(0);
	auto ag = crowd->getAgent(idx);
	assert(ag);
	if(ag->active == false)
	{
		return vm::Point3f();
	}
	return vm::Point3f(ag->npos[0], ag->npos[1], ag->npos[2]);
}

NavCrowd::NavCrowd(std::shared_ptr<dtNavMesh> navMesh, int maxAgent, float maxAgentRadius)
{
	VM_IMPL(NavCrowd)

	_->m_crowd = std::shared_ptr<dtCrowd>(dtAllocCrowd(), [](dtCrowd* p) {dtFreeCrowd(p); });

	auto nav = _->m_navMesh.get();
	auto crowd = _->m_crowd.get();

	assert(nav && crowd);


	crowd->init(maxAgent, maxAgentRadius, nav);

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
