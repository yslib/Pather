#include <memory>
#include <PathServer.h>

static std::shared_ptr<NavMesh> navMesh = nullptr;
static std::shared_ptr<rcContext> ctx = nullptr;

static bool is_init = false;


bool init()
{
	auto ctx = std::shared_ptr<rcContext>(new rcContext);

	return is_init = true;
}

bool open_from_navmesh_file(const char* fn)
{
	return false;
}

bool open_from_scene_mesh_file(const char* fn)
{
	auto geom = shared_ptr<InputGeom>(new InputGeom());
	geom->load(ctx.get(), std::string(fn));
	ExternalSettings extSettings;
	NavMeshDesc desc;

	desc.CellSize = 0.17;
	desc.CellHeight = 0.17;
	desc.AgentHeight = 1.0;
	desc.AgentRadius = 0.3;
	desc.AgentMaxClimb = 0.9;
	desc.AgentMaxSlope = 45;
	desc.RegionMinSize = 8;
	desc.RegionMergeSize = 20;
	desc.EdgeMaxLen = 12;
	desc.EdgeMaxError = 1.3;
	desc.VertsPerPoly = 6;
	desc.DetailSampleDist = 6;
	desc.DetailSampleMaxError = 1;

	navMesh = CreateNavMesh(ctx, geom, extSettings, desc);

	return navMesh != nullptr;
}

int add_agent(const float* pos)
{
	assert(navMesh);
	Point3f p(pos[0], pos[1], pos[2]);
	return navMesh->AddAgent(p);
}

void remove_agent(int idx)
{
	assert(navMesh);
	navMesh->RemoveAgent(idx);
}

void move_agent_to(int idx, const float* target)
{
	assert(navMesh);
	Point3f t(target[0], target[1], target[2]);
	navMesh->MoveAgentTo(idx, t);
}

void move_all_agents_to(const float* target)
{
	assert(navMesh);
	Point3f t(target[0], target[1], target[2]);
	navMesh->MoveAllAgentsTo(t);
}

void gen_rand_pos(int expect_count, float* pos, int* exact_count)
{
	assert(navMesh);
	assert(pos);
	assert(expect_count);
	auto res = navMesh->GetRandomPosition(expect_count);
	memcpy(pos, res.data(), sizeof(Point3f) * res.size());
	*exact_count = res.size();
}

void gen_rand_pos_around_circle(int expect_count, const float* center, float radius, float* pos, int* exact_count)
{
	assert(navMesh);
	assert(pos);
	assert(expect_count);
	Point3f c(center[0], center[1], center[2]);
	auto res = navMesh->GetRandomPositionAroundCircle(expect_count, c, radius);
	memcpy(pos, res.data(), sizeof(Point3f) * res.size());
	*exact_count = res.size();
}





