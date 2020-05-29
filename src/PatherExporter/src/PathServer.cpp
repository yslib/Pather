#include <memory>
#include <PathServer.h>
#include <VMUtils/fmt.hpp>
#include <VMUtils/log.hpp>

namespace {
	//std::shared_ptr<Pather> pather = nullptr;
	//std::shared_ptr<rcContext> ctx = nullptr;
	//bool is_init = false;
	//bool init()
	//{
	//	ctx = std::shared_ptr<rcContext>(new rcContext);
	//	ctx->enableLog(true);

	//	return is_init = true;
	//}


	//bool open_from_navmesh_file(const char* fn)
	//{
	//	return false;
	//}

	//bool open_from_scene_mesh_file_with_params(const char* fn, const NavMeshDesc* desc)
	//{
	//	auto name = std::string(fn);
	//	auto geom = shared_ptr<InputGeom>(new InputGeom());
	//	geom->load(ctx.get(), name);
	//	ExternalSettings extSettings;
	//	pather = BuildPather(ctx, geom, extSettings, *desc);
	//	println("scene is loaded");
	//	return pather != nullptr;

	//}

	//bool open_from_scene_mesh_file(const char* fn)
	//{
	//	NavMeshDesc desc;
	//	desc.CellSize = 0.17;
	//	desc.CellHeight = 0.17;
	//	desc.AgentHeight = 1.0;
	//	desc.AgentRadius = 0.3;
	//	desc.AgentMaxClimb = 0.9;
	//	desc.AgentMaxSlope = 45;
	//	desc.RegionMinSize = 8;
	//	desc.RegionMergeSize = 20;
	//	desc.EdgeMaxLen = 12;
	//	desc.EdgeMaxError = 1.3;
	//	desc.VertsPerPoly = 6;
	//	desc.DetailSampleDist = 6;
	//	desc.DetailSampleMaxError = 1;
	//	return false;
	//	//return open_from_scene_mesh_file_with_params(fn, &desc);
	//}

	//int add_agent(const float* pos)
	//{
	//	assert(pather);
	//	Point3f p(pos[0], pos[1], pos[2]);
	//	AgentDesc ap;
	//	memset(&ap, 0, sizeof(ap));

	//	ap.radius = 0.1;
	//	ap.height = 1;

	//	ap.maxAcceleration = 8.0f;
	//	ap.maxSpeed = 3.5f;
	//	ap.collisionQueryRange = ap.radius * 12.0f;
	//	ap.pathOptimizationRange = ap.radius * 30.0f;
	//	ap.updateFlags = 0;
	//	//if (m_toolParams.m_anticipateTurns)
	//	ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	//	//if (m_toolParams.m_optimizeVis)
	//	ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	//	//if (m_toolParams.m_optimizeTopo)
	//	ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	//	//if (m_toolParams.m_obstacleAvoidance)
	//	ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	//	//if (m_toolParams.m_separation)
	//	ap.updateFlags |= DT_CROWD_SEPARATION;
	//	ap.obstacleAvoidanceType = 0; // (unsigned char)m_toolParams.m_obstacleAvoidanceType;
	//	ap.separationWeight = 2.0f; // m_toolParams.m_separationWeight;
	//	return pather->AddAgent(p, ap);
	//}

	//void remove_agent(int idx)
	//{
	//	assert(pather);
	//	pather->RemoveAgent(idx);
	//}

	//void move_agent_to(int idx, const float* target)
	//{
	//	assert(pather);
	//	Point3f t(target[0], target[1], target[2]);
	//	pather->MoveAgentTo(idx, t);
	//}

	//void move_all_agents_to(const float* target)
	//{
	//	assert(pather);
	//	Point3f t(target[0], target[1], target[2]);
	//	pather->MoveAllAgentsTo(t);
	//}

	//bool get_agent_current_position(int idx, float* pos)
	//{
	//	const auto p = pather->GetAgentCurrentPosition(idx);
	//	memcpy(pos, p.ConstData(), sizeof(Point3f));
	//	return true;
	//}


	//void gen_rand_pos(int expect_count, float* pos, int* exact_count)
	//{
	//	assert(pather);
	//	assert(pos);
	//	assert(expect_count);
	//	auto res = pather->GetRandomPosition(expect_count);
	//	memcpy(pos, res.data(), sizeof(Point3f) * res.size());
	//	*exact_count = res.size();
	//}

	//void gen_rand_pos_around_circle(int expect_count, const float* center, float radius, float* pos, int* exact_count)
	//{
	//	assert(pather);
	//	assert(pos);
	//	assert(expect_count);
	//	Point3f c(center[0], center[1], center[2]);
	//	auto res = pather->GetRandomPositionAroundCircle(expect_count, c, radius);
	//	memcpy(pos, res.data(), sizeof(Point3f) * res.size());
	//	*exact_count = res.size();
	//}

	//void simulate(float dt)
	//{
	//	assert(pather);
	//	pather->Simulate(dt);
	//}

	//void save(const char* fn)
	//{

	//}
}

struct PathServer
{
	std::shared_ptr<Pather> pather;
	std::shared_ptr<rcContext> ctx;
	PathServer() = default;
	PathServer(std::shared_ptr<Pather> p, std::shared_ptr<rcContext> c) :pather(std::move(p)), ctx(std::move(c)) {}
};

static std::vector<PathServer> servers;

pather_handle build_pather(const char* mesh_file, const NavMeshDesc * desc)
{
	auto name = std::string(mesh_file);
	auto geom = shared_ptr<InputGeom>(new InputGeom());
	auto ctx = std::shared_ptr<rcContext>(new rcContext);

	geom->load(ctx.get(), name);
	ExternalSettings extSettings;
	auto pather = BuildPather(ctx, geom, extSettings, *desc);

	servers.emplace_back(std::move(pather), std::move(ctx));

	return servers.size() - 1;
}

pather_handle create_pather(const char* nav_mesh_file, const CrowdDesc* desc)
{
	auto name = std::string(nav_mesh_file);
	auto navMesh = LoadNavMesh(name);
	auto pather = CreatePather(navMesh, *desc);

	auto ctx = std::shared_ptr<rcContext>(new rcContext);
	ctx->enableLog(true);
	servers.emplace_back(std::move(pather), std::move(ctx));
	return servers.size() - 1;
}

void destroy_pather(pather_handle p)
{
	Debug("server:[{}][destroy_pather]", p);
	servers[p] = PathServer();
}

int add_agent(pather_handle p, const float* pos)
{
	Debug("server:[{}][add_agent]", p);
	
	auto& pather = servers[p].pather;

	assert(pather);
	Point3f po(pos[0], pos[1], pos[2]);
	AgentDesc ap;
	memset(&ap, 0, sizeof(ap));

	ap.radius = 0.1;
	ap.height = 1;

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
	return pather->AddAgent(po, ap);
}

void simulate(pather_handle p, float dt)
{
	Debug("server:[{}][simulate]", p);
	auto& pather = servers[p].pather;
	assert(pather);
	pather->Simulate(dt);
}

void save(pather_handle p, int idx)
{
	Debug("server:[{}][save]", p);
	auto& pather = servers[p].pather;
	assert(pather);
}

void remove_agent(pather_handle p, int idx)
{
	Debug("server:[{}][remove_agent]", p);
	auto& pather = servers[p].pather;
	assert(pather);
	pather->RemoveAgent(idx);
}

void move_agent_to(pather_handle p, int idx, const float* target)
{
	Debug("server:[{}][move_agent_to]", p);
	auto& pather = servers[p].pather;
	assert(pather);
	Point3f t(target[0], target[1], target[2]);
	pather->MoveAgentTo(idx, t);
}

void move_all_agents_to(pather_handle p, const float* target)
{
	Debug("server:[{}][move_all_agents_to]", p);
	auto& pather = servers[p].pather;
	assert(pather);
	Point3f t(target[0], target[1], target[2]);
	pather->MoveAllAgentsTo(t);
}

bool get_agent_current_position(pather_handle p, int idx, float* pos)
{
	Debug("server:[{}][get_agent_current_position]", p);
	auto& pather = servers[p].pather;
	assert(pather);
	const auto cp = pather->GetAgentCurrentPosition(idx);
	memcpy(pos, cp.ConstData(), sizeof(Point3f));
	return true;
}

void gen_rand_pos(pather_handle p, int expect_count, float* pos, int* exact_count)
{
	Debug("server:[{}][gen_rand_pos]", p);
	auto& pather = servers[p].pather;
	assert(pather);
	assert(pos);
	assert(expect_count);
	auto res = pather->GetRandomPosition(expect_count);
	memcpy(pos, res.data(), sizeof(Point3f) * res.size());
	*exact_count = res.size();
}

void gen_rand_pos_around_circle(pather_handle p, int expect_count, const float* center, float radius, float* pos,
	int* exact_count)
{
	Debug("server:[{}][gen_rand_pos_around_circle]", p);
	auto& pather = servers[p].pather;
	assert(pather);
	assert(pos);
	assert(expect_count);
	Point3f c(center[0], center[1], center[2]);
	auto res = pather->GetRandomPositionAroundCircle(expect_count, c, radius);
	memcpy(pos, res.data(), sizeof(Point3f) * res.size());
	*exact_count = res.size();
}

void save(pather_handle p, const char* fn)
{
	Debug("server:[{}][save]", p);
	auto& pather = servers[p].pather;
	assert(pather);
	pather->SaveAs(fn);
}



