#include <iostream>
#include <Pather.h>
#include <VMUtils/timer.hpp>
#include <VMat/geometry.h>
#include <VMUtils/fmt.hpp>
#include <Crowd.h>

int main()
{
	using namespace std;
	using namespace vm;
	string meshFile = R"(D:\scene_mesh.obj)";
	string navMeshSavePath = R"(D:\test_nav_mesh_data.bin)";
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


	AgentDesc ap;
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
	

	auto ctx = std::shared_ptr<rcContext>(new rcContext);

	auto geom = shared_ptr<InputGeom>(new InputGeom());
	geom->load(ctx.get(), meshFile);
	auto pather = BuildPather(ctx, geom, extSettings, desc);

	pather->SaveAs(navMeshSavePath);

	

	auto newNavMesh = LoadNavMesh(navMeshSavePath);

	CrowdDesc cd;
	cd.MaxAgentCount = 10;
	cd.MaxAgentRadius = 1;
	pather = CreatePather(newNavMesh, cd);

	Point3f agentInitPos(12.800000190734864, 1, -22.012280);
	Point3f agentTargetPos(13.626881, 1.400002, 5.984295);


	const auto agID = pather->AddAgent(agentInitPos,ap);
	pather->MoveAgentTo(agID, agentTargetPos);

	Timer timer;
	timer.start();

	auto prev = timer.elapsed().s();

	float timeAcc = 0.f;
	float sim = 0.02;
	float printTime = 0.f;


	for (const auto& p : pather->GetRandomPosition(5))
	{
		println("{}", p);
	}

	for (const auto& p : pather->GetRandomPositionAroundCircle(10, agentTargetPos, 5))
	{
		println("{}", p);
	}

	while (true)
	{
		auto dt = timer.elapsed().s() - prev;

		if (timer.elapsed().s() - timeAcc > sim)
		{
			timeAcc = timer.elapsed().s();
			pather->Simulate(sim);
		}
		if (timer.elapsed().s() - printTime > 0.5)
		{
			printTime = timer.elapsed().s();
			println("{}", pather->GetAgentCurrentPosition(agID));
		}
	}

	return 0;
}
