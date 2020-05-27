#include <iostream>
#include <Pather.h>
#include <VMUtils/timer.hpp>
#include <VMat/geometry.h>
#include <VMUtils/fmt.hpp>

#include "VMat/numeric.h"

int main()
{
	using namespace std;
	using namespace vm;
	string meshFile = R"(D:\test_scene_mesh.obj)";
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
	desc.EdgeMaxLen  = 12;
	desc.EdgeMaxError = 1.3;
	desc.VertsPerPoly = 6;
	desc.DetailSampleDist = 6;
	desc.DetailSampleMaxError = 1;

	auto ctx = std::shared_ptr<rcContext>(new rcContext);

	auto geom = shared_ptr<InputGeom>(new InputGeom());
	geom->load(ctx.get(), meshFile);
	auto navMesh = CreateNavMesh(ctx,geom, extSettings,desc);
	

	navMesh->SaveAs(navMeshSavePath);

	//auto saveNavMesh = LoadNavMesh(navMeshSavePath);


	Point3f agentInitPos(-22.209324, 1.400002 ,- 12.012280);
	Point3f agentTargetPos(13.626881, 1.400002, 5.984295);


	const auto agID = navMesh->AddAgent(agentInitPos);
	navMesh->MoveAgentTo(agID, agentTargetPos);

	Timer timer;
	timer.start();

	auto prev = timer.elapsed().s();

	float timeAcc = 0.f;
	float sim = 0.02;
	float printTime = 0.f;


	for(const auto & p:navMesh->GetRandomPosition(5))
	{
		println("{}", p);
	}

	for(const auto & p:navMesh->GetRandomPositionAroundCircle(10,agentTargetPos,5))
	{
		println("{}", p);
	}

	while (true)
	{
		auto dt = timer.elapsed().s() - prev;

		if(timer.elapsed().s() - timeAcc > sim)
		{
			timeAcc = timer.elapsed().s();
			navMesh->Simulate(sim);
		}
		if(timer.elapsed().s() - printTime > 0.5)
		{
			printTime = timer.elapsed().s();
			println("{}", navMesh->GetAgentCurrentPosition(agID));
		}
	}
	
	return 0;
}
