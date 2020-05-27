
#pragma once
#include <VMUtils/common.h>
#include <VMUtils/concepts.hpp>
#include <VMat/geometry.h>
#include <InputGeom.h>
#include <Recast.h>
#include <Pather_config.h>
#include "DetourCrowd.h"
#include <vector>

using namespace vm;

enum EPartitionType
{
	Sample_Partition_Watershed,
	Sample_Partition_Monoton,
};

struct ExternalSettings
{
	bool KeepInterResult = false;
	bool FilterLowHangingObstacles = false;
	bool FilterLedgeSpan = false;
	bool FilterWalkableLowHeighSpan = false;
	EPartitionType PartitionType;
};

enum EPolyAreas
{
	POLYAREA_GROUND,
	POLYAREA_WATER,
	POLYAREA_ROAD,
	POLYAREA_DOOR,
	POLYAREA_GRASS,
	POLYAREA_JUMP,
};
enum EPolyFlags
{
	POLYFLAGS_WALK = 0x01,		// Ability to walk (ground, grass, road)
	POLYFLAGS_SWIM = 0x02,		// Ability to swim (water).
	POLYFLAGS_DOOR = 0x04,		// Ability to move through doors.
	POLYFLAGS_JUMP = 0x08,		// Ability to jump.
	POLYFLAGS_DISABLED = 0x10,		// Disabled polygon
	POLYFLAGS_ALL = 0xffff	// All abilities.
};


struct NavMeshDesc
{
	float CellSize;
	float CellHeight;
	float AgentHeight;
	float AgentRadius;
	float AgentMaxClimb;
	float AgentMaxSlope;
	float RegionMinSize;
	float RegionMergeSize;
	float EdgeMaxLen;
	float EdgeMaxError;
	float VertsPerPoly;
	float DetailSampleDist;
	float DetailSampleMaxError;
};

class NavMesh__pImpl;
class PATHER_EXPORTS NavMesh:vm::NoCopy,vm::NoMove
{
	VM_DECL_IMPL(NavMesh)
public:
	bool SaveAs(const std::string& fileName);
	
	int AddAgent(const Point3f & pos);

	void RemoveAgent(int idx);

	void MoveAllAgentsTo(const Point3f & target);

	void MoveAgentTo(int idx, const Point3f& target);

	Point3f GetAgentCurrentPosition(int idx);

	std::vector<Point3f> GetRandomPosition(int count);

	std::vector<Point3f> GetRandomPositionAroundCircle(int count,const Point3f & center,float radius);
	//std::vector<Point3f> GetNearestRandomPosition(int count);
	void Simulate(float dt);

private:
	NavMesh(std::shared_ptr<InputGeom> geom,const ExternalSettings & es);
	NavMesh(std::shared_ptr<dtNavMesh> navMesh);
	void SetExternalSettings(const ExternalSettings& es);
	bool Build(std::shared_ptr<rcContext> ctx,const NavMeshDesc & desc);
	void InitCrowd();
	void InitQuery();

	friend std::shared_ptr<NavMesh> CreateNavMesh(std::shared_ptr<rcContext> ctx, std::shared_ptr<InputGeom> geom, const ExternalSettings& es, const NavMeshDesc& desc);
	friend std::shared_ptr<NavMesh> LoadNavMesh(const std::string& fileName);
};

std::shared_ptr<NavMesh> CreateNavMesh(std::shared_ptr<rcContext> ctx, std::shared_ptr<InputGeom> geom, const ExternalSettings& es, const NavMeshDesc& desc);
std::shared_ptr<NavMesh> LoadNavMesh(const std::string& fileName);




