
#pragma once
#include <VMUtils/common.h>
#include <VMUtils/concepts.hpp>
#include <VMUtils/json_binding.hpp>
#include <VMat/geometry.h>
#include <InputGeom.h>
#include <Recast.h>
#include <Pather_config.h>
#include "DetourCrowd.h"
#include <vector>
#include <Crowd.h>

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
	POLYFLAGS_DISABLED = 0x10,	// Disabled polygon
	POLYFLAGS_ALL = 0xffff	// All abilities.
};


//struct NavMeshDesc:vm::json::Serializable<NavMeshDesc>
//{
//	VM_JSON_FIELD(float, CellSize);
//	VM_JSON_FIELD(float, CellHeight);
//	VM_JSON_FIELD(float, AgentHeight);
//	VM_JSON_FIELD(float, AgentRadius);
//	VM_JSON_FIELD(float, AgentMaxClimb);
//	VM_JSON_FIELD(float, AgentMaxSlope);
//	VM_JSON_FIELD(float, RegionMinSize);
//	VM_JSON_FIELD(float, RegionMergeSize);
//	VM_JSON_FIELD(float, EdgeMaxLen);
//	VM_JSON_FIELD(float, EdgeMaxError);
//	VM_JSON_FIELD(float, VertsPerPoly);
//	VM_JSON_FIELD(float, DetailSampleDist);
//	VM_JSON_FIELD(float, DetailSampleMaxError);
//};

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

class Pather__pImpl;
class PATHER_EXPORTS Pather:vm::NoCopy,vm::NoMove
{
	VM_DECL_IMPL(Pather)
public:
	bool SaveAs(const std::string& fileName);
	
	int AddAgent(const Point3f & pos,const AgentDesc & ap);

	void RemoveAgent(int idx);

	void MoveAllAgentsTo(const Point3f & target);

	void MoveAgentTo(int idx, const Point3f& target);

	Point3f GetAgentCurrentPosition(int idx);

	std::vector<Point3f> GetRandomPosition(int count);

	std::vector<Point3f> GetRandomPositionAroundCircle(int count,const Point3f & center,float radius);

	void Simulate(float dt);
	
private:
	Pather(std::shared_ptr<InputGeom> geom,const ExternalSettings & es);
	Pather(std::shared_ptr<dtNavMesh> navMesh);
	
	
	void SetExternalSettings(const ExternalSettings& es);
	bool Build(std::shared_ptr<rcContext> ctx,const NavMeshDesc & desc);
	void InitCrowd(const CrowdDesc & desc);
	void InitQuery();

	friend class NavQuery;
	friend class NavCrowd;

	friend std::shared_ptr<Pather> BuildPather(std::shared_ptr<rcContext> ctx, std::shared_ptr<InputGeom> geom, const ExternalSettings& es, const NavMeshDesc& desc);
	friend std::shared_ptr<Pather> CreatePather(std::shared_ptr<dtNavMesh> navMesh,const CrowdDesc & desc);
};


std::shared_ptr<Pather> BuildPather(std::shared_ptr<rcContext> ctx, std::shared_ptr<InputGeom> geom, const ExternalSettings& es, const NavMeshDesc& desc);

std::shared_ptr<Pather> CreatePather(std::shared_ptr<dtNavMesh> navMesh,const CrowdDesc & desc);

std::shared_ptr<dtNavMesh> LoadNavMesh(const std::string& fileName);

