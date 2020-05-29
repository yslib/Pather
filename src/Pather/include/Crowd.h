
#pragma once
#include <VMUtils/common.h>
#include <VMUtils/concepts.hpp>
#include <VMUtils/json_binding.hpp>
#include <VMat/geometry.h>
#include <Pather_config.h>
#include <DetourCrowd.h>



struct AgentDesc:public dtCrowdAgentParams
{
};

struct CrowdDesc:vm::json::Serializable<CrowdDesc>
{
	VM_JSON_FIELD(int, MaxAgentCount);
	VM_JSON_FIELD(float, MaxAgentRadius);
};

class NavCrowd__pImpl;
class PATHER_EXPORTS NavCrowd :vm::NoCopy, vm::NoMove {
	VM_DECL_IMPL(NavCrowd)
public:
	void Simulate(float dt);
	int AddAgent(const vm::Point3f & pos,const AgentDesc & ap);
	void RemoveAgent(int idx);
	vm::Point3f GetAgentCurrentPosition(int idx);
private:
	NavCrowd(std::shared_ptr<dtNavMesh> navMesh, int maxAgent, float maxAgentRadius);;
};
