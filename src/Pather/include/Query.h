
#pragma once

#include <VMUtils/common.h>
#include <VMUtils/concepts.hpp>
#include <Pather_config.h>

#include <Pather.h>

class NavQuery__pImpl;
class PATHER_EXPORTS NavQuery :vm::NoCopy, vm::NoMove {
	VM_DECL_IMPL(NavQuery)
public:
	std::vector<Point3f> GetRandomPosition(int count);
	std::vector<Point3f> GetRandomPositionAroundCircle(int count, const Point3f& center, float radius);
private:
	NavQuery(std::shared_ptr<dtNavMesh> navMesh, int maxNode);
};