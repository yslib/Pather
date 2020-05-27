
#pragma once

#include <Pather.h>

#define C_FUNC_EXPORT extern "C" __declspec(dllexport)

using namespace std;


C_FUNC_EXPORT bool init();

C_FUNC_EXPORT bool open_from_navmesh_file(const char* fn);

C_FUNC_EXPORT bool open_from_scene_mesh_file(const char* fn);

C_FUNC_EXPORT int add_agent(const float* pos);

C_FUNC_EXPORT void remove_agent(int idx);

C_FUNC_EXPORT void move_agent_to(int idx, const float* target);

C_FUNC_EXPORT void move_all_agents_to(const float* target);

C_FUNC_EXPORT void gen_rand_pos(int expect_count, float* pos, int* exact_count);

C_FUNC_EXPORT void gen_rand_pos_around_circle(int expect_count, const float* center, float radius, float* pos, int* exact_count);
