#pragma once

#include <Pather.h>

#define C_FUNC_EXPORT extern "C" __declspec(dllexport)

#define CALL_SPEC _stdcall

using namespace std;

typedef int pather_handle;

//C_FUNC_EXPORT bool CALL_SPEC init();
//
//C_FUNC_EXPORT void CALL_SPEC simulate(float dt);
//
//C_FUNC_EXPORT void CALL_SPEC save(const char* fn);
//
//C_FUNC_EXPORT bool CALL_SPEC open_from_navmesh_file(const char* fn);
//
//C_FUNC_EXPORT bool CALL_SPEC open_from_scene_mesh_file_with_params(const char* fn, const NavMeshDesc * desc);
//
//C_FUNC_EXPORT bool CALL_SPEC open_from_scene_mesh_file(const char* fn);
//
//C_FUNC_EXPORT int CALL_SPEC add_agent(const float* pos);
//
//C_FUNC_EXPORT void CALL_SPEC remove_agent(int idx);
//
//C_FUNC_EXPORT void CALL_SPEC move_agent_to(int idx, const float* target);
//
//C_FUNC_EXPORT void CALL_SPEC move_all_agents_to(const float* target);
//
//C_FUNC_EXPORT bool CALL_SPEC get_agent_current_position(int idx, float* pos);
//
//C_FUNC_EXPORT void CALL_SPEC gen_rand_pos(int expect_count, float* pos, int* exact_count);
//
//C_FUNC_EXPORT void CALL_SPEC gen_rand_pos_around_circle(int expect_count, const float* center, float radius, float* pos, int* exact_count);
//


// new api

C_FUNC_EXPORT pather_handle CALL_SPEC build_pather(const char* mesh_file,const NavMeshDesc * desc);

C_FUNC_EXPORT pather_handle CALL_SPEC create_pather(const char* nav_mesh_file, const CrowdDesc * desc);

C_FUNC_EXPORT void CALL_SPEC destroy_pather(pather_handle);

C_FUNC_EXPORT int CALL_SPEC add_agent(pather_handle,const float* pos);

C_FUNC_EXPORT int CALL_SPEC add_agent_desc(pather_handle p, const float* pos, const AgentDesc* desc);

C_FUNC_EXPORT void CALL_SPEC simulate(pather_handle, float dt);

C_FUNC_EXPORT void CALL_SPEC save(pather_handle,const char * fn);

C_FUNC_EXPORT void CALL_SPEC remove_agent(pather_handle,int idx);

C_FUNC_EXPORT void CALL_SPEC move_agent_to(pather_handle,int idx, const float* target);

C_FUNC_EXPORT void CALL_SPEC move_all_agents_to(pather_handle,const float* target);

C_FUNC_EXPORT bool CALL_SPEC get_agent_current_position(pather_handle,int idx, float* pos);

C_FUNC_EXPORT void CALL_SPEC gen_rand_pos(pather_handle,int expect_count, float* pos, int* exact_count);

C_FUNC_EXPORT void CALL_SPEC gen_rand_pos_around_circle(pather_handle,int expect_count, const float* center, float radius, float* pos, int* exact_count);

