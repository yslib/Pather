
from ctypes import *
import ctypes
import sys
import time

reload(sys)
sys.setdefaultencoding('gb2312')
dll = ctypes.windll.LoadLibrary('lib/PathServer.dll')

dll.init()

pos_type = c_float * 3

def open_from_navmesh_file(fn):
    return dll.open_from_navmesh_file(c_char_p(fn))


def open_from_scene_mesh_file(fn):
    return dll.open_from_scene_mesh_file(c_char_p(fn))


def add_agent(pos):
    func = dll.add_agent
    func.argtypes = [POINTER(c_float)]
    func.restype = c_int
    p = pos_type()
    p[0] = pos[0]
    p[1] = pos[1]
    p[2] = pos[2]
    return func(p)


def remove_agent(idx):
    func = dll.remove_agent
    func.argtypes = [c_int]
    return func(c_int(idx))


def move_agent_to(idx, target_pos):
    func = dll.move_agent_to
    func.argtypes=[c_int,POINTER(c_float)]

    p = pos_type()
    p[0] = target_pos[0]
    p[1] = target_pos[1]
    p[2] = target_pos[2]
    return func(c_int(idx),p)


def move_all_agents_to(target_pos):
    func = dll.move_all_agents_to
    func.argtypes=[POINTER(c_float)]
    p = pos_type()
    p[0] = target_pos[0]
    p[1] = target_pos[1]
    p[2] = target_pos[2]
    return func(p)

def get_agent_current_position(idx):
    func = dll.get_agent_current_position
    func.argtypes = [c_int, pos_type]
    func.restype = c_bool
    pos = pos_type()
    res = func(c_int(idx),pos)
    p = [0] * 3
    if(res == False):
        return []
    p[0] = pos[0]
    p[1] = pos[1]
    p[2] = pos[2]
    return p

def simulate(dt):
    dll.simulate(c_float(dt))


def gen_rand_pos(count):
    func = dll.gen_rand_pos
    func.argtypes=[c_int, POINTER(c_float), POINTER(c_int)]

    res_pos_type = c_float * (3 * count)
    res_pos = res_pos_type()

    res_count = c_int()
    func(c_int(count), res_pos, byref(res_count))
    res = []
    for i in range(res_count.value):
        res.append([res_pos[i*3], res_pos[i*3+1], res_pos[i*3+2]])
    return res

def gen_rand_pos_around_circle(count,center,radius):
    func = dll.gen_rand_pos_around_circle
    func.argtypes = [c_int, POINTER(c_float), c_float, POINTER(c_float), POINTER(c_int)]

    res_pos_type = c_float * (3 * count)
    res_pos = res_pos_type()

    res_count = c_int()

    _center = (c_float * 3)()
    _center[0] = center[0]
    _center[1] = center[1]
    _center[2] = center[2]

    func(c_int(count), _center, c_float(radius), res_pos, byref(res_count))
    res = []
    for i in range(res_count.value):
        res.append([res_pos[i * 3], res_pos[i * 3 + 1], res_pos[i * 3 + 2]])
    return res



if __name__ == "__main__":

    mesh_obj_file = "D:\\test_scene_mesh.obj"

    open_from_scene_mesh_file(mesh_obj_file)

    startPos = [-22.209324, 1.400002,- 12.012280]
    endPos = [13.626881, 1.400002, 5.984295]
    agid = add_agent(startPos)
    move_agent_to(agid, endPos)

    curTime = time.time()
    simulateTime = time.time()
    printTime = time.time()

    simulateStep = 0.02

    for i in range(100):
        simulateTime = time.time()
        simulate(simulateStep)
        print get_agent_current_position(agid)



    for p in gen_rand_pos(5):
        print p,"====="

    for p in gen_rand_pos_around_circle(10, [0, 0, 0] , 5):
        print p, "----"













