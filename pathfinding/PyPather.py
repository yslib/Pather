
from ctypes import *
import ctypes
import sys
import time

reload(sys)
sys.setdefaultencoding('gb2312')
dll = ctypes.windll.LoadLibrary('D:/Mini/trunk/server/lib/PathServer.dll')
pos_type = c_float * 3

class NavMeshDesc(ctypes.Structure):
    _fields_ = [
        ("CellSize", c_float),
        ("CellHeight", c_float),
        ("AgentHeight", c_float),
        ("AgentRadius", c_float),
        ("AgentMaxClimb", c_float),
        ("RegionMinSize", c_float),
        ("RegionMergeSize", c_float),
        ("EdgeMaxLen", c_float),
        ("EdgeMaxError", c_float),
        ("VertsPerPoly", c_float),
        ("DetailSampleDist", c_float),
        ("DetailSampleMaxError", c_float)
        ]

class CrowdDesc(ctypes.Structure):
    _fields_ = [
        ("MaxAgentCount", c_int),
        ("MaxAgentRadius", c_float)
        ]


class Pather(object):

    def __init__(self):
        self.handle = -1

    def __destory(self):
        if self.handle >= 0:
            func = dll.destroy_pather
            func.argtypes = [c_int]
            func(self.handle)
            self.handle = -1

    def __del__(self):
        self.__destory()

    def build_pather(self, mesh_file, nav_mesh_desc):
        self.__destory()
        func = dll.build_pather
        func.argtypes = [c_char_p, POINTER(NavMeshDesc)]
        func.restype = c_int
        self.handle = func(c_char_p(mesh_file), byref(nav_mesh_desc))

    def create_pather(self, nav_mesh_file, crowd_desc):
        self.__destory()
        func = dll.create_pather
        func.argtypes = [c_char_p, POINTER(CrowdDesc)]
        self.handle = func(c_char_p(nav_mesh_file), byref(crowd_desc))

    def add_agent(self,pos):
        func = dll.add_agent
        func.argtypes = [c_int, POINTER(c_float)]
        func.restype = c_int
        p = pos_type()
        p[0] = pos[0]
        p[1] = pos[1]
        p[2] = pos[2]
        return func(c_int(self.handle),p)

    def remove_agent(self,idx):
        func = dll.remove_agent
        func.argtypes = [c_int,c_int]
        return func(c_int(self.handle),c_int(idx))

    def move_agent_to(self, idx, target_pos):
        func = dll.move_agent_to
        func.argtypes = [c_int, c_int, POINTER(c_float)]

        p = pos_type()
        p[0] = target_pos[0]
        p[1] = target_pos[1]
        p[2] = target_pos[2]
        return func(c_int(self.handle),c_int(idx), p)

    def move_all_agents_to(self, target_pos):
        func = dll.move_all_agents_to
        func.argtypes = [c_int,POINTER(c_float)]
        p = pos_type()
        p[0] = target_pos[0]
        p[1] = target_pos[1]
        p[2] = target_pos[2]
        return func(c_int(self.handle),p)

    def get_agent_current_position(self,idx):
        func = dll.get_agent_current_position
        func.argtypes = [c_int,c_int, pos_type]
        func.restype = c_bool
        pos = pos_type()
        res = func(c_int(self.handle),c_int(idx), pos)
        # bytearray a;
        p = [0] * 3
        if (res == False):
            return []
        p[0] = pos[0]
        p[1] = pos[1]
        p[2] = pos[2]
        return p

    def simulate(self, dt):
        dll.simulate(c_int(self.handle),c_float(dt))

    def gen_rand_pos(self, count):
        func = dll.gen_rand_pos
        func.argtypes = [c_int , c_int, POINTER(c_float), POINTER(c_int)]

        res_pos_type = c_float * (3 * count)
        res_pos = res_pos_type()

        res_count = c_int()
        func(c_int(self.handle),c_int(count), res_pos, byref(res_count))
        res = []
        for i in range(res_count.value):
            res.append([res_pos[i * 3], res_pos[i * 3 + 1], res_pos[i * 3 + 2]])
        return res

    def gen_rand_pos_around_circle(self,count, center, radius):
        func = dll.gen_rand_pos_around_circle
        func.argtypes = [c_int,c_int, POINTER(c_float), c_float, POINTER(c_float), POINTER(c_int)]

        res_pos_type = c_float * (3 * count)
        res_pos = res_pos_type()

        res_count = c_int()

        _center = (c_float * 3)()
        _center[0] = center[0]
        _center[1] = center[1]
        _center[2] = center[2]

        func(c_int(self.handle),c_int(count), _center, c_float(radius), res_pos, byref(res_count))
        res = []
        for i in range(res_count.value):
            res.append([res_pos[i * 3], res_pos[i * 3 + 1], res_pos[i * 3 + 2]])
        return res

    def save(self, filename):
        dll.save(c_int(self.handle),c_char_p(filename))


if __name__ == "__main__":

    mesh_obj_file = "D:\\scene_mesh.obj"
    nav_mesh_file = "D:\\solo_navmesh.bin"
    # desc = NavMeshDesc()
    # desc.CellSize = 0.17
    # desc.CellHeight = 0.17
    # desc.AgentHeight = 1.0
    # desc.AgentRadius = 0.3
    # desc.AgentMaxClimb = 0.9
    # desc.AgentMaxSlope = 45
    # desc.RegionMinSize = 8
    # desc.RegionMergeSize = 20
    # desc.EdgeMaxLen = 12
    # desc.EdgeMaxError = 1.3
    # desc.VertsPerPoly = 6
    # desc.DetailSampleDist = 6
    # desc.DetailSampleMaxError = 1

    pather = Pather()
    cd = CrowdDesc()
    cd.MaxAgentCount = 10
    cd.MaxAgentRadius = 1
    pather.create_pather(nav_mesh_file, cd)

    startPos = [-22.209324, 1.400002,- 12.012280]
    endPos = [13.626881, 1.400002, 5.984295]
    agid = pather.add_agent(startPos)
    print "agid",agid
    pather.move_agent_to(agid, endPos)

    curTime = time.time()
    simulateTime = time.time()
    printTime = time.time()

    simulateStep = 0.02

    for i in range(100):
        simulateTime = time.time()
        pather.simulate(simulateStep)
        print pather.get_agent_current_position(agid)

    for p in pather.gen_rand_pos(5):
        print p,"====="

    for p in pather.gen_rand_pos_around_circle(10, [0, 0, 0] , 5):
        print p, "----"













