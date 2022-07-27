import os
import math
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Plane
from compas.geometry import Frame
from compas.geometry import Sphere
from compas.geometry import Box
from compas.geometry import Transformation
from compas.datastructures import Mesh

from compas_fab.backends import AnalyticalInverseKinematics
from compas_fab.backends import PyBulletClient
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import ReachabilityMap
from compas_fab.robots import CollisionMesh
from compas_fab.robots import Tool

from arch import CRA_Arch
from compas_cra.datastructures import CRA_Assembly

from math import radians

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, ".", "data")

# 0. Prepare tool head
def get_tool():

    mesh = Mesh.from_stl(os.path.join(HERE, "test0.stl"))
    frameTool = Frame([0.078, 0, 0], [0, 1, 0], [0, 0, 1])
    tool = Tool(None, frameTool, link_name="wrist_3_link")
    return tool

def adjust_to_attached_tool(robot, frame):
    if not robot.attached_tool:
        return frame
    return robot.from_tcf_to_t0cf([frame])[0]

# 1. Define generators


def frames_on_position_generator(robot, frame0):
    N = 12
    arch = CRA_Arch(rise = 150/1000, span = 300/1000, thickness = 20/1000, depth = 20/1000, n = N)
    wedge_angle = 2
    frames = []

    for i in range(N):
        if i == 0:
            pass
        elif i==N-1:
            pass
        elif i== N-2:
            _, af = arch.wedged_block(i,0,0)
            frames.extend(af)
        elif i==N-3:
            _, af = arch.wedged_block(i,0,0)
            frames.extend(af)
        else:
            _, af = arch.wedged_block(i,0,0)
            frames.extend(af)
    t = Transformation.from_frame(frame0)
    transformed_frames = [f.transformed(t) for f in frames]
    return [adjust_to_attached_tool(robot, transformed_frame) for transformed_frame in transformed_frames]


    # for theta_deg in range(0, 20, 20):
    #     for phi_deg in range(0, 10, 10):
    #         theta = math.radians(theta_deg)
    #         phi = math.radians(phi_deg)
    #         x = sphere.point.x + sphere.radius * math.cos(theta) * math.sin(phi)
    #         y = sphere.point.y + sphere.radius * math.sin(theta) * math.sin(phi)
    #         z = sphere.point.z + sphere.radius * math.cos(phi)
    #         point = Point(x, y, z)
    #         axis = sphere.point - point
    #         plane = Plane((x, y, z), axis)
    #         f = Frame.from_plane(plane)
    #         # for UR5 is zaxis the xaxis
    #         yield Frame(f.point, f.zaxis, f.yaxis)


def position_generator():
    frame = Frame(Point(0, 0, 0), (0,1,0),(-1,0,0))
    for x in [3]:
        for y in [ 3, 4,]:
            center = frame.point + Vector(x, y, 0) * 0.1
            yield  Frame(center, frame.xaxis, frame.yaxis)

# 2. Create 2D generator


def generator(robot):
    for position in position_generator():
        yield frames_on_position_generator(robot, position)

# 3. Create reachability map 2D


with PyBulletClient(connection_type='direct') as client:
    # load robot and define settings
    robot = client.load_ur5(load_geometry=True)
    tool = get_tool()
    if tool:
        robot.attach_tool(tool)
    
    # attach floor collision meshs
    box = Box.from_corner_corner_height([-1, -1, -0.2], [1, 1, -0.2], 0.19)
    mesh = Mesh.from_shape(box)
    cm = CollisionMesh(mesh, 'floor')
    client.add_collision_mesh(cm)

    # attach tip collision mesh
    gripper = Mesh.from_stl(os.path.join(HERE, "test0.stl"))
    cm2 = CollisionMesh(gripper, 'tip')
    acm = AttachedCollisionMesh(cm2, 'ee_link')
    client.add_attached_collision_mesh(acm, {'mass': 0.1, 'robot': robot})

    ik = AnalyticalInverseKinematics(client)
    client.inverse_kinematics = ik.inverse_kinematics
    options = {"solver": "ur5", "check_collision": True, "keep_order": True}

    # calculate reachability map
    map = ReachabilityMap()
    map.calculate(generator(robot), robot, options)
    print(map.score)
    print(map.shape)

    #save to json
    map.to_json(os.path.join(DATA, "map2D_spheres.json"), pretty = True)