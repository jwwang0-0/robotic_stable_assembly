import simple_comm_vs as c
import simple_ur_script_vs as ur

import math as m
import os
import time

import compas
from compas.robots import Configuration
from compas.topology import breadth_first_ordering

HERE = os.path.dirname(__file__)
# Load assembly
#filename = os.path.join(HERE, 'arch_assembly.json')
filename = os.path.join(HERE, '0628_urtest.json')
assembly = compas.json_load(filename)

script = ""
# script += ur.set_tcp_by_angles(float(-17.90), float(45.85), float(143.68), m.radians(0.0), m.radians(180.0), m.radians(90.0))
# script = c.concatenate_script(script)
# c.send_script(script,"192.168.10.10")

trajectory_keys = ("start_trajectory",
                   "pick_trajectory",
                   "pick_trajectory_after",
                   "freespace_trajectory",
                   "place_trajectory",
                   "place_trajectory_after",)

# NOTE: Replace this with assembly.find_by_key after this is merged:
# https://github.com/compas-dev/compas/pull/1032

#script = ""
#script += ur.set_tcp_by_angles(float(-17.90), float(45.85), float(143.68), m.radians(0.0), m.radians(180.0), m.radians(90.0))
velocity = 0.15
acceleration = 0.02

IO = 0

part = 2
part = assembly.graph.node_attribute(str(int(part)), "part")

for traj_name in trajectory_keys:
    traj = part.attributes[traj_name]
    length = len(traj.points)

    # if traj_name == "pick_trajectory_after":
    #     #ur_io.setStandardDigitalOut(IO,True)
    #     #time.sleep(2) 
    # elif traj_name == "place_trajectory_after":
    #     # ur_io.setStandardDigitalOut(IO,False)
    #     # time.sleep(2) 
    
    for i in range(length):
        config = traj.points[i].joint_values
    # Move robot the new pos
        speed = 1  # rad/s
        accel = 1.4  # rad/s^2
        nowait = False
        script += ur.move_j(config, acceleration, velocity,0.0, 0.01)
    script = c.concatenate_script(script)
    c.send_script(script.encode(), "192.168.10.10" )


    # while(not read_digital_output(robot1)):
    #     print('waiting to complete motion...')

#############################
    # time.sleep(5)
#############################

for traj_name in trajectory_keys:
    traj = part.attributes[traj_name]
    length = len(traj.points)

    # if traj_name == "pick_trajectory_after":
    #     #ur_io.setStandardDigitalOut(IO,True)
    #     #time.sleep(2) 
    # elif traj_name == "place_trajectory_after":
    #     # ur_io.setStandardDigitalOut(IO,False)
    #     # time.sleep(2) 
    
    for i in range(length):
        config = traj.points[i].joint_values
    # Move robot the new pos
        speed = 1  # rad/s
        accel = 1.4  # rad/s^2
        nowait = False
        script += ur.move_j(config, acceleration, velocity,0.0, 0.01)
    script = c.concatenate_script(script)
    c.send_script(script.encode(), "192.168.10.12")

print("Finished.")






