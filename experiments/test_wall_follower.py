# src/tests/test_wall_follower.py
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
import time
import math

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.startSimulation()

robot = Robot(sim, '/roba', name='A')

kp_rot = 200
kd_rot = 10
ki_rot = 30
sum_rot_error = 0
last_rot_error = 0
kp_trans = 90
v0 = 120 * math.pi / 180
wall_distance = 0.25
ts = 0.05

t_start = time.time()
while time.time() - t_start < 30:
    readings = robot.read_sensors()
    distances = [d if detected else 1.0
                 for detected, d in readings]

    front_dist       = distances[4]
    right_front_dist = distances[7]
    right_back_dist  = distances[8]

    rot_error     = right_front_dist - right_back_dist
    sum_rot_error += rot_error
    pid_rot        = (kp_rot * rot_error +
                      ki_rot * sum_rot_error * ts +
                      kd_rot * (rot_error - last_rot_error) / ts)
    last_rot_error = rot_error

    trans_error = right_front_dist - wall_distance
    pid_trans   = kp_trans * trans_error

    vLeft  = v0 + pid_rot + pid_trans
    vRight = v0 - pid_rot - pid_trans
    robot.set_velocity(vLeft, vRight)

    if front_dist < 0.4:
        robot.set_velocity(-v0, v0)
        time.sleep(0.2)

    print(f"front:{front_dist:.2f} "
          f"r_front:{right_front_dist:.2f} "
          f"r_back:{right_back_dist:.2f}")
    time.sleep(ts)

robot.stop()
sim.stopSimulation()