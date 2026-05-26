# simulation_tests/test_sensor_angles.py
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.startSimulation()

robot = sim.getObject('/p3dx_1')
for i in range(16):
    h = sim.getObject('/p3dx_1/ultrasonicSensor', {'index': i})
    m = sim.getObjectMatrix(h, robot)
    fx = m[2]
    fy = m[6]
    angle = math.degrees(math.atan2(fy, fx))
    print(f"[{i:>2}] {angle:7.2f}°")

sim.stopSimulation()