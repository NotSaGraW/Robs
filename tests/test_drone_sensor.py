# tests/test_drone_sensor.py
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.startSimulation()
time.sleep(0.5)

qua = sim.getObject('/qua')
objects = sim.getObjectsInTree(qua, 0, 0)
for obj in objects:
    alias = sim.getObjectAlias(obj, 2)
    print(f"  {alias}")

sim.stopSimulation()