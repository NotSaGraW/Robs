from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient()
sim = client.getObject('sim')

# Modo síncrono — nos sincronizamos con el step de simulación
sim.setStepping(True)
sim.startSimulation()

sensor = sim.getObject('/roba/ultrasonicSensor', {'index': 0})

for _ in range(10):
    sim.step()  # avanzar un step de simulación
    result = sim.readProximitySensor(sensor)
    print(result)

sim.stopSimulation()