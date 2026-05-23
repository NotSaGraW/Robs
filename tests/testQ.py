from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(True)
sim.startSimulation()

# Primero exploramos qué objetos tiene el quadcopter
quad = sim.getObject('/Quadcopter')  # ajusta el nombre si es distinto
print(f"Quadcopter handle: {quad}")

# Intentar encontrar sensores
for name in ['sensingNose', 'sensor', 'Sensor', 'proxSensor', 
             'ultrasonicSensor', 'visionSensor']:
    try:
        h = sim.getObject(f'/Quadcopter/{name}')
        print(f"Encontrado: {name} = {h}")
    except:
        pass

# Listar todos los objetos de la escena para ver qué hay
objects = sim.getObjectsInTree(quad, 0, 0)  # 0 = todos los tipos
for obj in objects:
    alias = sim.getObjectAlias(obj, 2)
    print(f"  child: {alias}")

for _ in range(5):
    sim.step()

sim.stopSimulation()