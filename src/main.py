"""
main.py
Punto de entrada. Conecta con CoppeliaSim, instancia robots y ejecuta estrategia.

Uso:
    python -m src.robo.main

Estructura del proyecto:
    src/
        main.py
        robot.py
        drone.py
        scene.py
        strategy.py

Agentes:
  /qua   — Quadcopter, patrulla y detecta la caja
  /roba  — Pioneer P3DX, pusher izquierdo
  /robo  — Pioneer P3DX, pusher derecho

Uso:
    python -m src.main
"""

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from .robot import Robot
from .drone import Drone
from .scene import get_position, box_reached_target, dist2d
from .strategy import Strategy, Phase


# ------------------------------------------------------------------
# Configuración
# ------------------------------------------------------------------

STEP_INTERVAL = 0.05    # segundos entre ciclos (20 Hz)
MAX_DURATION  = 300.0   # timeout de seguridad (5 min)
LOG_EVERY     = 10      # imprimir estado cada N ciclos


def main():
    # ------------------------------------------------------------------
    # Conexión
    # ------------------------------------------------------------------
    print("Conectando con CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    print("Conexión establecida.\n")

    # ------------------------------------------------------------------
    # Instanciar agentes
    # ------------------------------------------------------------------
    robot_a = Robot(sim, '/roba', name='A')
    robot_b = Robot(sim, '/robo', name='B')
    drone   = Drone(sim, '/qua')

    box_handle    = sim.getObject('/box')
    target_handle = sim.getObject('/target')

    strategy = Strategy(robot_a, robot_b, drone, box_handle, target_handle, sim)

    print("Agentes inicializados:")
    print(f"  Robot A : {robot_a.get_position()[:2]}")
    print(f"  Robot B : {robot_b.get_position()[:2]}")
    print(f"  Drone   : {drone.get_position()[:2]}")
    print(f"  Caja    : posición desconocida hasta detección")
    print(f"  Target  : {get_position(sim, target_handle)[:2]}")
    print()

    # ------------------------------------------------------------------
    # Simulación
    # ------------------------------------------------------------------
    sim.startSimulation()
    print("Simulación iniciada.\n")

    t_start = time.time()
    t_detection = None
    cycle = 0

    try:
        while True:
            t_cycle = time.time()

            phase = strategy.step()

            # Registrar tiempo de detección
            if t_detection is None and strategy.box_known_pos is not None:
                t_detection = time.time() - t_start
                print(f"\n>>> CAJA DETECTADA por drone en t={t_detection:.2f}s")
                print(f"    Posición comunicada: {strategy.box_known_pos[:2]}\n")

            # Log periódico
            if cycle % LOG_EVERY == 0:
                elapsed = time.time() - t_start
                target_pos = get_position(sim, target_handle)
                print(f"t={elapsed:6.1f}s  {strategy.status(target_pos)}  "
                      f"| {drone.status()}")

            # Éxito
            if phase == Phase.SUCCESS:
                elapsed = time.time() - t_start
                box_pos    = get_position(sim, box_handle)
                target_pos = get_position(sim, target_handle)
                final_dist = dist2d(box_pos, target_pos)

                print(f"\n{'='*65}")
                print(f"  OBJETIVO ALCANZADO")
                print(f"  Tiempo total:          {elapsed:.2f} s")
                if t_detection:
                    print(f"  Tiempo hasta detección:{t_detection:.2f} s")
                    print(f"  Tiempo de empuje:      {elapsed - t_detection:.2f} s")
                print(f"  Distancia final:       {final_dist:.4f} m")
                print(f"  Posición caja:         ({box_pos[0]:.3f}, {box_pos[1]:.3f})")
                print(f"  Target:                ({target_pos[0]:.3f}, {target_pos[1]:.3f})")
                print(f"{'='*65}\n")
                break

            # Timeout
            elapsed = time.time() - t_start
            if elapsed > MAX_DURATION:
                print(f"\nTIMEOUT ({MAX_DURATION}s). Deteniendo.")
                robot_a.stop()
                robot_b.stop()
                break

            # Esperar siguiente ciclo
            sleep = max(0.0, STEP_INTERVAL - (time.time() - t_cycle))
            time.sleep(sleep)
            cycle += 1

    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
        robot_a.stop()
        robot_b.stop()

    finally:
        sim.stopSimulation()
        print("Simulación detenida.")


if __name__ == '__main__':
    main()