"""
main.py
Punto de entrada del sistema multi-robot cooperativo.

Uso:
    python -m src.main

Estructura del proyecto:
    src/
        main.py      — bucle principal, logs, timing
        robot.py     — clase Robot con sensores configurados
        drone.py     — clase Drone con patrulla y mapeo de corredor
        scene.py     — geometría, vectores, condición de éxito
        strategy.py  — máquina de estados con mapa de ocupación
        grid.py      — mapa de ocupación 2D compartido

Agentes:
    /qua   — Quadcopter: patrulla, detecta box, mapea corredor box→target
    /roba  — Pioneer P3DX: exploración sector izquierdo, pusher A
    /robo  — Pioneer P3DX: exploración sector derecho, pusher B
"""

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from .robot    import Robot
from .drone    import Drone
from .scene    import get_position, box_reached_target, dist2d
from .strategy import Strategy, Phase


STEP_INTERVAL = 0.05
MAX_DURATION  = 300.0
LOG_EVERY     = 10


def main():
    print("Conectando con CoppeliaSim...")
    client = RemoteAPIClient()
    sim    = client.getObject('sim')
    print("Conexión establecida.\n")

    robot_a = Robot(sim, '/roba', name='A')
    robot_b = Robot(sim, '/robo', name='B')
    drone   = Drone(sim, '/qua')

    box_h    = sim.getObject('/box')
    target_h = sim.getObject('/target')

    strategy = Strategy(robot_a, robot_b, drone, box_h, target_h, sim)

    print("Agentes inicializados:")
    print(f"  Robot A  : {robot_a.get_position()[:2]}")
    print(f"  Robot B  : {robot_b.get_position()[:2]}")
    print(f"  Drone    : {drone.get_position()[:2]}")
    print(f"  Sensores A: {len(robot_a.sensors)}/16")
    print(f"  Sensores B: {len(robot_b.sensors)}/16")
    print(f"  Target   : {get_position(sim, target_h)[:2]}")
    print(f"  Mapa     : {strategy.map.rows}x{strategy.map.cols} "
          f"({strategy.map.resolution}m/celda)\n")

    sim.startSimulation()
    print("Simulación iniciada.\n")

    t_start     = time.time()
    t_detection = None
    cycle       = 0

    # Notificar al drone el target cuando se detecte la caja
    corridor_set = False

    try:
        while True:
            t_cycle = time.time()

            phase = strategy.step()

            # Primera detección
            if t_detection is None and strategy.box_known_pos is not None:
                t_detection = time.time() - t_start
                print(f"\n>>> CAJA DETECTADA en t={t_detection:.2f}s")
                print(f"    Posición: {strategy.box_known_pos[:2]}\n")

            # Configurar corredor del drone una vez conocida box y target
            if (not corridor_set and
                    strategy.box_known_pos is not None):
                target_pos = get_position(sim, target_h)
                drone.set_corridor(strategy.box_known_pos, target_pos)
                corridor_set = True

            if cycle % LOG_EVERY == 0:
                elapsed    = time.time() - t_start
                target_pos = get_position(sim, target_h)
                print(f"t={elapsed:6.1f}s  "
                      f"{strategy.status(target_pos)}  "
                      f"| {drone.status()}")

            if phase == Phase.SUCCESS:
                elapsed    = time.time() - t_start
                box_pos    = get_position(sim, box_h)
                target_pos = get_position(sim, target_h)
                print(f"\n{'='*68}")
                print(f"  OBJETIVO ALCANZADO")
                print(f"  Tiempo total:      {elapsed:.2f} s")
                if t_detection:
                    print(f"  Tiempo detección:  {t_detection:.2f} s")
                    print(f"  Tiempo empuje:     {elapsed-t_detection:.2f} s")
                print(f"  Distancia final:   {dist2d(box_pos,target_pos):.4f} m")
                print(f"  Posición caja:     ({box_pos[0]:.3f},{box_pos[1]:.3f})")
                print(f"  Target:            ({target_pos[0]:.3f},{target_pos[1]:.3f})")
                print(f"  {strategy.map.stats()}")
                print(f"{'='*68}\n")
                break

            elapsed = time.time() - t_start
            if elapsed > MAX_DURATION:
                print(f"\nTIMEOUT ({MAX_DURATION}s).")
                robot_a.stop()
                robot_b.stop()
                break

            sleep = max(0.0, STEP_INTERVAL - (time.time() - t_cycle))
            time.sleep(sleep)
            cycle += 1

    except KeyboardInterrupt:
        print("\nInterrumpido.")
        try:
            robot_a.stop()
            robot_b.stop()
        except Exception:
            pass

    finally:
        try:
            sim.stopSimulation()
        except Exception:
            pass
        print("Simulación detenida.")


if __name__ == '__main__':
    main()