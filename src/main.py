"""
main.py
Punto de entrada del sistema multi-robot cooperativo.

Uso:
    python -m src.main

Estructura del proyecto:
    src/
        main.py      — bucle principal, logs, timing
        planner.py   — ContactPlanner: NNLS force decomposition, 8-direction frames
        robot.py     — clase Robot (Pioneer P3DX): sensores, motores, navegación
        team.py      — Team: registro de agentes, mapa compartido
        strategy.py  — máquina de estados de misión (exploración → empuje)
        scene.py     — geometría, vectores de empuje, condición de éxito
        grid.py      — mapa de ocupación 2D compartido

Agentes:
    /p3dx_1  — Pioneer P3DX: pusher 1
    /p3dx_2  — Pioneer P3DX: pusher 2
"""

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from .robot    import Robot
from .team     import Team
from .scene    import get_position, payload_reached_rally_point, dist2d
from .strategy import Strategy, Phase


MAX_STEPS = 6000
LOG_EVERY = 20


def main():
    print("Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim    = client.getObject('sim')
    print("Connection established.\n")

    p3dx_1 = Robot(sim, '/p3dx_1', name='1')
    p3dx_2 = Robot(sim, '/p3dx_2', name='2')

    payload_h     = sim.getObject('/payload')
    rally_point_h = sim.getObject('/rally_point')

    team = Team(
        sim           = sim,
        agents        = {'p3dx_1': p3dx_1, 'p3dx_2': p3dx_2},
        rally_point_h = rally_point_h,
        payload_h     = payload_h,
    )

    strategy = Strategy(team, sim)

    print("Agents initialized:")
    print(f"  P3DX 1     : {p3dx_1.get_position()[:2]}")
    print(f"  P3DX 2     : {p3dx_2.get_position()[:2]}")
    print(f"  Sensors 1  : {len(p3dx_1.sensors)}/16")
    print(f"  Sensors 2  : {len(p3dx_2.sensors)}/16")
    print(f"  Rally point: {get_position(sim, rally_point_h)[:2]}")
    print(f"  Map        : {team.map.rows}x{team.map.cols} "
          f"({team.map.resolution}m/cell)\n")

    sim.setStepping(True)
    sim.startSimulation()
    print("Simulation started (synchronous stepping).\n")

    t_start     = time.time()
    t_detection = None
    step        = 0

    try:
        while True:
            sim.step()

            phase = strategy.step()

            if t_detection is None and strategy.payload_known_pos is not None:
                t_detection = time.time() - t_start
                print(f"\n>>> PAYLOAD DETECTED at step={step} t={t_detection:.2f}s")
                print(f"    Position: {strategy.payload_known_pos[:2]}\n")

            if step % LOG_EVERY == 0:
                elapsed   = time.time() - t_start
                rally_pos = get_position(sim, rally_point_h)
                print(f"step={step:5d} t={elapsed:6.1f}s  "
                      f"{strategy.status(rally_pos)}")

            if phase == Phase.SUCCESS:
                elapsed     = time.time() - t_start
                payload_pos = get_position(sim, payload_h)
                rally_pos   = get_position(sim, rally_point_h)
                print(f"\n{'='*68}")
                print(f"  MISSION COMPLETE")
                print(f"  Total steps:      {step}")
                print(f"  Total time:       {elapsed:.2f} s")
                if t_detection:
                    print(f"  Detection time:   {t_detection:.2f} s")
                    print(f"  Push time:        {elapsed - t_detection:.2f} s")
                print(f"  Final distance:   {dist2d(payload_pos, rally_pos):.4f} m")
                print(f"  Payload position: ({payload_pos[0]:.3f},{payload_pos[1]:.3f})")
                print(f"  Rally point:      ({rally_pos[0]:.3f},{rally_pos[1]:.3f})")
                print(f"  {team.map.stats()}")
                print(f"{'='*68}\n")
                break

            step += 1
            if step >= MAX_STEPS:
                print(f"\nTIMEOUT ({MAX_STEPS} steps).")
                p3dx_1.stop()
                p3dx_2.stop()
                break

    except KeyboardInterrupt:
        print("\nInterrupted.")
        try:
            p3dx_1.stop()
            p3dx_2.stop()
        except Exception:
            pass

    finally:
        try:
            sim.stopSimulation()
        except Exception:
            pass
        print("Simulation stopped.")


if __name__ == '__main__':
    main()