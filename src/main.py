"""
main.py
Entry point for the multi-robot cooperative system.

Usage:
    python -m src.main

Project structure:
    src/
        main.py      — main loop, logs, timing
        robot.py     — Robot class with configured sensors
        drone.py     — Drone class with patrol and corridor mapping
        scene.py     — geometry, vectors, success condition
        strategy.py  — state machine with occupancy map
        grid.py      — shared 2D occupancy map

Agents:
    /drone   — Quadcopter: patrols, detects payload, maps corridor to rally point
    /p3dx_1  — Pioneer P3DX: exploration, pusher 1
    /p3dx_2  — Pioneer P3DX: exploration, pusher 2
"""

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from .robot    import Robot
from .drone    import Drone
from .scene    import get_position, payload_reached_rally_point, dist2d
from .strategy import Strategy, Phase


STEP_INTERVAL = 0.05
MAX_DURATION  = 300.0
LOG_EVERY     = 10


def main():
    print("Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim    = client.getObject('sim')
    print("Connection established.\n")

    p3dx_1 = Robot(sim, '/p3dx_1', name='1')
    p3dx_2 = Robot(sim, '/p3dx_2', name='2')
    drone  = Drone(sim, '/drone')

    payload_h     = sim.getObject('/payload')
    rally_point_h = sim.getObject('/rally_point')

    strategy = Strategy(p3dx_1, p3dx_2, drone, payload_h, rally_point_h, sim)

    print("Agents initialized:")
    print(f"  P3DX 1     : {p3dx_1.get_position()[:2]}")
    print(f"  P3DX 2     : {p3dx_2.get_position()[:2]}")
    print(f"  Drone      : {drone.get_position()[:2]}")
    print(f"  Sensors 1  : {len(p3dx_1.sensors)}/16")
    print(f"  Sensors 2  : {len(p3dx_2.sensors)}/16")
    print(f"  Rally point: {get_position(sim, rally_point_h)[:2]}")
    print(f"  Map        : {strategy.map.rows}x{strategy.map.cols} "
          f"({strategy.map.resolution}m/cell)\n")

    sim.startSimulation()
    print("Simulation started.\n")

    t_start      = time.time()
    t_detection  = None
    cycle        = 0
    corridor_set = False

    try:
        while True:
            t_cycle = time.time()

            phase = strategy.step()

            # First detection
            if t_detection is None and strategy.payload_known_pos is not None:
                t_detection = time.time() - t_start
                print(f"\n>>> PAYLOAD DETECTED at t={t_detection:.2f}s")
                print(f"    Position: {strategy.payload_known_pos[:2]}\n")

            # Set drone corridor once payload is known
            if (not corridor_set and
                    strategy.payload_known_pos is not None):
                rally_pos = get_position(sim, rally_point_h)
                drone.set_corridor(strategy.payload_known_pos, rally_pos)
                corridor_set = True

            if cycle % LOG_EVERY == 0:
                elapsed   = time.time() - t_start
                rally_pos = get_position(sim, rally_point_h)
                print(f"t={elapsed:6.1f}s  "
                      f"{strategy.status(rally_pos)}  "
                      f"| {drone.status()}")

            if phase == Phase.SUCCESS:
                elapsed     = time.time() - t_start
                payload_pos = get_position(sim, payload_h)
                rally_pos   = get_position(sim, rally_point_h)
                print(f"\n{'='*68}")
                print(f"  MISSION COMPLETE")
                print(f"  Total time:       {elapsed:.2f} s")
                if t_detection:
                    print(f"  Detection time:   {t_detection:.2f} s")
                    print(f"  Push time:        {elapsed - t_detection:.2f} s")
                print(f"  Final distance:   {dist2d(payload_pos, rally_pos):.4f} m")
                print(f"  Payload position: ({payload_pos[0]:.3f},{payload_pos[1]:.3f})")
                print(f"  Rally point:      ({rally_pos[0]:.3f},{rally_pos[1]:.3f})")
                print(f"  {strategy.map.stats()}")
                print(f"{'='*68}\n")
                break

            elapsed = time.time() - t_start
            if elapsed > MAX_DURATION:
                print(f"\nTIMEOUT ({MAX_DURATION}s).")
                p3dx_1.stop()
                p3dx_2.stop()
                break

            sleep = max(0.0, STEP_INTERVAL - (time.time() - t_cycle))
            time.sleep(sleep)
            cycle += 1

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