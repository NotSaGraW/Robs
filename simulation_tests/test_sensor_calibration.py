"""
test_sensor_calibration.py — v2

Validates the corrected sensor geometry:
  SENSOR_FORWARD_OFFSET = 0.209m  (sensors [3],[4] are 5.1cm behind robot front)

Also tests:
  - lateral sensor inference from [3],[4]
  - lateral sensors [0],[7] detection range

SETUP: manually place p3dx_1 at (-1.75, 0.0) facing east (0°) in CoppeliaSim.

Run:
    python -m simulation_tests.test_sensor_calibration
"""

import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
from src import scene


PAYLOAD_HALF          = 0.25
SENSOR_FORWARD_OFFSET = 0.209   # calibrated: sensors [3],[4] offset from center
SEP_34                = 0.18    # lateral separation [3]↔[4]

SAMPLE_DISTS  = [0.60, 0.45, 0.30, 0.20]
ADVANCE_SPD   = 1.0
HEADING_THR   = 0.04
SETTLE_STEPS  = 60
MAX_STEPS     = 5000


def read_sensor(sim, handle):
    try:
        res = sim.readProximitySensor(handle)
        if res[0] > 0:
            return float(res[1])
    except Exception:
        pass
    return float('inf')


def infer_center(rx, ry, yaw, d3, d4,
                 offset=SENSOR_FORWARD_OFFSET,
                 sep=SEP_34,
                 phalf=PAYLOAD_HALF):
    """
    Infer payload center using calibrated sensor offset.
    sensor position = robot_center + offset (along heading)
    payload center  = sensor_pos + face_dist + phalf (along heading)
    """
    if d3 == float('inf') or d4 == float('inf'):
        return None, None, None, None

    face_dist  = (d3 + d4) / 2.0
    lat_off    = (d4 - d3) / 2.0
    face_angle = math.atan2(d4 - d3, sep)

    # Sensor position in world
    sx = rx + math.cos(yaw) * offset
    sy = ry + math.sin(yaw) * offset

    # Payload center
    cx = sx + math.cos(yaw) * (face_dist + phalf) - math.sin(yaw) * lat_off
    cy = sy + math.sin(yaw) * (face_dist + phalf) + math.cos(yaw) * lat_off

    return cx, cy, math.degrees(face_angle), lat_off


def main():
    print("=== SENSOR CALIBRATION v2 ===")
    print(f"SENSOR_FORWARD_OFFSET = {SENSOR_FORWARD_OFFSET}m\n")

    client    = RemoteAPIClient()
    sim       = client.getObject('sim')
    robot     = Robot(sim, '/p3dx_1', name='1')
    payload_h = sim.getObject('/payload')

    payload_pos = scene.get_position(sim, payload_h)
    rpos        = robot.get_position()
    yaw         = robot.get_yaw()

    print(f"Payload GT : ({payload_pos[0]:.4f}, {payload_pos[1]:.4f})")
    print(f"Robot pos  : ({rpos[0]:.4f}, {rpos[1]:.4f})  "
          f"yaw={math.degrees(yaw):.1f}°")

    if abs(rpos[1]) > 0.15:
        print(f"\nWARNING: robot y={rpos[1]:.3f} — should be ~0.0\n")

    sim.setStepping(True)
    sim.startSimulation()

    phase        = 'ALIGN'
    settle_count = 0
    sample_idx   = 0
    step         = 0
    results      = []

    def target_x(dist):
        # Robot center x when sensor face is 'dist' from payload face
        return -(PAYLOAD_HALF + SENSOR_FORWARD_OFFSET + dist)

    try:
        while step < MAX_STEPS and sample_idx < len(SAMPLE_DISTS):
            sim.step()
            rpos = robot.get_position()
            yaw  = robot.get_yaw()

            if phase == 'ALIGN':
                err = math.atan2(math.sin(-yaw), math.cos(-yaw))
                if abs(err) < HEADING_THR:
                    robot.stop()
                    phase = 'SETTLE'
                    settle_count = 0
                    print(f"  Aligned: yaw={math.degrees(yaw):.2f}°")
                else:
                    w = max(-1.2, min(1.2, 4.0 * err))
                    robot.set_velocity(-w, w)

            elif phase == 'SETTLE':
                robot.stop()
                settle_count += 1
                if settle_count >= SETTLE_STEPS:
                    phase = 'ADVANCE'

            elif phase == 'ADVANCE':
                tgt_x = target_x(SAMPLE_DISTS[sample_idx])
                if rpos[0] >= tgt_x:
                    robot.stop()
                    phase        = 'SETTLE_SAMPLE'
                    settle_count = 0
                else:
                    y_err = -rpos[1]
                    hdg   = max(-0.25, min(0.25,
                                math.atan2(y_err * 4.0, 1.0)))
                    err   = math.atan2(math.sin(hdg-yaw),
                                       math.cos(hdg-yaw))
                    w     = max(-1.5, min(1.5, 4.0*err))
                    robot.set_velocity(ADVANCE_SPD-w, ADVANCE_SPD+w)

            elif phase == 'SETTLE_SAMPLE':
                robot.stop()
                settle_count += 1
                if settle_count >= SETTLE_STEPS:
                    phase = 'SAMPLE'

            elif phase == 'SAMPLE':
                d = {i: read_sensor(sim, robot.sensors[i])
                     for i in range(9) if i < len(robot.sensors)}

                d3 = d.get(3, float('inf'))
                d4 = d.get(4, float('inf'))
                d0 = d.get(0, float('inf'))
                d7 = d.get(7, float('inf'))
                d8 = d.get(8, float('inf'))

                pay_pos = scene.get_position(sim, payload_h)
                pay_yaw = math.degrees(
                    sim.getObjectOrientation(payload_h, -1)[2])

                # GT: distance from sensor face to payload face
                sensor_x     = rpos[0] + SENSOR_FORWARD_OFFSET * math.cos(yaw)
                payload_face = pay_pos[0] - PAYLOAD_HALF
                gt_face_dist = payload_face - sensor_x   # positive when sensor behind face

                icx, icy, face_deg, lat_off = infer_center(
                    rpos[0], rpos[1], yaw, d3, d4)

                avg34 = (d3+d4)/2 if d3 != float('inf') and \
                                     d4 != float('inf') else float('nan')
                delta = avg34 - gt_face_dist \
                        if not math.isnan(avg34) else float('nan')
                err_m = math.sqrt((icx-pay_pos[0])**2+(icy-pay_pos[1])**2) \
                        if icx is not None else float('nan')

                td = SAMPLE_DISTS[sample_idx]
                print(f"─── Sample {sample_idx+1}  target={td}m "
                      f"──────────────────────────────────")
                print(f"  Robot       : ({rpos[0]:.4f}, {rpos[1]:.4f})  "
                      f"yaw={math.degrees(yaw):.2f}°")
                print(f"  Sensor pos  : ({sensor_x:.4f}, {rpos[1]:.4f})")
                print(f"  Payload GT  : ({pay_pos[0]:.4f}, {pay_pos[1]:.4f})")
                print(f"  GT face dist: {gt_face_dist:.4f}m")
                print(f"  [0]={d0:.4f}  [3]={d3:.4f}  [4]={d4:.4f}  "
                      f"[7]={d7:.4f}  [8]={d8:.4f}")
                print(f"  avg(d3,d4)  : {avg34:.4f}  Δ={delta:+.4f}m  "
                      f"(0=perfect)")
                print(f"  d4-d3       : "
                      f"{(d4-d3) if d4!=float('inf') else float('nan'):+.4f}  "
                      f"(0=centered)")
                if icx is not None:
                    print(f"  Inferred ctr: ({icx:.4f}, {icy:.4f})  "
                          f"err={err_m:.4f}m")
                    print(f"  Face angle  : {face_deg:.2f}°")
                else:
                    print(f"  Inferred    : CANNOT INFER")
                print()

                results.append({
                    'td': td, 'gt_fd': gt_face_dist,
                    'd3': d3, 'd4': d4, 'avg34': avg34, 'delta': delta,
                    'd0': d0, 'd7': d7, 'd8': d8,
                    'err_m': err_m, 'face_deg': face_deg,
                    'robot_y': rpos[1], 'yaw': math.degrees(yaw),
                })

                sample_idx  += 1
                phase        = 'ADVANCE'
                settle_count = 0

            step += 1

        print(f"\n{'='*68}")
        print("SUMMARY")
        print(f"{'dist':>6} {'gt_fd':>7} {'avg34':>7} {'Δ':>7} "
              f"{'d4-d3':>7} {'err_m':>8} {'face°':>8}")
        print(f"{'-'*68}")
        for r in results:
            d3v = r['d3'] if r['d3'] != float('inf') else float('nan')
            d4v = r['d4'] if r['d4'] != float('inf') else float('nan')
            fd  = r['face_deg'] if r['face_deg'] is not None else float('nan')
            print(f"{r['td']:6.2f}  {r['gt_fd']:7.4f}  {r['avg34']:7.4f}  "
                  f"{r['delta']:+7.4f}  {(d4v-d3v):+7.4f}  "
                  f"{r['err_m']:8.4f}  {fd:8.2f}°")
        print(f"{'='*68}")
        print(f"\nIf Δ ≈ 0 and err_m ≈ 0 → SENSOR_FORWARD_OFFSET={SENSOR_FORWARD_OFFSET}m confirmed.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
        robot.stop()
    finally:
        sim.stopSimulation()
        print("Simulation stopped.")


if __name__ == '__main__':
    main()