"""
planner.py — ContactPlanner
Central force-decomposition planner for two-robot cooperative push.

Algorithm (each planning call):
  1. Compute F_des = normalise(payload → rally).
  2. Enumerate all unordered face pairs from 8 directions (N S E W NE NW SE SW).
  3. For each pair, evaluate both robot-to-face orderings:
       a. Reject if approach separation < MIN_APPROACH_SEP  (collision avoidance).
       b. Reject if either approach is on the rally side of the payload
          (robot would push payload away from goal).
       c. Solve 2-variable NNLS: min ||f0*n0 + f1*n1 - F_des||²  s.t. f0,f1 ≥ 0.
       d. Score the assignment (lower is better):
            score = residual
                  + PROGRESS_WEIGHT  * progress_penalty   # directional alignment
                  + ALIGNMENT_WEIGHT * alignment_penalty   # per-robot alignment
                  + BALANCE_WEIGHT   * balance             # |f0 - f1|
                  + TORQUE_WEIGHT    * torque              # net rotation proxy
                  + NAV_WEIGHT       * nav_cost            # normalised travel distance
                  - lock_bonus                             # sticky assignment reward
  4. Select minimum-score feasible assignment.
  5. Generate collision-aware waypoints for each robot via _safe_waypoints.
  6. Lock assignment (sticky) — only re-scored if face changes or stall reset.

Scoring rationale:
  NAV_WEIGHT (0.30) is intentionally higher than the original (0.08) to prevent
  the planner from choosing geometrically optimal but navigationally infeasible
  assignments where one robot must cross the full scene (~3 m) while the other
  is already in position.  nav_cost is normalised by NAV_NORM (3.0 m) so the
  weight stays in the same scale as the other penalty terms.
"""

import math
import itertools


PAYLOAD_HALF       = 0.25
ROBOT_HALF_W       = 0.19   # Pioneer P3DX body half-width (0.381m / 2, rounded up)
PUSH_DIST          = 0.489
NAV_CLEARANCE      = PAYLOAD_HALF + ROBOT_HALF_W + 0.01  # Minkowski sum: 0.45 m
ROBOT_CLEARANCE    = 0.50
MIN_APPROACH_SEP   = 0.52   # m — min distance between approach positions
                             # Pioneer width=0.415m, +0.10m margin

MIN_FORCE          = 0.05
SOLO_PENALTY       = 0.20   # added to single-robot scores to prefer cooperation when both are reachable

NAV_WEIGHT         = 0.30   # raised from 0.08 to penalise long approach routes
NAV_NORM           = 3.0    # typical scene span in metres — normalises nav_cost to [0,1]
BALANCE_WEIGHT     = 0.20
PROGRESS_WEIGHT    = 0.50
TORQUE_WEIGHT      = 0.10
ALIGNMENT_WEIGHT   = 0.30

# Hysteresis: new assignment must beat current by more than PLAN_EPSILON to switch.
# lock_bonus (0.2 total) provides a first layer of stickiness.
# PLAN_EPSILON adds an explicit second layer so that marginal geometry changes
# (plan_margin < PLAN_EPSILON) never cause a face flip.
PLAN_EPSILON       = 0.05

_R2 = 0.7071  # 1/sqrt(2)
PUSH_CONFIGS = {
    'N':  {'approach_offset': ( 0,    -1   )},
    'S':  {'approach_offset': ( 0,    +1   )},
    'E':  {'approach_offset': (-1,     0   )},
    'W':  {'approach_offset': (+1,     0   )},
    'NE': {'approach_offset': (-_R2,  -_R2 )},
    'NW': {'approach_offset': (+_R2,  -_R2 )},
    'SE': {'approach_offset': (-_R2,  +_R2 )},
    'SW': {'approach_offset': (+_R2,  +_R2 )},
}


# ── Geometry helpers ──────────────────────────────────────────────

def _norm(dx, dy):
    n = math.sqrt(dx*dx + dy*dy) + 1e-9
    return dx/n, dy/n

def _dot(a, b):
    return a[0]*b[0] + a[1]*b[1]

def _cross2d(r, n):
    return r[0]*n[1] - r[1]*n[0]

def _dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def _seg_closest(p1, p2, q):
    dx, dy = p2[0]-p1[0], p2[1]-p1[1]
    L2 = dx*dx + dy*dy
    if L2 < 1e-9:
        return _dist(p1, q)
    t = max(0.0, min(1.0, ((q[0]-p1[0])*dx + (q[1]-p1[1])*dy) / L2))
    return math.sqrt((p1[0]+t*dx-q[0])**2 + (p1[1]+t*dy-q[1])**2)


def _approach_pos(payload_pos, face_key):
    px, py = payload_pos
    ao = PUSH_CONFIGS[face_key]['approach_offset']
    return (px + ao[0]*PUSH_DIST, py + ao[1]*PUSH_DIST)


def _push_dir_for_face(payload_pos, face_key):
    """Dynamic push direction: approach → payload centre (normalised)."""
    px, py = payload_pos
    app = _approach_pos(payload_pos, face_key)
    return _norm(px - app[0], py - app[1])


def _approach_on_rally_side(approach_pos, payload_pos, F_des, threshold=0.1):
    """
    True if the approach is on the rally side of the payload
    (robot would push payload away from rally).
    """
    px, py = payload_pos
    return _dot((approach_pos[0]-px, approach_pos[1]-py), F_des) > threshold


# ── NNLS solver (exact, 2 variables) ─────────────────────────────

def _nnls_2d(n1, n2, F):
    """min ||f1*n1 + f2*n2 - F||²  s.t. f1,f2 >= 0  (closed-form)."""
    def res(f1, f2):
        ex = f1*n1[0]+f2*n2[0]-F[0]
        ey = f1*n1[1]+f2*n2[1]-F[1]
        return ex*ex+ey*ey

    a, b, c = _dot(n1,n1), _dot(n1,n2), _dot(n2,n2)
    d, e    = _dot(n1,F),  _dot(n2,F)
    cands   = [(0.0, 0.0)]
    det = a*c - b*b
    if abs(det) > 1e-8:
        f1u = (c*d-b*e)/det
        f2u = (a*e-b*d)/det
        if f1u >= 0 and f2u >= 0:
            cands.append((f1u, f2u))
    if c > 1e-9: cands.append((0.0, max(0.0, e/c)))
    if a > 1e-9: cands.append((max(0.0, d/a), 0.0))
    return min(cands, key=lambda p: res(*p))


# ── Assignment scorer ─────────────────────────────────────────────

def _score_assignment(robot0_pos, robot1_pos,
                      key_r0, key_r1,
                      payload_pos, F_des,
                      lock_r0, lock_r1):
    """
    Score assignment (R0→key_r0, R1→key_r1).
    Returns (score, nd0, nd1, f0, f1, app0, app1) or None if infeasible.
    """
    px, py = payload_pos

    app0 = _approach_pos((px, py), key_r0)
    app1 = _approach_pos((px, py), key_r1)

    # ── Physical separation: robots must not collide at approach positions ──
    if _dist(app0, app1) < MIN_APPROACH_SEP:
        return None

    # ── Geometric guardrail: approach must not be on the rally side ──────
    if _approach_on_rally_side(app0, (px, py), F_des):
        return None
    if _approach_on_rally_side(app1, (px, py), F_des):
        return None

    nd0 = _push_dir_for_face((px, py), key_r0)
    nd1 = _push_dir_for_face((px, py), key_r1)

    f0, f1 = _nnls_2d(nd0, nd1, F_des)

    if max(f0, f1) < MIN_FORCE:
        return None

    # ── Force residual ────────────────────────────────────────────
    rx = f0*nd0[0] + f1*nd1[0]
    ry = f0*nd0[1] + f1*nd1[1]
    rn = math.sqrt(rx*rx + ry*ry) + 1e-9
    residual = math.sqrt((rx-F_des[0])**2 + (ry-F_des[1])**2)

    progress_penalty  = 1.0 - ((rx/rn)*F_des[0] + (ry/rn)*F_des[1])

    align0 = max(_dot(nd0, F_des), 0.0)
    align1 = max(_dot(nd1, F_des), 0.0)
    alignment_penalty = (1.0 - align0) + (1.0 - align1)

    balance = abs(f0 - f1)

    r0 = (app0[0]-px, app0[1]-py)
    r1 = (app1[0]-px, app1[1]-py)
    torque = abs(f0*_cross2d(r0, nd0) + f1*_cross2d(r1, nd1))

    nav_cost = (_dist(robot0_pos, app0) + _dist(robot1_pos, app1)) / NAV_NORM

    lock_bonus = (0.1 if lock_r0 == key_r0 else 0.0) + \
                 (0.1 if lock_r1 == key_r1 else 0.0)

    score = (residual
             + PROGRESS_WEIGHT   * progress_penalty
             + ALIGNMENT_WEIGHT  * alignment_penalty
             + BALANCE_WEIGHT    * balance
             + TORQUE_WEIGHT     * torque
             + NAV_WEIGHT        * nav_cost
             - lock_bonus)

    return score, nd0, nd1, f0, f1, app0, app1


# ── Waypoint generation ───────────────────────────────────────────

def _safe_waypoints(start, end, payload, nav_clr,
                    robot_obstacles=None, robot_clr=None):
    obstacles = list(robot_obstacles) if robot_obstacles else []
    r_clr = robot_clr if robot_clr is not None else nav_clr

    def seg_ok(p1, p2):
        if _seg_closest(p1, p2, payload) < nav_clr:
            return False
        for ob in obstacles:
            if _seg_closest(p1, p2, ob) < r_clr:
                return False
        return True

    if seg_ok(start, end):
        return []

    px, py = payload

    # Pass 1 — single intermediate waypoint sampled on 8 directions around
    # the payload at increasing radii.  Pick the shortest valid path so the
    # robot takes the most direct arc rather than a long L-shaped detour.
    best_wp   = None
    best_len  = float('inf')
    for scale in [1.2, 1.5, 2.0, 3.0]:
        r = nav_clr * scale
        for deg in range(0, 360, 45):
            rad = math.radians(deg)
            wp = (px + r * math.cos(rad), py + r * math.sin(rad))
            if seg_ok(start, wp) and seg_ok(wp, end):
                length = _dist(start, wp) + _dist(wp, end)
                if length < best_len:
                    best_len, best_wp = length, wp
    if best_wp is not None:
        return [best_wp]

    # Pass 2 — fallback: two-waypoint L-shaped column path.
    # Reliable when start and end are on the same side of the payload and a
    # single-waypoint arc cannot clear both segments simultaneously.
    sy = start[1]
    ey = end[1]
    if obstacles:
        obs_x_mean = sum(o[0] for o in obstacles) / len(obstacles)
        x_signs = [+1, -1] if obs_x_mean <= px else [-1, +1]
    else:
        x_signs = [+1, -1]

    for x_sign in x_signs:
        for x_offset in [2.5, 3.5, 5.0]:
            col_x = px + x_sign * nav_clr * x_offset
            wp1, wp2 = (col_x, sy), (col_x, ey)
            if seg_ok(start, wp1) and seg_ok(wp1, wp2) and seg_ok(wp2, end):
                return [wp1, wp2]

    return []


# ── Contact Planner ───────────────────────────────────────────────

class ContactPlanner:
    """
    Force-decomposition planner with 8-direction frames and collision filtering.
    Adjacent faces (45° apart) are automatically rejected by MIN_APPROACH_SEP.
    """

    def __init__(self):
        self._locked          = {}
        self.last_best_score  = 0.0   # score of the winning assignment (after lock_bonus)
        self.last_margin      = 0.0   # score gap: 2nd-best minus best (stability proxy)
        self.last_scores      = []    # [(score, key_r0, key_r1), ...] top-3 feasible
        self.last_epsilon_held = False  # True when PLAN_EPSILON prevented a face switch

    def reset(self, robot_idx=None, keep_locked=False):
        if robot_idx is None:
            if not keep_locked:
                self._locked.clear()
        else:
            self._locked.pop(robot_idx, None)

    def plan(self, robots_pos, payload_pos, rally_pos):
        px, py   = payload_pos[0], payload_pos[1]
        F_des    = _norm(rally_pos[0]-px, rally_pos[1]-py)
        all_keys = list(PUSH_CONFIGS.keys())

        lock_r0 = self._locked.get(0)
        lock_r1 = self._locked.get(1)
        r0_pos  = robots_pos[0]
        r1_pos  = robots_pos[1]

        candidates     = []             # (score, key_r0, key_r1) — all feasible
        best_score     = float('inf')
        best_assign    = None
        current_score  = float('inf')   # score of the currently-locked assignment
        current_assign = None           # result dict for the locked assignment

        for k0, k1 in itertools.combinations(all_keys, 2):
            for (key_r0, key_r1) in [(k0, k1), (k1, k0)]:
                result = _score_assignment(
                    r0_pos, r1_pos, key_r0, key_r1,
                    (px, py), F_des, lock_r0, lock_r1
                )
                if result is None:
                    continue
                score, nd0, nd1, f0, f1, app0, app1 = result
                assign = {
                    0: (key_r0, app0, f0, nd0),
                    1: (key_r1, app1, f1, nd1),
                }
                candidates.append((score, key_r0, key_r1))
                if score < best_score:
                    best_score  = score
                    best_assign = assign
                # Track the score of the currently-locked pair (fresh geometry).
                if key_r0 == lock_r0 and key_r1 == lock_r1:
                    current_score  = score
                    current_assign = assign

        candidates.sort(key=lambda c: c[0])
        self.last_scores     = candidates[:3]
        self.last_best_score = candidates[0][0] if candidates else 0.0
        self.last_margin     = (candidates[1][0] - candidates[0][0]
                                if len(candidates) >= 2 else float('inf'))

        # Epsilon hysteresis: only switch if new winner beats current by > PLAN_EPSILON.
        # current_score already includes lock_bonus (-0.2), so effective threshold
        # is lock_bonus + PLAN_EPSILON = 0.25 of raw score advantage needed to flip.
        if (current_assign is not None
                and best_score >= current_score - PLAN_EPSILON):
            best_assign           = current_assign
            best_score            = current_score
            self.last_epsilon_held = True
        else:
            self.last_epsilon_held = False

        if best_assign is None:
            best_key = max(all_keys,
                           key=lambda k: _dot(_push_dir_for_face((px,py),k), F_des))
            app = _approach_pos((px,py), best_key)
            nd  = _push_dir_for_face((px,py), best_key)
            best_assign = {
                0: (best_key, app, 1.0, nd),
                1: (None, tuple(r1_pos), 0.0, (0.0, 0.0))
            }

        for i, vals in best_assign.items():
            if vals[0] is not None:
                self._locked[i] = vals[0]

        results = []
        for i, rpos in enumerate(robots_pos):
            key, app, force_scale, nd = best_assign[i]
            if key is None:
                results.append({'face': None, 'push_dir': (0.0, 0.0),
                                'approach': tuple(rpos), 'waypoints': [],
                                'force_scale': 0.0, 'score': 0.0})
                continue

            other_obs = []
            for j in range(len(robots_pos)):
                if j == i: continue
                other_obs.append(tuple(robots_pos[j]))
                j_key = best_assign[j][0]
                if j_key:
                    other_obs.append(_approach_pos((px,py), j_key))

            wps = _safe_waypoints(
                tuple(rpos), app, (px, py),
                nav_clr=NAV_CLEARANCE,
                robot_obstacles=other_obs,
                robot_clr=ROBOT_CLEARANCE
            )
            results.append({'face': key, 'push_dir': nd,
                            'approach': app, 'waypoints': wps,
                            'force_scale': force_scale,
                            'score': best_score})
        return results


if __name__ == '__main__':
    planner = ContactPlanner()
    scenarios = [
        {'name': 'Standard — NE rally',
         'robots': [(-1.75,-1.475),(-1.75,1.85)],
         'payload': (0,0), 'rally': (1.5,0.5)},
        {'name': 'Rally south',
         'robots': [(-1.75,-1.475),(-1.75,1.85)],
         'payload': (0,0), 'rally': (0.5,-1.5)},
        {'name': 'Rally NW',
         'robots': [(-1.75,-1.475),(-1.75,1.85)],
         'payload': (0,0), 'rally': (-1.5,0.5)},
        {'name': 'Payload shifted east',
         'robots': [(-1.75,-1.475),(-1.75,1.85)],
         'payload': (0.8,0.1), 'rally': (1.5,0.5)},
    ]
    import math as _math
    for sc in scenarios:
        F = _norm(sc['rally'][0]-sc['payload'][0], sc['rally'][1]-sc['payload'][1])
        print(f"\n{'='*60}\n  {sc['name']}")
        print(f"  F_desired: ({F[0]:+.3f},{F[1]:+.3f})  {_math.degrees(_math.atan2(F[1],F[0])):.1f}°")
        plan = planner.plan(sc['robots'], sc['payload'], sc['rally'])
        planner.reset()
        fs = [p['force_scale'] for p in plan]
        ratio = min(fs)/(max(fs)+1e-9) if max(fs)>0 else 0
        for i, p in enumerate(plan):
            face = p['face'] or '-'
            nav = 'direct' if not p['waypoints'] else str([(round(w[0],2),round(w[1],2)) for w in p['waypoints']])
            dx, dy = p['push_dir']
            app = p['approach']
            print(f"  R{i+1}: face={face:3s} push=({dx:+.3f},{dy:+.3f}) f={p['force_scale']:.3f}  ratio={ratio:.2f}  app=({app[0]:.3f},{app[1]:.3f})  nav={nav}")