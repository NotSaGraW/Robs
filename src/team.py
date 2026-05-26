"""
team.py
Fleet identity and collective perception layer.

Responsibilities:
  - Agent registry: handles and identities for all Pioneer P3DX robots
  - Handle identification: resolve any sub-object handle to its root agent
  - Shared occupancy map: updated by all robots each cycle
  - Global state vector: structured observation for ML/RL integration

What Team does NOT own:
  - Mission state (payload position, current phase) → Strategy
  - Tactical decisions → Strategy

Design rationale (CTDE):
  Team = centralized critic input (global perception)
  Strategy = centralized policy (replaceable by learned policy)
  Each robot = decentralized actor
"""

from .grid import OccupancyGrid


# Root handles — given at deployment, never change
ROOT_HANDLES = {
    15:  'p3dx_1',
    99:  'p3dx_2',
    98:  'payload',
    13:  'floor',
    97:  'rally_point',
}


class Team:

    def __init__(self, sim, agents: dict,
                 payload_h: int, rally_point_h: int):
        """
        sim           — CoppeliaSim API handle
        agents        — {'p3dx_1': Robot, 'p3dx_2': Robot}
        payload_h     — handle of payload (identity, not position)
        rally_point_h — handle of rally point (identity, not tactics)
        """
        self.sim           = sim
        self.agents        = agents
        self.payload_h     = payload_h
        self.rally_point_h = rally_point_h

        # Shared occupancy map — all robots contribute each cycle
        self.map = OccupancyGrid(width=5.0, height=5.0, resolution=0.10)

        # Handle resolution cache — built lazily as objects are detected
        self._handle_cache = {}

        # Sensor data shared between agents (perception, not mission state)
        self._shared_perception = {}

        # Action log for replay buffer (ML integration)
        self._action_log = []

    # ------------------------------------------------------------------
    # Agent access
    # ------------------------------------------------------------------

    def get(self, name: str):
        """Returns agent by name."""
        return self.agents[name]

    def all_robots(self) -> list:
        """Returns all Pioneer P3DX agents."""
        return list(self.agents.values())

    # ------------------------------------------------------------------
    # Handle identification
    # ------------------------------------------------------------------

    def identify(self, handle: int) -> str:
        """
        Resolves a detected sensor handle to its root agent name.
        Walks up the scene hierarchy, caches results.

        Returns agent name ('p3dx_1', 'payload', etc.)
        or 'wall' for unknown static objects.
        """
        if handle in ROOT_HANDLES:
            return ROOT_HANDLES[handle]
        if handle in self._handle_cache:
            return self._handle_cache[handle]

        current = handle
        visited = [handle]
        while current != -1:
            try:
                parent = self.sim.getObjectParent(current)
            except Exception:
                break
            if parent in ROOT_HANDLES:
                name = ROOT_HANDLES[parent]
                for h in visited:
                    self._handle_cache[h] = name
                return name
            if parent == -1:
                break
            current = parent
            visited.append(current)

        for h in visited:
            self._handle_cache[h] = 'wall'
        return 'wall'

    def is_payload(self, handle: int) -> bool:
        return self.identify(handle) == 'payload'

    def is_teammate(self, handle: int) -> bool:
        return self.identify(handle) in self.agents

    def is_wall(self, handle: int) -> bool:
        return self.identify(handle) == 'wall'

    # ------------------------------------------------------------------
    # Shared perception — sensor data exchange between agents
    # ------------------------------------------------------------------

    def share(self, key: str, value):
        """Share perceptual data between agents (not mission state)."""
        self._shared_perception[key] = value

    def perceived(self, key: str, default=None):
        """Read shared perceptual data."""
        return self._shared_perception.get(key, default)

    # ------------------------------------------------------------------
    # Map update — called each cycle by Strategy
    # ------------------------------------------------------------------

    def update_map(self):
        """All robots update the shared map each cycle."""
        for agent in self.agents.values():
            pos = agent.get_position()
            self.map.mark_robot_path(pos[0], pos[1])
            self.map.update_from_sensor(
                pos[0], pos[1], agent.get_yaw(),
                agent.read_sensors_legacy()
            )

    # ------------------------------------------------------------------
    # ML integration — global state observation
    # ------------------------------------------------------------------

    def get_global_state(self) -> dict:
        """
        Structured global state for centralized critic in CTDE/MARL.
        Contains fleet perception — not mission state (that belongs to Strategy).
        """
        state = {
            'map_coverage': self.map.coverage_percent(),
            'agents':       {}
        }
        for name, agent in self.agents.items():
            readings = agent.read_sensors_legacy()
            state['agents'][name] = {
                'position': agent.get_position(),
                'yaw':      agent.get_yaw(),
                'sensors':  [d for _, d in readings],
            }
        return state

    def record_action(self, agent_name: str, action: str,
                      speed: float = None):
        """Log action for offline RL replay buffer."""
        self._action_log.append({
            'agent':  agent_name,
            'action': action,
            'speed':  speed,
            'state':  self.get_global_state(),
        })

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def status(self) -> str:
        return (f"Team[{len(self.agents)} robots] "
                f"map:{self.map.coverage_percent():.0f}% "
                f"cache:{len(self._handle_cache)} handles resolved")