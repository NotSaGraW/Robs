# Unit Tests

Automated tests using pytest. Do not require CoppeliaSim to run.
They verify the internal logic of Python modules in isolation,
using mocks for the ZMQ connection and the sim object.

## Run
```bash
pytest tests/
pytest tests/ -v            # verbose output
pytest tests/ --tb=short    # short traceback
pytest tests/test_grid.py   # single file
```

## Conventions
- One test file per src/ module
- All sim/ZMQ dependencies must be mocked
- No hardware required — must pass on any machine without CoppeliaSim
- Descriptive names: `test_grid_world_to_cell_boundary`, `test_scene_push_vector_normalized`

## Planned tests
| File | Module | What it verifies |
|------|--------|-----------------|
| `test_grid.py` | `src/grid.py` | `world_to_cell`, `mark_free`, `mark_occupied`, ray casting, BFS pathfinding, `save_png` fallback |
| `test_scene.py` | `src/scene.py` | `push_vector` normalization, `pusher_position`, `dist2d`, `box_reached_target` threshold |
| `test_strategy_logic.py` | `src/strategy.py` | `_on_front_side` dot product, phase transitions, priority hierarchy (A > B) |

## Status
| File | Status |
|------|--------|
| `test_grid.py` | PENDING |
| `test_scene.py` | PENDING |
| `test_strategy_logic.py` | PENDING |