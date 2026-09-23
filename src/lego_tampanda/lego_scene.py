"""TAMPanda scene with Duplo bricks (plain boxes), modelled on ``make_blocks_builder``."""
from pathlib import Path

import numpy as np

from tampanda.scenes import ArmSceneBuilder, TABLE_SYMBOLIC_TEMPLATE

TEMPLATES = Path(__file__).parent / "templates"

# Full brick sizes in metres (x, y, z).  z is the dataset's layer pitch.
BRICK_SIZE = {"brick_2x2": (0.032, 0.032, 0.0192), "brick_4x2": (0.064, 0.032, 0.0192)}

COLORS = {"red": [0.85, 0.1, 0.1, 1], "green": [0.1, 0.65, 0.2, 1], "blue": [0.1, 0.3, 0.85, 1],
          "yellow": [0.95, 0.8, 0.1, 1], "white": [0.95, 0.95, 0.95, 1],
          "orange": [0.95, 0.5, 0.1, 1], "black": [0.1, 0.1, 0.1, 1]}


def half_size(brick_type: str) -> np.ndarray:
    return np.array(BRICK_SIZE[brick_type]) / 2


def make_lego_builder(bricks: list[dict]) -> ArmSceneBuilder:
    """``bricks``: dicts with ``id``, ``type`` and optionally ``color``.  Bricks are
    parked off-screen like the exercise block pool; place them with ``set_object_pose``."""
    b = ArmSceneBuilder()
    b.add_resource("table_symbolic", TABLE_SYMBOLIC_TEMPLATE)
    for brick_type in BRICK_SIZE:
        b.add_resource(brick_type, str(TEMPLATES / f"{brick_type}.xml"))
    # Same simulation options as the exercise blocks world.
    b._options = {"timestep": "0.005", "iterations": "5", "ls_iterations": "8",
                  "integrator": "implicitfast", "solver": "Newton",
                  "density": "1.2", "viscosity": "0.01"}
    b._option_flags = {"eulerdamp": "disable"}
    b._custom_numerics = {"max_contact_points": "12"}
    b.add_object("table_symbolic", name="simple_table",
                 pos=[0.0, 0.4, 0.0], quat=[0.0, 0.0, 0.0, 1.0])
    for i, brick in enumerate(bricks):
        b.add_object(brick["type"], name=brick["id"], pos=[100.0, 0.1 * i, 0.0096],
                     rgba=COLORS.get(brick.get("color"), [0.6, 0.6, 0.6, 1]))
    return b


# ---------------------------------------------------------------- pose helpers
def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def yaw_of(quat) -> float:
    w, x, y, z = quat
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def table_top_z(env) -> float:
    import mujoco
    g = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "simple_table_surface")
    env.forward()
    return float(env.data.geom_xpos[g][2] + env.model.geom_size[g][2])


# -------------------------------------------------------------------- spawning
# Pick area on the table (robot base = world origin), clear of the assembly area.
SPAWN_REGION = {"x": (0.00, 0.30), "y": (0.35, 0.60)}
SPAWN_MIN_DIST = 0.09      # centre distance, leaves room for the open gripper


def spawn_bricks(env, product: dict, seed: int) -> None:
    """Seeded loose placement like lego_sim's episodes: random xy, yaw 0 or 90 deg."""
    rng = np.random.default_rng(seed)
    table_z, placed = table_top_z(env), []
    for block in product["blocks"]:
        for _ in range(1000):
            xy = np.array([rng.uniform(*SPAWN_REGION["x"]), rng.uniform(*SPAWN_REGION["y"])])
            if all(np.linalg.norm(xy - p) >= SPAWN_MIN_DIST for p in placed):
                break
        else:
            raise RuntimeError("spawn region too crowded")
        placed.append(xy)
        z = table_z + BRICK_SIZE[block["type"]][2] / 2 + 0.002
        env.set_object_pose(block["id"], np.array([*xy, z]), yaw_quat(rng.choice([0.0, np.pi / 2])))
    env.reset_velocities()
    env.forward()
    env.rest(1.0)


# ------------------------------------------------------------------ stud latch
class StudLatch:
    """Stand-in for Duplo studs on plain boxes: a pinned brick is held at the pose it
    had when pinned, the way ``attach_object_to_ee`` holds a carried one.  lego_sim's
    ROS bridge does the same with its "fake welds" in snap mode."""

    def __init__(self, env):
        self.env, self.pins = env, {}
        step = env.step

        def latched_step():
            self._apply()
            step()
            self._apply()
        env.step = latched_step

    def pin(self, body_name: str) -> None:
        import mujoco
        j = mujoco.mj_name2id(self.env.model, mujoco.mjtObj.mjOBJ_JOINT, f"{body_name}_freejoint")
        qadr, vadr = self.env.model.jnt_qposadr[j], self.env.model.jnt_dofadr[j]
        self.pins[body_name] = (qadr, vadr, self.env.data.qpos[qadr:qadr + 7].copy())

    def _apply(self) -> None:
        for qadr, vadr, pose in self.pins.values():
            self.env.data.qpos[qadr:qadr + 7] = pose
            self.env.data.qvel[vadr:vadr + 6] = 0.0
