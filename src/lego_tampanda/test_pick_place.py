"""Step 1 feasibility test: can TAMPanda's GraspPlanner + PickPlaceExecutor build a
5-brick Duplo tower and place a 4x2 brick that has to be turned by 90 degrees, all
within lego_sim's tolerances (6 mm xy, 4 mm z, 10 deg yaw)?

  python -m lego_tampanda.test_pick_place [table_clearance_m]

GraspPlanner's default table_clearance (0.025) rejects a 19 mm brick lying on the
table; 0.003 is the value that works.
"""
import sys
import mujoco
import numpy as np

from tampanda import RRTStar, GraspPlanner
from tampanda.planners.pick_place import PickPlaceExecutor
from .lego_scene import make_lego_builder, half_size

XY_TOL, Z_TOL, YAW_TOL_DEG = 0.006, 0.004, 10.0
clearance = float(sys.argv[1]) if len(sys.argv) > 1 else 0.003


def yaw_quat(yaw):
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])

def quat_mul(a, b):
    out = np.zeros(4); mujoco.mju_mulQuat(out, a, b); return out

def yaw_of(quat):
    w, x, y, z = quat
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# Robot base is at the world origin; the table spans x -0.4..1.2, y 0.2..1.0.
tower = [{"id": f"t{i}", "type": "brick_2x2", "color": c, "spawn": (0.10 + 0.06 * i, 0.40, (i % 2) * np.pi / 2)}
         for i, c in enumerate(["red", "green", "blue", "yellow", "white"])]
beam = {"id": "beam", "type": "brick_4x2", "color": "orange", "spawn": (0.20, 0.52, np.pi / 2)}
bricks = tower + [beam]

env = make_lego_builder(bricks).build_env(rate=200.0)
env.forward()
g = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "simple_table_surface")
table_z = float(env.data.geom_xpos[g][2] + env.model.geom_size[g][2])
for b in bricks:
    x, y, yaw = b["spawn"]
    env.set_object_pose(b["id"], np.array([x, y, table_z + half_size(b["type"])[2] + 0.002]), yaw_quat(yaw))
env.reset_velocities(); env.forward(); env.rest(1.0)

planner = RRTStar(env)
planner.max_iterations, planner.step_size, planner.goal_sample_rate = 3000, 0.2, 0.2
executor = PickPlaceExecutor(env, planner, GraspPlanner(table_z=table_z, table_clearance=clearance),
                             use_attachment=True)

layer = 2 * half_size("brick_2x2")[2]
base = np.array([0.45, 0.50, table_z + layer / 2])
jobs = [(b, base + [0, 0, layer * i], 0.0, tower[i - 1]["id"] if i else None) for i, b in enumerate(tower)]
jobs.append((beam, np.array([0.45, 0.35, table_z + layer / 2]), 0.0, None))

def report(brick, target, target_yaw, tag=""):
    sym = np.pi if brick["type"] == "brick_4x2" else np.pi / 2
    err = env.get_object_position(brick["id"]) - target
    yaw_err = (yaw_of(env.get_object_orientation(brick["id"])) - target_yaw + sym / 2) % sym - sym / 2
    good = np.linalg.norm(err[:2]) < XY_TOL and abs(err[2]) < Z_TOL and abs(np.degrees(yaw_err)) < YAW_TOL_DEG
    print(f"== {brick['id']:>5}{tag}: xy={1e3*np.linalg.norm(err[:2]):.2f}mm z={1e3*err[2]:+.2f}mm "
          f"yaw={np.degrees(yaw_err):+.1f}deg ok={bool(good)}")
    return bool(good)

for brick, target, target_yaw, under in jobs:
    name, h = brick["id"], half_size(brick["type"])
    quat = env.get_object_orientation(name)
    picked = executor.pick(name, env.get_object_position(name), h, quat)
    if picked:
        # Turn the hand by the yaw the brick still has to undergo (reduced by symmetry).
        sym = np.pi if brick["type"] == "brick_4x2" else np.pi / 2
        turn = (target_yaw - yaw_of(quat) + sym / 2) % sym - sym / 2
        ee_quat = quat_mul(yaw_quat(turn), executor._last_grasp_quat)
        executor.place(name, target.copy(), ee_quat=ee_quat, target_block_name=under)
    env.rest(1.0)
    report(brick, target, target_yaw)

print("---- final state of every brick")
wins = sum(report(b, t, y, " final") for b, t, y, _ in jobs)
print(f"{wins}/{len(jobs)} bricks within tolerance")
