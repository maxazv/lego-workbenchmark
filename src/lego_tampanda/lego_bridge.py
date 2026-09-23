"""DomainBridge wiring for LEGO assembly, modelled on tampanda's ``blocks_bridge``.

  bridge, objects, goals = make_lego_bridge(env, product, executor)
  plan = bridge.plan(objects, goals, planner_name="fast-downward")
  for action, params in plan:
      bridge.execute_action(action, *params)

The names registered here must match the PDDL domain; with the team's own domain
only the ``@bridge.predicate`` / ``@bridge.action`` names below change.
"""
from pathlib import Path

import mujoco
import numpy as np

from tampanda.tamp import DomainBridge
from .lego_scene import BRICK_SIZE, StudLatch, half_size, table_top_z, yaw_of, yaw_quat
from .lego_task import footprint, support_graph, world_target

DEFAULT_DOMAIN = Path(__file__).parent / "pddl" / "lego_domain.pddl"

# lego_sim's scoring tolerances (benchmark_core.score_episode).
XY_TOL, Z_TOL, YAW_TOL = 0.006, 0.004, np.radians(8.0)

# Open Panda finger, seen from above: outer face this far from the grasp centre along
# the closing axis, and this wide across it.
FINGER_OUTER, FINGER_HALF_WIDTH = 0.052, 0.011


def symmetry(block: dict) -> float:
    return np.pi if block["type"] == "brick_4x2" else np.pi / 2


def wrap(angle: float, period: float) -> float:
    return (angle + period / 2) % period - period / 2


def pose_error(env, block: dict, table_z: float) -> tuple[float, float, float]:
    """(xy, z, yaw) error of the brick against its target; yaw modulo brick symmetry."""
    target, target_yaw = world_target(block, table_z)
    err = env.get_object_position(block["id"]) - target
    yaw_err = wrap(yaw_of(env.get_object_orientation(block["id"])) - target_yaw, symmetry(block))
    return float(np.linalg.norm(err[:2])), float(err[2]), float(yaw_err)


def make_lego_bridge(env, product: dict, executor=None, domain_path=DEFAULT_DOMAIN, latch=True):
    """Pass ``executor=None`` for grounding and planning without robot motion.
    ``latch``: a brick released within tolerance of its target clicks onto its studs."""
    table_z = table_top_z(env)
    studs = StudLatch(env) if latch else None
    blocks = {b["id"]: b for b in product["blocks"]}
    supports = support_graph(product)
    bridge = DomainBridge(domain_path, env)
    bridge.lego_log = []          # grasp-selection notes for the failure analysis

    def at_target(name):
        xy, z, yaw = pose_error(env, blocks[name], table_z)
        return xy < XY_TOL and abs(z) < Z_TOL and abs(yaw) < YAW_TOL

    # ── Predicates measured in the simulator ──────────────────────────────
    @bridge.predicate("in_asm_area")
    def eval_in_asm_area(env, fluents, b):
        return not fluents.get(("holding", b), False) and at_target(b)

    @bridge.predicate("in_pick_area")
    def eval_in_pick_area(env, fluents, b):
        return not fluents.get(("holding", b), False) and not at_target(b)

    # ── Static predicates read off the product YAML ───────────────────────
    @bridge.predicate("on_base")
    def eval_on_base(env, fluents, b):
        return not supports[b]

    @bridge.predicate("supports")
    def eval_supports(env, fluents, s, b):
        return s in supports[b]

    # ── Fluents tracked through action effects ────────────────────────────
    bridge.fluent("holding", initial=None)
    bridge.fluent("handempty", initial=True)

    # ── Action executors ──────────────────────────────────────────────────
    if executor is not None:

        def fingers_blocked(b, finger_axis_at_target):
            """Would the open fingers hit an already placed brick at b's target pose?"""
            target, _ = world_target(blocks[b], table_z)
            axis = np.abs(np.round(finger_axis_at_target[:2]))        # yaws are multiples of 90 deg
            brick_half = np.array(footprint(blocks[b])) / 2
            inner = float(axis @ brick_half)
            centre_off = axis * (inner + FINGER_OUTER) / 2
            finger_half = axis * (FINGER_OUTER - inner) / 2 + (1 - axis) * FINGER_HALF_WIDTH
            for other in blocks.values():
                if other["id"] == b or not at_target(other["id"]):
                    continue
                o_target, _ = world_target(other, table_z)
                if o_target[2] + BRICK_SIZE[other["type"]][2] / 2 < target[2] - brick_half.min():
                    continue                                          # entirely below the fingers
                o_half = np.array(footprint(other)) / 2
                for sign in (1, -1):
                    gap = np.abs(target[:2] + sign * centre_off - o_target[:2]) - (finger_half + o_half)
                    if np.all(gap < -0.001):
                        return True
            return False

        @bridge.action("pick")
        def exec_pick(env, fluents, b):
            pos, quat = env.get_object_position(b), env.get_object_orientation(b)
            _, target_yaw = world_target(blocks[b], table_z)
            turn = wrap(target_yaw - yaw_of(quat), symmetry(blocks[b]))
            turn_mat = np.zeros(9)
            mujoco.mju_quat2Mat(turn_mat, yaw_quat(turn))
            # Prefer grasps whose fingers stay clear of placed neighbours at the target.
            cands = executor.grasp_planner.generate_candidates(pos, half_size(blocks[b]["type"]), quat)
            def blocked(c):
                mat = np.zeros(9)
                mujoco.mju_quat2Mat(mat, c.grasp_quat)
                return fingers_blocked(b, turn_mat.reshape(3, 3) @ mat.reshape(3, 3)[:, 1])
            flags = [blocked(c) for c in cands]
            cands = [c for c, f in zip(cands, flags) if not f] + [c for c, f in zip(cands, flags) if f]
            bridge.lego_log.append({"brick": b, "candidates": len(cands), "blocked": int(sum(flags))})
            ok = executor.pick(b, pos, half_size(blocks[b]["type"]), quat, candidates=cands)
            if not ok:
                return False, {}
            exec_pick.turn = turn
            return True, {("holding", b): True, ("handempty",): False}

        def put_down(env, fluents, b):
            target, _ = world_target(blocks[b], table_z)
            # Turn the hand by the yaw the brick still has to undergo.
            ee_quat = np.zeros(4)
            mujoco.mju_mulQuat(ee_quat, yaw_quat(exec_pick.turn), executor._last_grasp_quat)
            below = sorted(supports[b])
            for other in below[1:]:             # place() excepts only one target itself
                env.add_collision_exception(other)
            # Studs engage while the brick is still held in place, i.e. just before
            # the executor opens the gripper.
            open_gripper = env.controller.open_gripper
            def open_and_latch():
                if studs is not None and at_target(b):
                    studs.pin(b)
                open_gripper()
            env.controller.open_gripper = open_and_latch
            try:
                ok = executor.place(b, target, ee_quat=ee_quat,
                                    target_block_name=below[0] if below else None,
                                    place_clearance=0.001 if studs is not None else 0.003)
            finally:
                env.controller.open_gripper = open_gripper
            for other in below[1:]:
                env.remove_collision_exception(other)
            env.rest(1.0)
            # The gripper has opened either way, so the hand is empty even on failure;
            # the bridge only applies the returned delta on success.
            released = {("holding", b): False, ("handempty",): True}
            fluents.update(released)
            return bool(ok and at_target(b)), released

        bridge.action("place")(put_down)
        bridge.action("stack")(put_down)

    objects = {"brick": list(blocks)}
    goals = [("in_asm_area", b) for b in blocks]
    return bridge, objects, goals
