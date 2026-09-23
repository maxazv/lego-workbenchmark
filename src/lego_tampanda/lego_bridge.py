"""DomainBridge wiring for the team's coarse domain (``src/pddl/coarse/lego_coarse.pddl``).

  bridge, objects, goals = make_lego_bridge(env, product, executor)
  plan = bridge.plan(objects, goals, planner_name="fast-downward")
  for action, params in plan:
      bridge.execute_action(action, *params)

"""
from pathlib import Path

import mujoco
import numpy as np

from tampanda.tamp import DomainBridge
from .lego_scene import BRICK_SIZE, StudLatch, half_size, table_top_z, yaw_of, yaw_quat
from .lego_task import DIM, assembly_locations, footprint, parent_map, support_graph, world_target

DEFAULT_DOMAIN = Path(__file__).parents[1] / "pddl" / "coarse" / "lego_coarse.pddl"


XY_TOL, Z_TOL, YAW_TOL = 0.006, 0.004, np.radians(8.0)


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
    parent = parent_map(product)            
    asm_loc = assembly_locations(product)
    roots = [b for b in blocks if b not in parent]
    bridge = DomainBridge(domain_path, env)
    bridge.lego_log = []          

    def at_target(name):
        xy, z, yaw = pose_error(env, blocks[name], table_z)
        return xy < XY_TOL and abs(z) < Z_TOL and abs(yaw) < YAW_TOL

    def held(fluents, name):
        return bool(fluents.get(("holding", name), False))

    def predicate(name):
        """Like ``bridge.predicate`` but silently skipped if the domain lacks the predicate."""
        def register(fn):
            if name in bridge.predicate_names:
                bridge.predicate(name)(fn)
            return fn
        return register

    
    @predicate("at")
    def eval_at(env, fluents, b, loc):
        if held(fluents, b):
            return False
        return loc == (asm_loc[b] if at_target(b) else f"pick_{b}")

    @predicate("stacked_on")
    def eval_stacked_on(env, fluents, a, b):
        return parent.get(a) == b and not held(fluents, a) and at_target(a) and at_target(b)

    @predicate("shape")
    def eval_shape(env, fluents, b, t):
        return DIM[blocks[b]["type"]] == t

    @predicate("is_color")
    def eval_is_color(env, fluents, b, c):
        return blocks[b].get("color", "none") == c

    bridge.fluent("holding", initial=None)
    bridge.fluent("hand-empty", initial=True)

    
    if executor is not None:
        grasp = {"turn": 0.0}          

        def fingers_blocked(b, finger_axis_at_target):
            """Would the open fingers hit an already placed brick at b's target pose?"""
            target, _ = world_target(blocks[b], table_z)
            axis = np.abs(np.round(finger_axis_at_target[:2]))        
            brick_half = np.array(footprint(blocks[b])) / 2
            inner = float(axis @ brick_half)
            centre_off = axis * (inner + FINGER_OUTER) / 2
            finger_half = axis * (FINGER_OUTER - inner) / 2 + (1 - axis) * FINGER_HALF_WIDTH
            for other in blocks.values():
                if other["id"] == b or not at_target(other["id"]):
                    continue
                o_target, _ = world_target(other, table_z)
                if o_target[2] + BRICK_SIZE[other["type"]][2] / 2 < target[2] - brick_half.min():
                    continue                                          
                o_half = np.array(footprint(other)) / 2
                for sign in (1, -1):
                    gap = np.abs(target[:2] + sign * centre_off - o_target[:2]) - (finger_half + o_half)
                    if np.all(gap < -0.001):
                        return True
            return False

        @bridge.action("pick")
        def exec_pick(env, fluents, b, source=None):
            pos, quat = env.get_object_position(b), env.get_object_orientation(b)
            _, target_yaw = world_target(blocks[b], table_z)
            turn = wrap(target_yaw - yaw_of(quat), symmetry(blocks[b]))
            turn_mat = np.zeros(9)
            mujoco.mju_quat2Mat(turn_mat, yaw_quat(turn))

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
            grasp["turn"] = turn
            return True, {("holding", b): True, ("hand-empty",): False}

        def put_down(env, fluents, b, primary=None):
            """Release b at its target; ``primary`` is the supporter the executor may touch."""
            target, _ = world_target(blocks[b], table_z)
            ee_quat = np.zeros(4)
            mujoco.mju_mulQuat(ee_quat, yaw_quat(grasp["turn"]), executor._last_grasp_quat)
            others = sorted(supports[b] - {primary})
            for other in others:                
                env.add_collision_exception(other)
            open_gripper = env.controller.open_gripper
            def open_and_latch():
                if studs is not None and at_target(b):
                    studs.pin(b)
                open_gripper()
            env.controller.open_gripper = open_and_latch
            try:
                ok = executor.place(b, target, ee_quat=ee_quat, target_block_name=primary,
                                    place_clearance=0.001 if studs is not None else 0.003)
            finally:
                env.controller.open_gripper = open_gripper
            for other in others:
                env.remove_collision_exception(other)
            env.rest(1.0)
            
            released = {("holding", b): False, ("hand-empty",): True}
            fluents.update(released)
            return bool(ok and at_target(b)), released

        @bridge.action("place")
        def exec_place(env, fluents, b, to=None):
            return put_down(env, fluents, b)

        @bridge.action("stack")
        def exec_stack(env, fluents, b, on=None, to=None):
            
            if not at_target(on):
                bridge.lego_log.append({"brick": b, "infeasible": f"{on} not at its target"})
                return False, {}
            return put_down(env, fluents, b, primary=on)

    objects = {"brick": list(blocks),
               "type": sorted({DIM[b["type"]] for b in blocks.values()}),
               "color": sorted({b.get("color", "none") for b in blocks.values()}),
               "location": [f"pick_{b}" for b in blocks] + [f"asm_{r}" for r in roots]}

    goals = [("at", r, asm_loc[r]) for r in roots] + [("stacked_on", b, p) for b, p in parent.items()]
    return bridge, objects, goals
