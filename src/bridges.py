# tampanda setup
import numpy as np
import math
import mujoco

from tampanda.tamp import DomainBridge
from tampanda.planners.grasp_planner import GRASP_CONTACT_OFFSET

from yaml_loader import Brick
from emit_coarse import support_tree, root_location, pick_location

from scenes import (
    BRICK_HALF_HEIGHT, COLORS, SYMMETRY,
    Layout,
    qz, qmul, yaw_of, rotz, wrap,
    SimpleLegoScene,
)



def pddl_name(name):
    """unified-planning PDDL reader rejects object names starting with a digit"""
    return name if name[0].isalpha() else f"b_{name}"


def execute_plan(bridge, plan):
    # blind plan execution (not TAMP)
    for i, (name, args) in enumerate(plan):
        ok, _ = bridge.execute_action(name, *args)
        print(f"[{i+1}/{len(plan)}] {name}({', '.join(args)}) -> {'ok' if ok else 'FAILED'}")
        if not ok:
            return False, i
    return True, len(plan)


class LegoCoarseSimpleSLSBridge:
    # domain_path = ""

    def make_bridge(domain_path, scene: SimpleLegoScene):
        """Set up the DomainBridge between the PDDL domain and the SimpleLegoScene (env and executor).
        Ie we have to connect/track predicates to the bridge and define the symbolic actions in the environment.
        """
        bricks = list(scene.bricks.values())
        bridge = DomainBridge(domain_path, scene.env)

        # we use fluents because these are all predicates which are only influenced by action effects
        # if we want to use TAMP planning we would need to define these as predicates (not that difficult as main work already done in SimpleLegoScene)
        bridge.fluent("hand-empty", initial=True)
        bridge.fluent("holding")
        bridge.fluent("at", initial=[(pddl_name(b.name), pick_location(b.name)) for b in bricks])
        roots = {r for r in scene.bricks if r not in support_tree(bricks)[0]}
        bridge.fluent("clear", initial=[root_location(r) for r in roots])
        bridge.fluent("top-clear", initial=[pddl_name(b.name) for b in bricks])
        bridge.fluent("stacked_on")
        ex = scene.executor
        real_name = {pddl_name(n): n for n in scene.bricks}   # translate DomainBridge params back to IR/MuJoCo names

        def _place_at(env, brick, target, below=None):
            pos, yaw = target
            br = scene.bricks[brick]
            _, rel, ee_q = scene._held  # get how we are holding current brick
            yaw_now = yaw_of(env.get_object_orientation(brick))
            sym = SYMMETRY[br.type]
            # little trick: multiple yaws of brick yield same orientation => generate many yaws yielding same rotation and let motion planner choose one that works wrt joint constraints
            options = sorted({wrap(yaw + k * sym - yaw_now) for k in range(360 // sym)}, key=abs)
            for d in options:
                ee_quat = qmul(qz(d), ee_q)
                # some magic linear algebra so we know where to place brick EVEN AFTER wrist rotation
                place_center = pos - rotz(rel, d) - np.array([0.0, 0.0, GRASP_CONTACT_OFFSET])
                if ex.place(brick, place_center, ee_quat, target_block_name=below,
                            place_clearance=scene.layout.place_clearance):
                    return True
            return False

        @bridge.action("pick")
        def exec_pick(env, fluents, b, frm):
            # see https://snoato.github.io/TAMPanda/tutorial.html#pickplace
            b = real_name[b]
            pos, half, quat = env.get_object_position(b), env.get_object_half_size(b), env.get_object_orientation(b)
            cands = scene.grasp_planner.generate_candidates(pos, half, quat)
            if not ex.pick(b, pos, half, quat, candidates=cands):
                return False, {}
            scene._held = (b, env.get_object_position(b) - scene.ee_pos(), ex._last_grasp_quat.copy())
            # return action success and the action effects
            return True, {("holding", pddl_name(b)): True, ("hand-empty",): False}

        @bridge.action("unstack")
        def exec_unstack(env, fluents, b, on):
            # this is just here for completion of domain: no plan actually uses this action but plans still allowed to use it
            return exec_pick(env, fluents, b, None)   # physically identical to pick

        @bridge.action("place")
        def exec_place(env, fluents, b, to):
            b = real_name[b]
            # look which location we want to place target (pick-location or assembly-location)
            # ideally no plan would place in pick but again not restricted in the domain, so we kept for completion
            target = scene.target_pose[b] if to.startswith("asm_") else scene.pick_pose[to[len("pick_"):]]
            if not _place_at(env, b, target):
                return False, {}
            scene._held = None
            return True, {("holding", pddl_name(b)): False, ("hand-empty",): True}

        @bridge.action("stack")
        def exec_stack(env, fluents, b, on):
            b, on = real_name[b], real_name[on]
            # FIXME: should get current position of on and place there, not to target_pose
            # pos, half, quat = env.get_object_position(b), env.get_object_half_size(b), env.get_object_orientation(b)
            # target_pos = pos + half
            # NOTE: stack assumes that brick 'on' is already at its target pose => we can stack our brick to target pose (which is above)
            if not _place_at(env, b, scene.target_pose[b], below=on):
                return False, {}
            scene._held = None
            return True, {("holding", pddl_name(b)): False, ("hand-empty",): True}

        return bridge

    def build_objects_and_goal(bricks: list[Brick]):
        """Set up all objects, predicates and goal conditions for bridge (NOTE that this is for lego_coarse_simple.pddl domain)"""
        parent, children, multi = support_tree(bricks)
        for n, names in multi.items():
            print(f"NOTE: {n} rests on {names}; keeping only {parent[n].name} (see bridge limitation)")
        roots = [b for b in bricks if b.name not in parent]

        objects = {"brick": [pddl_name(b.name) for b in bricks],
                "location": [pick_location(b.name) for b in bricks] + [root_location(r.name) for r in roots]}
        goals = [("at", pddl_name(r.name), root_location(r.name)) for r in roots]
        goals += [("stacked_on", pddl_name(b.name), pddl_name(parent[b.name].name)) for b in bricks if b.name in parent]
        return objects, goals


class LegoCoarseSimpleV2SLSBridge:
    # domain_path = ""

    def make_bridge(domain_path, scene: SimpleLegoScene):
        """Set up the DomainBridge between the PDDL domain and the SimpleLegoScene (env and executor).
        Ie we have to connect/track predicates to the bridge and define the symbolic actions in the environment.
        """
        bricks = list(scene.bricks.values())
        bridge = DomainBridge(domain_path, scene.env)

        # we use fluents because these are all predicates which are only influenced by action effects
        # if we want to use TAMP planning we would need to define these as predicates (not that difficult as main work already done in SimpleLegoScene)
        roots = {r for r in scene.bricks if r not in support_tree(bricks)[0]}
        bridge.fluent("is-root", initial=[pddl_name(r) for r in roots])
        bridge.fluent("at-target")
        bridge.fluent("top-clear", initial=[pddl_name(b.name) for b in bricks])
        bridge.fluent("stacked_on")
        bridge.fluent("holding")
        bridge.fluent("hand-empty", initial=True)
        ex = scene.executor
        real_name = {pddl_name(n): n for n in scene.bricks}   # translate DomainBridge params back to IR/MuJoCo names

        def _place_at(env, brick, target, below=None):
            """This method is essentially just PickPlaceExecutor.place(...).
            The added complexity before that call comes from figuring out how to rotate arm wrist so brick target-orientation correct.
            """
            pos, yaw = target
            br = scene.bricks[brick]
            _, rel, ee_q = scene._held  # how we are holding current brick (see exec_pick below)
            yaw_now = yaw_of(env.get_object_orientation(brick))
            sym = SYMMETRY[br.type]
            # little trick: multiple yaws of brick yield same orientation => generate many yaws yielding same rotation and let motion planner choose one that works wrt joint constraints
            # could just choose one, but this way we get more options => higher success rate
            options = sorted({wrap(yaw + k * sym - yaw_now) for k in range(360 // sym)}, key=abs)
            for d in options:
                ee_quat = qmul(qz(d), ee_q)
                # some magic linear algebra so we know where to place brick EVEN AFTER wrist rotation
                place_center = pos - rotz(rel, d) - np.array([0.0, 0.0, GRASP_CONTACT_OFFSET])
                if ex.place(brick, place_center, ee_quat, target_block_name=below,
                            place_clearance=scene.layout.place_clearance):
                    return True
            return False

        @bridge.action("pick")
        def exec_pick(env, fluents, b):
            # see https://snoato.github.io/TAMPanda/tutorial.html#pickplace
            b = real_name[b]
            pos, half, quat = env.get_object_position(b), env.get_object_half_size(b), env.get_object_orientation(b)
            cands = scene.grasp_planner.generate_candidates(pos, half, quat)
            if not ex.pick(b, pos, half, quat, candidates=cands):
                return False, {}
            # save brick, relative position of brick and end-effector, orientation of arm
            scene._held = (b, env.get_object_position(b) - scene.ee_pos(), ex._last_grasp_quat.copy())
            # return action success and the action effects
            return True, {("holding", pddl_name(b)): True, ("hand-empty",): False}

        @bridge.action("place")
        def exec_place(env, fluents, b):
            b = real_name[b]
            target = scene.target_pose[b]
            if not _place_at(env, b, target):
                return False, {}
            scene._held = None
            return True, {("holding", pddl_name(b)): False, ("at-target", pddl_name(b)): True, ("hand-empty",): True}

        @bridge.action("stack")
        def exec_stack(env, fluents, b, on):
            b, on = real_name[b], real_name[on]
            # NOTE: stack assumes that brick 'on' is already at its target pose => we can stack our brick to target pose (which is above)
            # works for this domain as this is assumption of pddl domain as well
            if not _place_at(env, b, scene.target_pose[b], below=on):
                return False, {}
            scene._held = None
            return True, {
                ("holding", pddl_name(b)): False, ("top-clear", pddl_name(on)): False,
                ("stacked_on", pddl_name(b), pddl_name(on)): True, ("at-target", pddl_name(b)): True,
                ("hand-empty",): True,
            }

        return bridge

    def build_objects_and_goal(bricks: list[Brick]):
        """Set up all objects, predicates and goal conditions for bridge (NOTE that this is for lego_coarse_simple.pddl domain)"""
        parent, children, multi = support_tree(bricks)
        for n, names in multi.items():
            print(f"NOTE: {n} rests on {names}; keeping only {parent[n].name} (see bridge limitation)")
        roots = [b for b in bricks if b.name not in parent]

        objects = {"brick": [pddl_name(b.name) for b in bricks]}
        goals = [("at-target", pddl_name(b.name)) for b in bricks]
        goals += [("stacked_on", pddl_name(b.name), pddl_name(parent[b.name].name)) for b in bricks if b.name in parent]
        return objects, goals


class LegoCoarseSimpleV2SLSWeldBridge:
    # domain_path = ""

    def make_bridge(domain_path, scene: SimpleLegoScene):
        """Set up the DomainBridge between the PDDL domain and the SimpleLegoScene (env and executor).
        Ie we have to connect/track predicates to the bridge and define the symbolic actions in the environment.
        """
        bricks = list(scene.bricks.values())
        bridge = DomainBridge(domain_path, scene.env)

        # we use fluents because these are all predicates which are only influenced by action effects
        # if we want to use TAMP planning we would need to define these as predicates (not that difficult as main work already done in SimpleLegoScene)
        roots = {r for r in scene.bricks if r not in support_tree(bricks)[0]}
        bridge.fluent("is-root", initial=[pddl_name(r) for r in roots])
        bridge.fluent("at-target")
        bridge.fluent("top-clear", initial=[pddl_name(b.name) for b in bricks])
        bridge.fluent("stacked_on")
        bridge.fluent("holding")
        bridge.fluent("hand-empty", initial=True)
        ex = scene.executor
        real_name = {pddl_name(n): n for n in scene.bricks}   # translate DomainBridge params back to IR/MuJoCo names

        def _place_at(env, brick, target, below=None):
            """Basically just a copy of PickPlaceExecutor.place but with snap + welding logic when gripper releases brick."""
            ex = scene.executor
            pos, yaw = target
            br = scene.bricks[brick]
            _, rel, ee_q = scene._held
            yaw_now = yaw_of(env.get_object_orientation(brick))
            sym = SYMMETRY[br.type]

            options = sorted({wrap(yaw + k * sym - yaw_now) for k in range(360 // sym)}, key=abs)
            for d in options:
                ee_quat = qmul(qz(d), ee_q)
                place_center = pos - rotz(rel, d) - np.array([0.0, 0.0, GRASP_CONTACT_OFFSET])

                # PickPlaceExecutor.place starts here!!!! (above is just normal _place_at from before)
                ee_place = place_center.copy()
                ee_place[2] += GRASP_CONTACT_OFFSET + scene.layout.place_clearance
                ee_approach = ee_place.copy()
                ee_approach[2] += 0.15   # PickPlaceExecutor.place()'s own default approach_height
    
                env.add_collision_exception(brick)
                if below is not None:
                    env.add_collision_exception(below)
    
                path = scene.planner.plan_to_pose(ee_approach, ee_quat, dt=0.005, max_iterations=ex.max_plan_iters)
                if path is None:
                    ex._clear_exceptions(brick, below); continue
                env.execute_path(path, scene.planner, step_size=ex.approach_step_size)
                env.wait_idle(settle_steps=ex.settle_steps)
    
                if ex.use_attachment:
                    env.detach_object()
    
                path = scene.planner.plan_to_pose(ee_place, ee_quat, dt=0.005, max_iterations=ex.max_plan_iters)
                if path is None:
                    ex._clear_exceptions(brick, below); continue
                env.execute_path(path, scene.planner, step_size=ex.place_step_size)
                env.wait_idle(settle_steps=ex.settle_steps)
    
                env.controller.open_gripper()

                # NOTE: now snapping + welding logic starts after gripper opens
                # IDEA: while brick released and moves, if brick pose within tolerance to snap, we snap it to the grid:
                MAX_STEPS = 300
                welded = False
                for _ in range(MAX_STEPS):
                    env.controller.step()
                    env.step()
                    if welded:
                        # we have already locked the brick => just run remaining steps for gripper to open etc (PPE code)
                        continue
                    
                    # check whether brick within tolerance of taret pos (if ok => snap and weld)
                    p = env.get_object_position(brick)
                    measured_yaw = yaw_of(env.get_object_orientation(brick))
                    delta = wrap(measured_yaw - yaw) % sym
                    delta = delta if delta <= sym / 2 else delta - sym   # signed, nearest symmetric equivalent
                    ok = (np.linalg.norm(p[:2] - pos[:2]) < 0.006 and abs(p[2] - pos[2]) < 0.006 and abs(delta) < 6.0)
                    if ok:
                        # snap onto the exact IR pose before welding
                        snap_yaw = measured_yaw - delta
                        a = math.radians(snap_yaw) / 2  # target angle in radians
                        # get index of where brick info stored in global mujoco vectors
                        bid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, brick)
                        qa = env.model.jnt_qposadr[env.model.body_jntadr[bid]]
                        va = env.model.jnt_dofadr[env.model.body_jntadr[bid]]
                        # set the position, orientation and velocity of brick (ie the snap)
                        env.data.qpos[qa:qa + 3] = pos
                        env.data.qpos[qa + 3:qa + 7] = [math.cos(a), 0.0, 0.0, math.sin(a)]
                        env.data.qvel[va:va + 6] = 0.0
                        env.forward()  # called so xpos, xmat are updated
                        # weld it
                        scene.weld(brick, below if below is not None else "table")
                        welded = True  # from here on this loop only keeps stepping, doesnt re-check
                if not welded:
                    print(f"  NOTE: {brick} never settled within tolerance in {MAX_STEPS} steps, not welding (left free)")

                # run remaining code of PickPlaceExecutor.place
                ex._clear_exceptions(brick, below)
                path = scene.planner.plan_to_pose(ee_approach, ee_quat, dt=0.005, max_iterations=ex.max_plan_iters)
                if path is not None:
                    env.execute_path(path, scene.planner, step_size=ex.retreat_step_size)
                    env.wait_idle(settle_steps=ex.settle_steps)
                return True
            return False


        @bridge.action("pick")
        def exec_pick(env, fluents, b):
            # see https://snoato.github.io/TAMPanda/tutorial.html#pickplace
            b = real_name[b]
            pos, half, quat = env.get_object_position(b), env.get_object_half_size(b), env.get_object_orientation(b)
            cands = scene.grasp_planner.generate_candidates(pos, half, quat)
            if not ex.pick(b, pos, half, quat, candidates=cands):
                return False, {}
            # save brick, relative position of brick and end-effector, orientation of arm
            scene._held = (b, env.get_object_position(b) - scene.ee_pos(), ex._last_grasp_quat.copy())
            # return action success and the action effects
            return True, {("holding", pddl_name(b)): True, ("hand-empty",): False}

        @bridge.action("place")
        def exec_place(env, fluents, b):
            b = real_name[b]
            target = scene.target_pose[b]
            if not _place_at(env, b, target):
                return False, {}
            scene._held = None
            return True, {("holding", pddl_name(b)): False, ("at-target", pddl_name(b)): True, ("hand-empty",): True}

        @bridge.action("stack")
        def exec_stack(env, fluents, b, on):
            b, on = real_name[b], real_name[on]
            # NOTE: stack assumes that brick 'on' is already at its target pose => we can stack our brick to target pose (which is above)
            # works for this domain as this is assumption of pddl domain as well
            if not _place_at(env, b, scene.target_pose[b], below=on):
                return False, {}
            scene._held = None
            return True, {
                ("holding", pddl_name(b)): False, ("top-clear", pddl_name(on)): False,
                ("stacked_on", pddl_name(b), pddl_name(on)): True, ("at-target", pddl_name(b)): True,
                ("hand-empty",): True,
            }

        return bridge

    def build_objects_and_goal(bricks: list[Brick]):
        """Set up all objects, predicates and goal conditions for bridge (NOTE that this is for lego_coarse_simple.pddl domain)"""
        parent, children, multi = support_tree(bricks)
        for n, names in multi.items():
            print(f"NOTE: {n} rests on {names}; keeping only {parent[n].name} (see bridge limitation)")
        roots = [b for b in bricks if b.name not in parent]

        objects = {"brick": [pddl_name(b.name) for b in bricks]}
        goals = [("at-target", pddl_name(b.name)) for b in bricks]
        goals += [("stacked_on", pddl_name(b.name), pddl_name(parent[b.name].name)) for b in bricks if b.name in parent]
        return objects, goals