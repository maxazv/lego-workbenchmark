# tampanda setup

import math
from dataclasses import dataclass
import numpy as np

from tampanda import ArmSceneBuilder, RRTStar, GraspPlanner, PickPlaceExecutor
from tampanda.scenes import TABLE_SYMBOLIC_TEMPLATE
from tampanda.tamp import DomainBridge
from tampanda.planners.grasp_planner import GRASP_CONTACT_OFFSET

from yaml_loader import Brick, LAYER_PITCH
from emit_coarse import support_tree, root_location, pick_location

# see yaml_loader
BRICK_HALF_HEIGHT = LAYER_PITCH / 2
COLORS = {"red": [0.85, 0.1, 0.1, 1], "green": [0.1, 0.7, 0.2, 1],
          "blue": [0.1, 0.3, 0.9, 1], "yellow": [0.95, 0.85, 0.1, 1]}
SYMMETRY = {"brick_4x2": 180, "brick_2x2": 90}   # yaw symmetries of a box footprint


@dataclass
class Layout:
    # yaml files represent two world coords: pick and assembly coordinates.
    # ie the initial positions are relative to some arbitray pick coordinate zero-point.
    # same for assembly positions which are relative to their own coordinates.

    # thus we just define some zero-points for pick and assembly coordinates in our TAMPanda env coordinate system.
    # chosen so that arm can actually reach the positions.
    asm_center: tuple = (0.45, 0.45)
    pick_y: float = 0.27
    pick_x0: float = 0.25
    pick_spacing: float = 0.10  # how far blocks are apart in initial position (see analysis this is pretty accurate)
    table_pos: tuple = (0.0, 0.4, 0.0)
    table_quat: tuple = (0.0, 0.0, 0.0, 1.0)
    grasp_table_clearance: float = 0.004   # GraspPlanner otherwise rejects anything shorter than about 3.5cm (which our bricks are)
    place_clearance: float = 0.008         # default 3mm presses the brick, and kicks it (thus we release the object this far above the target)


def qz(deg):
    a = math.radians(deg) / 2
    return np.array([math.cos(a), 0.0, 0.0, math.sin(a)])

def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])

def yaw_of(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

def rotz(v, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([c*v[0] - s*v[1], s*v[0] + c*v[1], v[2]])

def wrap(deg):
    return (deg + 180) % 360 - 180


class SimpleLegoScene:
    """
    Builds: 
        - TAMPanda environment (table + bricks) from Brick representatioin (see yaml_loader.Brick)
        - Executor for motion planning (RRTStar, GraspPlanner, PickPlaceExecutor)
            - for PickPlaceExecutor see (https://snoato.github.io/TAMPanda/tutorial.html).
    """
    def __init__(self, bricks: list[Brick], template_dir="block_templates", layout=Layout(), rate=200.0):
        # bricks we get from yaml_loader.load_task
        self.bricks = {b.name: b for b in bricks}
        self.layout = layout

        # the builder like in the exercises but now with brick templates
        b = ArmSceneBuilder()
        b.add_resource("table", TABLE_SYMBOLIC_TEMPLATE)
        for t in ("brick_2x2", "brick_4x2"):
            b.add_resource(t, f"{template_dir}/{t}.xml")
        b.add_object("table", name="table", pos=list(layout.table_pos), quat=list(layout.table_quat))

        # this is just so we can get table_z (so we know where to place bricks)
        probe = ArmSceneBuilder()
        probe.add_resource("table", TABLE_SYMBOLIC_TEMPLATE)
        probe.add_object("table", name="table", pos=list(layout.table_pos), quat=list(layout.table_quat))
        penv = probe.build_env(rate=rate); penv.forward()
        self.table_z = self._table_top(penv)

        # NOTE: initial positions are relative to some world coordinate
        # important is just having positions not exactly where they are (no obstruction anyway)
        # so we just define initial positions such that feasible for TAMPanda arm
        # (symbolic planner doesnt care anyway)
        self.pick_pose = {}
        for i, br in enumerate(bricks):
            pos = np.array([layout.pick_x0 + i * layout.pick_spacing, layout.pick_y,
                            self.table_z + BRICK_HALF_HEIGHT])
            yaw = br.initial_yaw or 0
            self.pick_pose[br.name] = (pos, yaw)
            b.add_object(br.type, name=br.name, pos=pos.tolist(), quat=qz(yaw).tolist(),
                         rgba=COLORS.get(br.color, [0.6, 0.6, 0.6, 1]))

        # set up dict of each bricks target position using the IR
        # note that IR target pos is relative to some arbitrary assembly world coordinate
        # thus, like initial, we set assembly coordinate zero so arm can still reach but keep relative positions
        c = np.mean([br.center[:2] for br in bricks], axis=0)
        self.target_pose = {}
        for br in bricks:
            xy = np.array(br.center[:2]) - c + np.array(layout.asm_center)
            z = self.table_z + BRICK_HALF_HEIGHT + br.layer * LAYER_PITCH
            self.target_pose[br.name] = (np.array([xy[0], xy[1], z]), br.yaw)
 
        self.env = b.build_env(rate=rate)
        self.env.forward()
        self.env.rest(0.5)

        # set up executor with attachment to arm
        # NOTE: see Layout class for table_clearance 
        self.planner = RRTStar(self.env)
        self.grasp_planner = GraspPlanner(table_z=self.table_z, table_clearance=layout.grasp_table_clearance)
        self.executor = PickPlaceExecutor(self.env, self.planner, self.grasp_planner, use_attachment=True)
        self._held = None  # tracks what brick and how we are holding: (brick_name, grip_offset_vector, grasp_quat)

    @staticmethod
    def _table_top(env):
        """get z-position of table from env"""
        import mujoco
        gid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "table_surface")
        return float(env.data.geom_xpos[gid][2] + env.model.geom_size[gid][2])

    def ee_pos(self):
        """get position of end executor"""
        return self.env.data.site_xpos[self.executor._ee_site_id].copy()

    def check_goal(self, tol_xy=0.006, tol_z=0.006, tol_yaw=6.0):
        """compare env position of bricks to target positions"""
        report = {}  # keeps track of how far we are apart from target coords
        for name, (pos, yaw) in self.target_pose.items():
            br = self.bricks[name]
            p = self.env.get_object_position(name)
            dyaw = abs(wrap(yaw_of(self.env.get_object_orientation(name)) - yaw)) % SYMMETRY[br.type]
            dyaw = min(dyaw, SYMMETRY[br.type] - dyaw)
            ok = (np.linalg.norm(p[:2] - pos[:2]) < tol_xy and abs(p[2] - pos[2]) < tol_z and dyaw < tol_yaw)
            report[name] = (np.linalg.norm(p[:2] - pos[:2]), abs(p[2] - pos[2]), dyaw, ok)
        return all(r[3] for r in report.values()), report


def pddl_name(name):
    """unified-planning PDDL reader rejects object names starting with a digit"""
    return name if name[0].isalpha() else f"b_{name}"


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


def make_bridge_for_v2(domain_path, scene: SimpleLegoScene):
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
    def exec_pick(env, fluents, b):
        # see https://snoato.github.io/TAMPanda/tutorial.html#pickplace
        b = real_name[b]
        pos, half, quat = env.get_object_position(b), env.get_object_half_size(b), env.get_object_orientation(b)
        cands = scene.grasp_planner.generate_candidates(pos, half, quat)
        if not ex.pick(b, pos, half, quat, candidates=cands):
            return False, {}
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

def build_objects_and_goal_for_v2(bricks: list[Brick]):
    """Set up all objects, predicates and goal conditions for bridge (NOTE that this is for lego_coarse_simple.pddl domain)"""
    parent, children, multi = support_tree(bricks)
    for n, names in multi.items():
        print(f"NOTE: {n} rests on {names}; keeping only {parent[n].name} (see bridge limitation)")
    roots = [b for b in bricks if b.name not in parent]

    objects = {"brick": [pddl_name(b.name) for b in bricks]}
    goals = [("at-target", pddl_name(b.name)) for b in bricks]
    return objects, goals

def execute_plan(bridge, plan):
    # blind plan execution (not TAMP)
    for i, (name, args) in enumerate(plan):
        ok, _ = bridge.execute_action(name, *args)
        print(f"[{i+1}/{len(plan)}] {name}({', '.join(args)}) -> {'ok' if ok else 'FAILED'}")
        if not ok:
            return False, i
    return True, len(plan)