# tampanda setup

import math
from dataclasses import dataclass
import numpy as np

import mujoco
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
    work_area: tuple = (0.25, 0.65, 0.25, 0.65) # Square area "around" which we may spawn
    pick_spacing: tuple = (0.08, 0.08)
    edge_margin: float = 0.03
    asm_clearance: float = 0.06 # Avoid bridges too close to the edge to be sure
    brick_radius: float = 0.02
    arm_base: tuple = (0.0, 0.0)
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

def qconj(q):
    """Used by weld() to express a body's pose relative to another body's frame (see weld())"""
    return np.array([q[0], -q[1], -q[2], -q[3]])



class SimpleLegoScene:
    """
    Builds: 
        - TAMPanda environment (table + bricks) from Brick representatioin (see yaml_loader.Brick)
        - Executor for motion planning (RRTStar, GraspPlanner, PickPlaceExecutor)
            - for PickPlaceExecutor see (https://snoato.github.io/TAMPanda/tutorial.html).
    """
    def __init__(self, bricks: list[Brick], template_dir="block_templates", layout=Layout(), rate=200.0, seed=None):
        # variability due to RRTStar
        if seed is not None:
            np.random.seed(seed)

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

        self.pick_pose = {}
        for br, (x, y) in zip(bricks, self._spawn_slots(len(bricks))):
            pos = np.array([x, y, self.table_z + BRICK_HALF_HEIGHT])
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

        self._welds = []  # list of dicts: {child, parent, rel_pos, rel_quat} (see weld() below)
        self._install_weld_hook()

    # Spawn n bricks with respect to work area (plus clearance) but also margins
    def _spawn_slots(self, n):
        L = self.layout
        x0, x1, y0, y1 = L.work_area
        dx, dy = L.pick_spacing
        xs = np.arange(x0 + L.edge_margin, x1 - L.edge_margin + 1e-9, dx)
        ys = np.arange(y0 + L.edge_margin, y1 - L.edge_margin + 1e-9, dy)

        asm_xy = np.array([p[:2] for p, _ in self.target_pose.values()])
        min_dist = 2 * L.brick_radius + L.asm_clearance
        slots = [np.array([x, y]) for x in xs for y in ys
                 if np.min(np.linalg.norm(asm_xy - [x, y], axis=1)) >= min_dist]
        slots.sort(key=lambda s: np.linalg.norm(s - np.array(L.arm_base)))
        return slots[:n]

    def _install_weld_hook(self):
        """Rigid welding in TAMPanda. TAMPanda already does this in franka_env.py with attach_object_to_ee / _apply_attachment:
            - capture the held object's pose relative to EE once at grasp-time (offset)
            - every step thereafter teleport the object's qpos to the EE's current pose composed with fixed relative offset
                - mujoco environemnt step is wrapped with custom step where teleported

        We generalize to two arbitrary bodies in env => we wrap TAMPanda env step.
        """
        orig_step = self.env.step
        def step_with_welds():
            self._apply_welds()   # must run before mj_step
            orig_step()           # the real step(): attachment check, then mj_step, then its own bookkeeping
        self.env.step = step_with_welds

        # fix: we have to reset the welds, otherwise after reset bricks are teleported back
        orig_reset = self.env.reset
        def reset_with_welds():
            self._welds = []
            orig_reset()
        self.env.reset = reset_with_welds
 
    def _apply_welds(self):
        m, d = self.env.model, self.env.data
        mujoco.mj_kinematics(m, d)
        by_child = {w["child"]: w for w in self._welds}  # child to weld-relation mapping
        fresh = {}   # body id -> (pos, quat) written by this pass so far

        # recall ex1 forward kinematics: we have to first transform parent then apply relative transform to children
        # => bottom-up approach whcih we do recursively
        def resolve(bid):
            if bid in fresh:
                return fresh[bid]
            w = by_child.get(bid)
            if w is None:  # not welded: its current pose is the truth
                return d.xpos[bid].copy(), d.xquat[bid].copy()
            p_pos, p_quat = resolve(w["parent"])  # parent first (recall ex1)
            rotated = np.zeros(3)
            mujoco.mju_rotVecQuat(rotated, w["rel_pos"], p_quat)
            # our position in world coords (we apply the transform now)
            pos, quat = p_pos + rotated, qmul(p_quat, w["rel_quat"])
            # get indices of where our pos/orientation saved in mujoco
            j = m.body_jntadr[bid]
            qa, va = m.jnt_qposadr[j], m.jnt_dofadr[j]
            # set the position in mujoco
            d.qpos[qa:qa + 3] = pos
            d.qpos[qa + 3:qa + 7] = quat
            d.qvel[va:va + 6] = 0.0
            # track that we've calculated this brick
            fresh[bid] = (pos, quat)
            return fresh[bid]
 
        for w in self._welds:
            resolve(w["child"])
 
    def weld(self, child, parent):
        """Rigidly lock `child`'s pose to `parent`'s, from whatever their CURRENT relative pose is."""
        m, d = self.env.model, self.env.data
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, child)
        pid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, parent)

        # some safety checks
        assert cid >= 0, f"unknown body '{child}'"
        assert pid >= 0, f"unknown body '{parent}'"
        assert m.body_jntadr[cid] >= 0, f"'{child}' has no free joint (nothing to weld)"
        if cid == pid:
            raise ValueError(f"cannot weld '{child}' to itself")
        # walk up from the new parent: if we reach the child, this weld would close loop (which is meaningless)
        by_child = {w["child"]: w["parent"] for w in self._welds if w["child"] != cid}
        b = pid
        while b in by_child:
            b = by_child[b]
            if b == cid:
                raise ValueError(f"welding '{child}' to '{parent}' would create a weld cycle")

        # we get the relative pose of child wrt parent: ie what operation do we have to apply to parent to get child pose (recall ex1)
        # aka in world coords where parent has zero pos and orientation: what is child pose
        Rp = d.xmat[pid].reshape(3, 3)
        rel_pos = Rp.T @ (d.xpos[cid] - d.xpos[pid])
        rel_quat = qmul(qconj(d.xquat[pid]), d.xquat[cid])
        # save it in our welds-relation list
        self._welds = [w for w in self._welds if w["child"] != cid]   # replace any prior weld on this child
        self._welds.append({"child": cid, "parent": pid, "rel_pos": rel_pos, "rel_quat": rel_quat})
 
    def unweld(self, child, parent=None):
        """Remove any weld on `child`"""
        # NOTE: parent unnecessary as we only every weld child to single parent (for tier[1,2] at least)
        m = self.env.model
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, child)
        before = len(self._welds)
        self._welds = [w for w in self._welds if w["child"] != cid]
        return len(self._welds) < before
    
    def to_target_structure(self):
        """teleport bricks to their target pose for visualization."""
        # same principle as in _apply_welds:
        #  - get mujoco state
        #  - get brick indcies to know where pose information stored
        #  - override info with our target pose
        #  - do final mj_kinematics so positions/orientations refreshed (no forces calculated, that is job of mj_forward)
        m, d = self.env.model, self.env.data
        for name, (pos, target_yaw) in self.target_pose.items():
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
            qa = m.jnt_qposadr[m.body_jntadr[bid]]
            va = m.jnt_dofadr[m.body_jntadr[bid]]
            d.qpos[qa:qa + 3] = pos
            d.qpos[qa + 3:qa + 7] = qz(target_yaw)
            d.qvel[va:va + 6] = 0.0
        mujoco.mj_kinematics(m, d)


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