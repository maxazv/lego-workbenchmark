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