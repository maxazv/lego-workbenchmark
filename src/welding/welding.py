from __future__ import annotations

import itertools
import os
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def _bid(model, name: str) -> int:
    i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if i < 0:
        raise ValueError(f"body '{name}' not found")
    return i


def table_body_name(env, table_geom: str = "table_surface") -> str:
    """Body that owns the table's top geom (parent for bottom-layer bricks)."""
    m = env.model
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, table_geom)
    if g < 0:
        raise ValueError(f"geom '{table_geom}' not found")
    return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])


def relative_pose(data, parent_id: int, child_id: int):
    """Pose of child in parent frame: (pos[3], quat_wxyz[4])."""
    rp = data.xmat[parent_id].reshape(3, 3)
    rel_pos = rp.T @ (data.xpos[child_id] - data.xpos[parent_id])
    inv = np.zeros(4)
    mujoco.mju_negQuat(inv, data.xquat[parent_id])
    rel_quat = np.zeros(4)
    mujoco.mju_mulQuat(rel_quat, inv, data.xquat[child_id])
    return rel_pos, rel_quat


class KinematicClutch:
    """lego_sim-style fake weld: teleport children to their stored relative pose each step.

    Installs itself by wrapping env.step on this env instance, so env.rest(),
    env.wait_idle() and everything else that steps the sim picks it up.
    """

    def __init__(self, env):
        self.env = env
        self.welds: list[dict] = []
        original_step = env.step

        def step_with_clutch():
            self.maintain()
            original_step()

        env.step = step_with_clutch

    def engage(self, child: str, parent: str) -> None:
        m, d = self.env.model, self.env.data
        mujoco.mj_kinematics(m, d)
        pid, cid = _bid(m, parent), _bid(m, child)
        rel_pos, rel_quat = relative_pose(d, pid, cid)
        jnt = m.body_jntadr[cid]
        self.release(child)
        self.welds.append({"parent": parent, "child": child, "pid": pid,
                           "qadr": m.jnt_qposadr[jnt], "vadr": m.jnt_dofadr[jnt],
                           "rel_pos": rel_pos, "rel_quat": rel_quat})

    def maintain(self) -> None:
        if not self.welds:
            return
        d = self.env.data
        # parents before children so chains (table -> b0 -> b1 -> b2) resolve in one pass
        for w in self.welds:
            rp = d.xmat[w["pid"]].reshape(3, 3)
            q = np.zeros(4)
            mujoco.mju_mulQuat(q, d.xquat[w["pid"]], w["rel_quat"])
            d.qpos[w["qadr"]:w["qadr"] + 3] = d.xpos[w["pid"]] + rp @ w["rel_pos"]
            d.qpos[w["qadr"] + 3:w["qadr"] + 7] = q
            d.qvel[w["vadr"]:w["vadr"] + 6] = 0.0
            mujoco.mj_kinematics(self.env.model, d)   # update xpos for the next link in the chain

    def release(self, child: str) -> None:
        self.welds = [w for w in self.welds if w["child"] != child]

    def release_all(self) -> None:
        self.welds = []

    def engaged(self) -> list[tuple[str, str]]:
        return [(w["parent"], w["child"]) for w in self.welds]




# geometry helpers
def support_below(env, child: str, candidates: list[str], table: str | None = None,
                  z_tol: float = 0.004, xy_margin: float = 0.001) -> str | None:
    """The body `child` currently rests on: highest candidate whose top touches child's
    bottom and whose XY footprint overlaps child's. Falls back to `table` if child's
    bottom is at table height. Axis-aligned boxes only (fine for BlocksWorld cubes)."""
    pc = env.get_object_position(child)
    hc = env.get_object_half_size(child)
    best, best_z = None, -np.inf
    for c in candidates:
        if c == child:
            continue
        p, h = env.get_object_position(c), env.get_object_half_size(c)
        touching = abs((p[2] + h[2]) - (pc[2] - hc[2])) < z_tol
        overlap = (abs(p[0] - pc[0]) < h[0] + hc[0] - xy_margin and
                   abs(p[1] - pc[1]) < h[1] + hc[1] - xy_margin)
        if touching and overlap and p[2] > best_z:
            best, best_z = c, p[2]
    return best if best is not None else table


def ee_goal_for_held(env, target_pos, seat: float = 0.0) -> np.ndarray:
    """EE position that puts the *held* object's centre at target_pos.

    Uses the in-hand offset that FrankaEnvironment.attach_object_to_ee recorded, so it
    is exact regardless of where on the object the grasp landed. Assumes the EE keeps
    the orientation it has now (true when you place with the grasp's quaternion).
    `seat` > 0 lowers the goal by that many metres (small press into the support).
    """
    if env._attached is None:
        raise RuntimeError("nothing attached to the EE")
    env.forward()
    ee_mat = env.data.site_xmat[env._ee_site_id].reshape(3, 3)
    goal = np.asarray(target_pos, float) - ee_mat @ env._attached["rel_pos"]
    goal[2] -= seat
    return goal
