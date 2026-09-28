"""Helpers for the PDDL-vs-ABD baseline comparison.  Import from src/ for example the notebooks

We are using: 
ABD from lego_sim (executor_planner.plan_assembly) whichi is using  pure Python.
PDDL which is our DomainBridge pipeline (scenes.SimpleLegoScene + bridges.*Bridge + Fast Downward as planner).
"""
import time

from mj_bridge.executor_planner import plan_assembly #abd planner from lego_sim
from mj_bridge.benchmark_core import load_registry #for reading lego_sim brick sizes 
from emit_coarse import support_tree 
from bridges import pddl_name # our rule that turns 4x2_bricks into pddl names (b_4x2_brick) and so on.

REGISTRY = load_registry()
ASSEMBLY_CENTRE_Y = 0.148        # dataset targets are centred on y = 0.148; since lego_sim wants plate-centred


def strip_prefix(name):
    """bridges.pddl_name adds 'b_' to names starting with a digit; undo it."""
    return name[2:] if name.startswith("b_") else name


def bricks_to_product(bricks, name="task"):
    """yaml_loader.Brick list -> lego_sim product dict (schema v1)."""
    blocks = [{"id": b.name, "type": b.type, "color": b.color,
               "target": {"position": [float(b.center[0]), float(b.center[1]) - ASSEMBLY_CENTRE_Y, float(b.center[2])],
                          "yaw_deg": float(b.yaw)}}
              for b in bricks]
    return {"schema_version": 1, "product": {"name": name}, "blocks": blocks}


def abd_plan(bricks, name="task"):
    """Runs the baseline, which converts, calls plan-assemmbly stops the clock and returns ABD plan order and time."""
    t0 = time.perf_counter()
    steps = plan_assembly(bricks_to_product(bricks, name), REGISTRY)["steps"]
    return {"order": [s["id"] for s in steps], "steps": len(steps),
            "time_ms": 1000.0 * (time.perf_counter() - t0)}


def pddl_plan(bricks, scene, Bridge, domain, planner="fast-downward"):
    """OUR PLANNER: convert bricks to a scene, make a bridge, plan with Fast Downward, return plan  and time."""
    bridge = Bridge.make_bridge(domain, scene)
    objects, goals = Bridge.build_objects_and_goal(bricks)
    t0 = time.perf_counter()
    plan = bridge.plan(objects, goals, planner_name=planner)
    dt = time.perf_counter() - t0
    order = [strip_prefix(params[0]) for action, params in plan if action == "pick"] if plan else []
    return {"bridge": bridge, "plan": plan, "solved": plan is not None,
            "pairs": (len(plan) // 2) if plan else None, "time_s": dt, "order": order}


def compare_orders(pddl_order, abd_order, bricks):
    """valid: every brick after its supporter.  equal: same (type, colour, layer) sequence as ABD,
    so two interchangeable bricks of the same kind do not count as a difference."""
    parent, _, _ = support_tree(bricks)
    pos = {n: i for i, n in enumerate(pddl_order)}
    valid = bool(pddl_order) and all(pos[c] > pos[p.name] for c, p in parent.items())
    role = {b.name: (b.type, b.color, b.layer) for b in bricks}
    equal = [role[n] for n in pddl_order] == [role[n] for n in abd_order]
    return {"valid": valid, "equal": equal}


def abd_to_actions(abd_order, bricks):
    """ABD order -> actions the v2 bridge executes: pick, then place (root) or stack (on parent)."""
    parent, _, _ = support_tree(bricks)
    actions = []
    for n in abd_order:
        actions.append(("pick", (pddl_name(n),)))
        if n in parent:
            actions.append(("stack", (pddl_name(n), pddl_name(parent[n].name))))
        else:
            actions.append(("place", (pddl_name(n),)))
    return actions


def measure(scene):
    """Benchmark measurement after execution."""
    ok, report = scene.check_goal()
    placed = sum(1 for r in report.values() if r[3])
    return {"success": bool(ok), "placed": placed, "total": len(report),
            "welded": len(scene._welds), "sim_time_s": float(scene.env.data.time)}