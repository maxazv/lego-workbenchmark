"""Plan one WorkBenchMark task with PDDL and execute the plan in TAMPanda.

  python -m lego_tampanda.run_task PRODUCT [SEED] [--domain FILE] [--view] [--no-latch]

PRODUCT is a YAML path or a name found in lego_task.PRODUCTS, e.g. tower5,
off_tier1_task_001, off_tier2_task_001.
"""
import argparse, contextlib, io, json, tempfile, time

import numpy as np
from unified_planning.shortcuts import get_environment

from tampanda import RRTStar, GraspPlanner
from tampanda.planners.pick_place import PickPlaceExecutor
from .lego_bridge import DEFAULT_DOMAIN, make_lego_bridge, pose_error, XY_TOL, Z_TOL, YAW_TOL
from .lego_scene import make_lego_builder, spawn_bricks, table_top_z
from .lego_task import load_product

get_environment().credits_stream = None


def run(product_name, seed=0, domain=DEFAULT_DOMAIN, view=False, verbose=True, latch=True):
    product = load_product(product_name)
    env = make_lego_builder(product["blocks"]).build_env(rate=200.0)
    table_z = table_top_z(env)
    spawn_bricks(env, product, seed)

    planner = RRTStar(env)
    planner.max_iterations, planner.step_size, planner.goal_sample_rate = 3000, 0.2, 0.2
    # table_clearance: the 0.025 default rejects a 19 mm brick lying on the table.
    executor = PickPlaceExecutor(env, planner, GraspPlanner(table_z=table_z, table_clearance=0.003),
                                 use_attachment=True)
    bridge, objects, goals = make_lego_bridge(env, product, executor, domain, latch=latch)

    start = time.perf_counter()
    # Fast Downward drops output.sas into the cwd; keep parallel runs apart.
    with tempfile.TemporaryDirectory() as tmp, contextlib.chdir(tmp):
        plan = bridge.plan(objects, goals, planner_name="fast-downward")
    plan_time = time.perf_counter() - start
    summary = {"product": product["product"]["name"], "seed": seed, "planned": plan is not None,
               "plan_length": len(plan or []), "planning_time_s": round(plan_time, 3),
               "executed": False, "failed_step": None, "latch": latch}

    def execute():
        for action, params in plan:
            with contextlib.redirect_stdout(io.StringIO()):      # executor chatter
                ok, _ = bridge.execute_action(action, *params)
            if verbose:
                print(f"  {'ok  ' if ok else 'FAIL'} ({action} {' '.join(params)})")
            if not ok:
                summary["failed_step"] = f"({action} {' '.join(params)})"
                return
        summary["executed"] = True

    if plan is not None:
        if verbose:
            print(f"plan: {len(plan)} actions, planned in {plan_time:.2f}s")
        if view:
            with env.launch_viewer():
                execute()
        else:
            execute()

    rows, placed = [], 0
    for block in product["blocks"]:
        xy, z, yaw = pose_error(env, block, table_z)
        ok = bool(xy < XY_TOL and abs(z) < Z_TOL and abs(yaw) < YAW_TOL)
        placed += ok
        rows.append(f"  {block['id']:>16}: ok={ok} xy={1e3*xy:.2f}mm z={1e3*z:+.2f}mm yaw={np.degrees(yaw):+.1f}deg")
    summary.update({"placed": placed, "total": len(rows), "success": placed == len(rows)})
    env.close()
    return summary, rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("product")
    ap.add_argument("seed", nargs="?", type=int, default=0)
    ap.add_argument("--domain", default=DEFAULT_DOMAIN)
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--no-latch", action="store_true", help="plain boxes, no stud latch")
    args = ap.parse_args()
    summary, rows = run(args.product, args.seed, args.domain, args.view, latch=not args.no_latch)
    print(json.dumps(summary, indent=1))
    print("\n".join(rows))
