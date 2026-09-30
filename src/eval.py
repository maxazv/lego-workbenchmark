"""Run a bridge (plan + physical execution in MuJoCo), spread over N worker processes, and collect the results.

    python eval.py --tiers tier3 --workers 4
    python eval.py --tiers tier3 tier4 --workers 4
    python eval.py --tasks task_001 task_012 task_082 --workers 3
    python eval.py --tiers tier3 --limit 20 --workers 8 --out sweep.csv
    python eval.py --tiers tier3 --bridge slots

Each task runs in its own process (a MuJoCo sim isn't something you want to
share across threads), returns a small result dict, and the main process
writes everything to a CSV plus a one-line pass/fail summary at the end.
Uses the "spawn" start method rather than the Linux default "fork" which is 
safer around native libraries like MuJoCo that a fork can leave in a bad state.

This is meant to stay a quick smoke test / sweep: did anything break, which
tasks a bridge still can't handle. For analysis, continue with notebook.

To add a bridge, import it below and add a line to BRIDGES: every bridge is
assumed to have make_bridge(domain_path, scene, strict_preconditions=...) 
and build_objects_and_goal(bricks).
"""
import argparse
import csv
import glob
import multiprocessing as mp
import os
import sys
import time
import warnings
from concurrent.futures import ProcessPoolExecutor, as_completed
import contextlib

from bridges import (
    LegoCoarseSimpleSLSBridge, LegoCoarseSimpleV2SLSBridge, LegoCoarseSimpleV2SLSWeldBridge,
    LegoBeyondTier2SlotsSLSWeldBridge
)

DATASET = "dataset/ground_truth"
DOMAIN_FOLDER = "domains"

# name -> (bridge namespace, default domain .pddl file)
BRIDGES = {
    "simple": (LegoCoarseSimpleSLSBridge, "lego_coarse_simple.pddl"),
    "simple_v2": (LegoCoarseSimpleV2SLSBridge, "lego_coarse_simple_v2.pddl"),
    "simple_v2_welds": (LegoCoarseSimpleV2SLSWeldBridge, "lego_coarse_simple_v2.pddl"),
    "slots": (LegoBeyondTier2SlotsSLSWeldBridge, "lego_beyond_tier2_slots.pddl"),
}



def run_one(path, bridge_key, domain, strict, seed, quiet=True):
    """Runs single task inside a worker process."""

    # we import everything here so each process builds its own MuJoCo / unified-planning state rather than inheriting parent's.
    warnings.filterwarnings("ignore")
    from unified_planning.shortcuts import get_environment
    get_environment().credits_stream = None
    from yaml_loader import load_task
    from scenes import SimpleLegoScene

    # build results dict
    task = os.path.splitext(os.path.basename(path))[0]
    tier = os.path.dirname(path).split("/")[-1]
    result = dict(
        tier=tier, task=task, n_bricks=None, 
        planned=False, plan_time_s=None, plan_len=None, 
        executed=False, exec_time_s=None, goal_ok=None, max_dxy_mm=None, max_dyaw_deg=None, error=None
    )

    # context-handler so we can redirect annoying prints
    with contextlib.ExitStack() as stack:
        if quiet:
            devnull = stack.enter_context(open(os.devnull, "w"))
            stack.enter_context(contextlib.redirect_stdout(devnull))
            stack.enter_context(contextlib.redirect_stderr(devnull))
        try:
            ns, _ = BRIDGES[bridge_key]  # get bridge builder based on key user provided

            # same pipeline as in notebooks
            bricks, _ = load_task(path)
            result["n_bricks"] = len(bricks)
            scene = SimpleLegoScene(bricks, seed=seed)
            scene.env.ik.solver = "daqp"
            bridge = ns.make_bridge(domain, scene)
            objects, goals = ns.build_objects_and_goal(bricks)

            # symbolic planning
            t0 = time.time()
            plan = bridge.plan(objects, goals)
            result["plan_time_s"] = round(time.time() - t0, 2)
            if plan is None:
                result["error"] = "no plan found"
                return result
            result["planned"] = True
            result["plan_len"] = len(plan)

            # DomainBridge execution loop
            t0 = time.time()
            for name, args in plan:
                ok, _ = bridge.execute_action(name, *args, objects=objects)
                if not ok:
                    result["error"] = f"{name}({','.join(args)}) failed"
                    break
            result["exec_time_s"] = round(time.time() - t0, 1)
            result["executed"] = result["error"] is None

            # check if goal reached
            scene.env.rest(1.0)
            done, report = scene.check_goal()
            result["goal_ok"] = done
            if report:
                result["max_dxy_mm"] = round(max(dxy for dxy, _, _, _ in report.values()) * 1000, 2)
                result["max_dyaw_deg"] = round(max(dyaw for _, _, dyaw, _ in report.values()), 2)
        except Exception as e:
            result["error"] = f"{type(e).__name__}: {e}"
    return result



def collect_paths(tiers, tasks, limit):
    # handles task selection based on argparse flags
    if tasks:
        paths = []
        for t in tasks:
            if t.endswith(".yaml"):
                hits = glob.glob(t)                                        # raw path/glob
            elif "/" in t:
                hits = glob.glob(f"{DATASET}/{t}.yaml")                    # "tier4/task_051"
            else:
                hits = [p for tier in tiers for p in glob.glob(f"{DATASET}/{tier}/{t}.yaml")]
                if len(hits) > 1:                                          # task_NNN repeats across tiers
                    sys.exit(f"{t!r} matches more than one of {tiers}; qualify it, "
                              f"e.g. {tiers[0]}/{t}")
            if not hits:
                sys.exit(f"couldn't find a task matching {t!r} in {tiers}: "
                          f"use <tier>/{t} or a full path for a tier not in --tiers")
            paths += hits
    else:
        paths = []
        for tier in tiers:
            found = sorted(glob.glob(f"{DATASET}/{tier}/*.yaml"))
            if not found:
                sys.exit(f"no tasks found under {DATASET}/{tier}")
            paths += found
    return paths[:limit] if limit else paths


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tiers", nargs="+", default=["tier3"], help="one or more benchmark_tasks/<tier> to sweep entirely (default: tier3)")
    ap.add_argument("--tasks", nargs="*", help="explicit task names (e.g. task_012) instead of whole tiers")
    ap.add_argument("--limit", type=int, help="only run the first N tasks")
    ap.add_argument("--bridge", choices=list(BRIDGES), default="slots")
    ap.add_argument("--domain", help=f"override the bridge's default domain .pddl file")
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--seed", type=int, default=0, help="passed to every SimpleLegoScene (same seed for all, per-task RNG use differs)")
    ap.add_argument("--no-strict", action="store_true", help="skip re-checking PDDL preconditions during execution (faster, less safe)")
    ap.add_argument("--verbose-workers", action="store_true", help="don't silence tampanda/pyperplan's own prints in each worker")
    ap.add_argument("--out", default="parallel_results.csv")
    args = ap.parse_args()

    # get domain and task-paths user wants to eval
    domain = args.domain or BRIDGES[args.bridge][1]
    paths = collect_paths(args.tiers, args.tasks, args.limit)
    print(f"running {len(paths)} task(s) across {args.workers} worker(s), bridge={args.bridge} ({domain})...")

    # add tasks to pool and let worker-PROCESSES pick and execute in parallel
    results = []
    t_start = time.time()
    ctx = mp.get_context("spawn")
    domain_path = f"{DOMAIN_FOLDER}/{domain}"
    with ProcessPoolExecutor(max_workers=args.workers, mp_context=ctx) as pool:
        futs = {pool.submit(run_one, p, args.bridge, domain_path, not args.no_strict, args.seed, not args.verbose_workers): p for p in paths}
        for i, fut in enumerate(as_completed(futs), 1):
            r = fut.result()
            results.append(r)
            status = "OK" if r["goal_ok"] else ("PLAN-FAIL" if not r["planned"] else "EXEC-FAIL")
            print(f"[{i}/{len(paths)}] {r['tier'] + '/' + r['task']:20s} {status:10s} ({r['n_bricks']} bricks)  {r['error'] or ''}")

    # write results to csv
    results.sort(key=lambda r: r["task"])
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)

    # short summary print
    n_ok = sum(1 for r in results if r["goal_ok"])
    n_plan_fail = sum(1 for r in results if not r["planned"])
    n_exec_fail = sum(1 for r in results if r["planned"] and not r["executed"])
    print(f"\n{n_ok}/{len(results)} reached the goal "
          f"({n_plan_fail} failed to plan, {n_exec_fail} planned but failed to execute) "
          f"in {time.time() - t_start:.0f}s wall (saved to {args.out})")


if __name__ == "__main__":
    main()