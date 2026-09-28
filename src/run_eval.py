"""Execute PDDL plans and ABD orders through the same TAMPanda executor
basically the same as for the notebooks but with CLI interface to execute in code


Run from src/:
  python run_eval.py --tiers 1 2 --every 5 --seed 42 --out ../results/execution_seed42.csv
"""
import argparse, contextlib, csv, glob, io, time
from pathlib import Path
from unified_planning.shortcuts import get_environment
get_environment().credits_stream = None

from yaml_loader import load_task
from scenes import SimpleLegoScene
from bridges import LegoCoarseSimpleV2SLSWeldBridge as Bridge, execute_plan
from eval_baseline import abd_plan, pddl_plan, abd_to_actions, measure

ap = argparse.ArgumentParser()
ap.add_argument("--tiers", nargs="+", type=int, default=[1, 2])
ap.add_argument("--every", type=int, default=5)
ap.add_argument("--seed", type=int, default=42)
ap.add_argument("--domain", default="domains/lego_coarse_simple_v2.pddl")
ap.add_argument("--out", default="../results/execution_seed42.csv")
args = ap.parse_args()

FIELDS = ["tier", "task", "planner", "seed", "n_bricks", "plan_len", "executed", "failed_step",
          "success", "placed", "total", "placed_frac", "welded", "welded_frac", "sim_time_s", "wall_s"]
out = Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
done = set()
if out.exists():
    with out.open() as f:
        done = {(r["task"], r["planner"]) for r in csv.DictReader(f)}
else:
    with out.open("w", newline="") as f:
        csv.DictWriter(f, fieldnames=FIELDS).writeheader()


def run(scene, bridge, plan):
    t0 = time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):          # silence the executor
        ok, n = execute_plan(bridge, plan)
    scene.env.rest(1.0)
    m = measure(scene)
    m.update({"plan_len": len(plan), "executed": ok, "failed_step": None if ok else n,
              "placed_frac": round(m["placed"] / m["total"], 3), "welded_frac": round(m["welded"] / m["total"], 3),
              "wall_s": round(time.perf_counter() - t0, 1)})
    return m


tasks = [f for t in args.tiers for f in sorted(glob.glob(f"dataset/ground_truth/tier{t}/task_*.yaml"))[::args.every]]
for f in tasks:
    tier, name = int(f.split("tier")[1][0]), Path(f).stem
    if (name, "pddl") in done and (name, "abd") in done:
        continue
    bricks, _ = load_task(f, f)
    scene = SimpleLegoScene(bricks, seed=args.seed)
    scene.env.ik.solver = "daqp"
    base = {"tier": tier, "task": name, "seed": args.seed, "n_bricks": len(bricks)}
    results = []

    if (name, "pddl") not in done:
        p = pddl_plan(bricks, scene, Bridge, args.domain)
        r = run(scene, p["bridge"], p["plan"]) if p["plan"] else {"plan_len": 0, "executed": False, "success": False}
        results.append(dict(base, planner="pddl", **r))
        scene.env.reset(); scene.env.rest(1.0); scene._held = None     # reset also clears the welds

    if (name, "abd") not in done:
        bridge = Bridge.make_bridge(args.domain, scene)               # fresh fluents
        a = abd_plan(bricks, name)
        r = run(scene, bridge, abd_to_actions(a["order"], bricks))
        results.append(dict(base, planner="abd", **r))

    with out.open("a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        for r in results:
            w.writerow({k: r.get(k) for k in FIELDS})
            print({k: r.get(k) for k in ("task", "planner", "executed", "success", "placed", "total", "wall_s")}, flush=True)
    scene.env.close()