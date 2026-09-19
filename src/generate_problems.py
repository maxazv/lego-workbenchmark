"""python generate_problems.py <dataset_root> tier1 coarse|granular"""
import sys, glob, os
from yaml_loader import load_task
import emit_coarse, emit_granular

dataset_root, tier, kind = sys.argv[1:4]
out_dir = f"problems_{kind}/{tier}"
os.makedirs(out_dir, exist_ok=True)
for path in sorted(glob.glob(f"{dataset_root}/ground_truth/{tier}/task_*.yaml")):
    task_id = os.path.splitext(os.path.basename(path))[0]
    bricks, lat = load_task(path, path)
    pddl = (emit_coarse.emit_problem(bricks, f"{tier}_{task_id}") if kind == "coarse" else
            emit_granular.emit_problem(bricks, lat, f"{tier}_{task_id}"))
    open(f"{out_dir}/{task_id}.pddl", "w").write(pddl)
    print(f"wrote {out_dir}/{task_id}.pddl ({len(bricks)} bricks)")