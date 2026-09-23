"""Run several tasks and seeds, one CSV row each.

  python -m lego_tampanda.run_batch OUT.csv SEEDS PRODUCT [PRODUCT ...]
  e.g.  python -m lego_tampanda.run_batch t1.csv 0,1,2 off_tier1_task_001
"""
import csv, sys, traceback
from .run_task import run

out, seeds, products = sys.argv[1], [int(s) for s in sys.argv[2].split(",")], sys.argv[3:]
fields = ["product", "seed", "domain", "planned", "plan_length", "planning_time_s", "executed",
          "failed_step", "latch", "placed", "total", "success"]
with open(out, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fields)
    writer.writeheader()
    for product in products:
        for seed in seeds:
            try:
                summary, _ = run(product, seed, verbose=False)
            except Exception:
                traceback.print_exc()
                summary = {"product": product, "seed": seed, "failed_step": "EXCEPTION"}
            writer.writerow(summary); f.flush()
            print({k: summary.get(k) for k in ("product", "seed", "plan_length", "success", "placed", "total", "failed_step")}, flush=True)
