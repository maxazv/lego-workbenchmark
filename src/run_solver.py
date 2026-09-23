import subprocess, re, tempfile, time
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

@dataclass
class FDResult:
    solved: bool
    unsolvable: bool     # FD proved no plan exists (different from timed_out/crashed)
    timed_out: bool
    wall_time: float      # measured from Python, includes process startup
    planner_time: Optional[float]   # FD's own reported time (translate+search)
    expanded_states: Optional[int]
    plan: Optional[list]
    plan_length: Optional[int]
    exit_code: Optional[int]
    raw_output: Optional[str]        # kept for when something unexpected happens

def run_fast_downward(fd_script, domain_path, problem_path, search="astar(blind())", timeout=120, save_raw_out=True):
    domain_path = str(Path(domain_path).resolve())
    problem_path = str(Path(problem_path).resolve())
    start = time.time()

    with tempfile.TemporaryDirectory() as tmp:
        try:
            proc = subprocess.run(
                ["timeout", str(timeout), fd_script, domain_path, problem_path,
                 "--search", search],
                cwd=tmp, capture_output=True, text=True,
            )
        except FileNotFoundError as e:
            raise RuntimeError(f"couldn't launch fast-downward.py or `timeout`: {e}")

        wall_time = time.time() - start
        out = proc.stdout + proc.stderr

        # `timeout` itself exits 124 when it had to kill the child
        timed_out = proc.returncode == 124
        solved = "Solution found!" in out
        unsolvable = "Task is provably unsolvable" in out

        m = re.search(r"Expanded (\d+) state", out)
        expanded = int(m.group(1)) if m else None
        m = re.search(r"Planner time: ([\d.]+)s", out)
        planner_time = float(m.group(1)) if m else None

        plan = None
        if solved:
            plan_files = sorted(Path(tmp).glob("sas_plan*"))
            if plan_files:
                lines = plan_files[-1].read_text().strip().splitlines()
                plan = [l for l in lines if not l.startswith(";")]

        return FDResult(
            solved=solved, unsolvable=unsolvable, timed_out=timed_out,
            wall_time=wall_time, planner_time=planner_time,
            expanded_states=expanded, plan=plan,
            plan_length=len(plan) if plan is not None else None,
            exit_code=proc.returncode, raw_output=out if save_raw_out else None,
        )