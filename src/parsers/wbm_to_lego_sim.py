# TODO: Parses workbenchmark yaml file into lego-sim specific yaml file

import yaml
from pathlib import Path

sourcepath = "lego-workbenchmark/src/dataset/benchmark_tasks/tier1/task_001.yaml"

with open(sourcepath) as f:
    task = yaml.safe_load(f)

out = "schema_version: 1\nproduct:\n  name: parsed_problem\nblocks:\n"

for brick in task["blocks"]:
    id = brick["name"]
    type = brick["type"]
    color = brick["color"]
    position = brick["pos"]
    yaw = brick["rotation"][2] # Only third one (index 2) is actually yaw, others apparently not relevant
    out += f"  - id: {id}\n    type: {type}\n    color: {color}\n    target: {{position: {position}, yaw_deg: {yaw}}}\n"

# Output will overwrite and create directories if necessary
output_path = Path("lego-workbenchmark/out/testparser/tier1/parsed_task_001.yaml")
output_path.parent.mkdir(parents=True, exist_ok=True)
output_path.write_text(out)