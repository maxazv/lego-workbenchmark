# TODO: parses yaml file into pddl problem file

import yaml

with open("dataset/benchmark_tasks/tier1/task_001.yaml") as f:
    task = yaml.safe_load(f)

for brick in task["blocks"]:
    print(brick["name"], brick["type"], brick["color"], brick["pos"], brick["rotation"])