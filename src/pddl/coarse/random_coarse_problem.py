from itertools import product
import copy
import random

TYPES = ["1x1", "2x2", "4x2"]
COLORS = ["red", "blue", "green", "yellow"]

# dim = [1, 2, 4]
# for i in range(len(dim)):
#     for j in range(i, len(dim)):
#         print(f"{dim[i]}x{dim[j]}")


NUM_BLOCKS_INIT = 3
# print(random.choice(TYPES))


objs = {"brick": [], "type": [], "color": [], "location": []}

init_preds = {"shape": {}, "is_color": {}, "at": {}}

init_preds = {}

#
# define objects and init
#
for i in range(NUM_BLOCKS_INIT):

    brick_id = f"brick_{i}"
    brick_type = random.choice(TYPES)
    brick_color = random.choice(COLORS)
    brick_location = f"pick_{i}"

    objs["brick"].append(brick_id)
    objs["type"].append(brick_type)
    objs["color"].append(brick_color)
    objs["location"].append(brick_location)

    init_preds[brick_id] = {
        "shape": brick_type,
        "is_color": brick_color,
        "at": brick_location
    }


# print(objs)
print(init_preds)

# NOTE: we can have some nonsensical configurations but if we solve those, then also the sensical ones

#
# define goal
#

goal_preds = {}

num_blocks_goal = random.randint(1, NUM_BLOCKS_INIT)

# pick random blocks to be base (ie not stacked on other blocks)
num_blocks_base = random.randint(1, num_blocks_goal)
for i in range(num_blocks_base):
    brick_id = random.choice(objs["brick"])
    asm_loc = f"asm_{i}"

    goal_preds[brick_id] = {**init_preds[brick_id], "at": asm_loc}

# print(goal_preds)

# define rest (stacked on one another)

rem_candidates = init_preds.keys() - goal_preds.keys()
stack_candidates = goal_preds.keys()

for i in range(num_blocks_goal - num_blocks_base):
    brick_id = random.choice(list(rem_candidates))
    base_id = random.choice(list(stack_candidates))

    goal_preds[brick_id] = {**init_preds[brick_id], "stacked_on": base_id}

    # remove from remaining and add as candidate base to be stacked on
    rem_candidates.remove(brick_id)
    stack_candidates = stack_candidates - {base_id} | {brick_id}

print(goal_preds)

#
# building pddl string
#

problem_name = f"rand_problem_{random.randint(0, 10000)}"


objs_str = ""
for key in objs.keys():
    objs_str += "    " + " ".join(objs[key]) + f" - {key}\n"
objs_str = objs_str[:-1]  # remove last /n

init_str = "\n"
for brick_id in init_preds:
    for pred in init_preds[brick_id]:
        init_str += "    "
        init_str += f"{pred} {brick_id} {init_preds[brick_id][pred]}" 
        init_str += "\n"
    init_str += "\n"
init_str = init_str[:-2]


goal_exists_vars = " ".join(f"?b{i}" for i in range(len(goal_preds)))
goal_lines = []
for i, brick_id in enumerate(goal_preds):
    goal_brick_id = f"?b{i}"
    predicates = [
        f"({pred} {goal_brick_id} {value})"
        for pred, value in goal_preds[brick_id].items()
    ]
    goal_lines.append(" ".join(predicates))

goal_str = (
    f"    (exists ({goal_exists_vars} - brick)\n"
    f"      (and\n"
    + "".join(f"           {line}\n" for line in goal_lines)
    + "      )\n"
    f"    )"
)

pddl_problem = f"""
(define (problem {problem_name})
  (:domain lego-coarse)

  (:objects
{objs_str}
  )

  (:init
    (hand-empty)
{init_str}
  )

  (:goal
{goal_str}
  )
)
"""

print(pddl_problem)
