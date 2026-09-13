from itertools import product
import random

TYPES = ["1x1", "2x2", "4x2"]
COLORS = ["red", "blue", "green", "yellow"]

# dim = [1, 2, 4]
# for i in range(len(dim)):
#     for j in range(i, len(dim)):
#         print(f"{dim[i]}x{dim[j]}")


NUM_BLOCKS_GOAL = 2
NUM_BLOCKS_INIT = 3
print(random.choice(TYPES))


objs = {"bricks": [], "types": [], "colors": [], "locations": []}
preds = {"shape": [], "is_color": [], "at": []}

#
# define init
#
for i in range(NUM_BLOCKS_INIT):

    brick_id = f"brick_{i}"
    brick_type = random.choice(TYPES)
    brick_color = random.choice(COLORS)
    brick_location = f"pick_{i}"

    objs["bricks"].append(brick_id)
    objs["types"].append(brick_type)
    objs["colors"].append(brick_color)
    objs["locations"].append(brick_location)

    preds["shape"].append([brick_id, brick_type])
    preds["is_color"].append([brick_id, brick_color])
    preds["at"].append([brick_id, brick_location])


print(preds)

# NOTE: we can have some nonsensical configurations but if we solve those, then also the sensical ones

#
# define goal
#
# for i in range(NUM_BLOCKS_GOAL):
