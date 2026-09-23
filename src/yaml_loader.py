"""Shared loading + geometry layer which is independent of pddl domain. 
Emitters (pddl parsers) consume the representation this module produces, and executor side (DomainBridge action callbacks) 
should use cell_to_centre() from here so cell/metres is defined in one place.
Note we only have discrete grid for target structure. Initial positions only stored as brick centre.

Conventions:
- under the dataset README's "y is vertical" claim, 19 of 33 float in mid-air, hence we use:
   - x, y : horizontal (table plane), stud pitch 0.016m
   -  z    : vertical, layer pitch 0.0191 m  (scene_builder.py in lego_sim: 0.0192)
- YAW: yaw=0 -> a brick's long side runs along x; yaw=90 -> along y
"""
import yaml
from dataclasses import dataclass
from typing import Optional

# derived from yaml files and https://github.com/ma-haha-hehe/lego_sim/blob/main/src/mj_bridge/mj_bridge/scene_builder.py
STUD_PITCH = 0.016
LAYER_PITCH = 0.0191
STUDS = {"brick_2x2": (2, 2), "brick_4x2": (4, 2)}     # (long side, short side)
ORIENT = {0: "deg0", 90: "deg90"}


def stud_counts(brick_type: str, yaw: int) -> tuple[int, int]:
    """(# studs along x, # studs along y) for a brick at a given yaw."""
    n_long, n_short = STUDS[brick_type]
    return (n_long, n_short) if yaw == 0 else (n_short, n_long)


class Lattice:
    """Takes the yaml brick center position and convertes it into a list of its stud center positions.
    We represent environment as grid over possible stud positions, thus we convert the brick center, which doesnt
    lie on a grid cell, into the stud center positions which do lie on the grid (by def).
    
    E.g. a 4x2 brick has 4*2=8 studs. 
    """

    def __init__(self):
        self.off = None   # (off_x, off_y)

    @staticmethod
    def _stud_centres(cx, cy, brick_type, yaw):
        """Brick centre TARGET-positions to its stud positionsc.
        First create nx by ny evenly spaced points, mult by STUD_PITCH and offset by brick center to get actual coords.
        """
        nx, ny = stud_counts(brick_type, yaw)
        return [(cx + (i - (nx - 1) / 2) * STUD_PITCH,
                 cy + (j - (ny - 1) / 2) * STUD_PITCH)
                for i in range(nx) for j in range(ny)]

    def calibrate(self, bricks_raw):
        """Safety check: each stud position in metres has same offset, ie x-/y-position inside their grid cell.
        Ie we don't want misaligned studs, all stud centres should lie on evenly spaced lattice for each axis.
        """

        studs = [s for b in bricks_raw
                 for s in self._stud_centres(b["pos"][0], b["pos"][1], b["type"], b["rotation"][2])]
        # print(studs)
        offs = []
        for axis in (0, 1):
            residues = sorted(s[axis] % STUD_PITCH for s in studs)  # collect all offsets of studs wrt their cell on axis
            ref = residues[0]  # take some random stud as reference
            for r in residues:
                d = min(abs(r - ref), STUD_PITCH - abs(r - ref))
                assert d < 1e-6, (
                    f"stud centres are not on one lattice along axis {axis} "
                    f"(residues {ref:.5f} vs {r:.5f}): axis or yaw convention is wrong for this task")
            offs.append(ref)
        self.off = tuple(offs)

    def cells(self, cx, cy, brick_type, yaw) -> frozenset:
        """Stud positions (in metres) to integer grid coords."""
        return frozenset(
            (round((sx - self.off[0]) / STUD_PITCH), round((sy - self.off[1]) / STUD_PITCH))
            for sx, sy in self._stud_centres(cx, cy, brick_type, yaw))

    def cell_to_centre(self, anchor, layer, brick_type, yaw):
        """Inverse mapping for executor: from grid positions to brick centre (x, y, z)."""
        nx, ny = stud_counts(brick_type, yaw)
        ix, iy = anchor
        x = self.off[0] + (ix + (nx - 1) / 2) * STUD_PITCH
        y = self.off[1] + (iy + (ny - 1) / 2) * STUD_PITCH
        return (x, y, layer * LAYER_PITCH)


@dataclass
class Brick:
    """Representation of single brick that each domain emitter uses to build problem"""
    name: str
    type: str
    color: str
    layer: int                 # target layer (0 = on the base plate)
    yaw: int                   # 0 or 90
    cells: frozenset           # target stud cells {(ix, iy)} on the lattice
    anchor: tuple              # (ix, iy): min-corner stud cell (used as the granular `at` anchor)
    center: tuple              # target centre (x, y, z), metres, assembly-plate frame
    initial: Optional[tuple] = None      # initial centre (x, y, z), metres, WORLD frame
    initial_yaw: Optional[int] = None


def load_task(benchmark_path, ground_truth_path=None):
    with open(benchmark_path) as f:
        target_blocks = yaml.safe_load(f)["blocks"]
    initial_by_name = {}
    if ground_truth_path:
        with open(ground_truth_path) as f:
            initial_by_name = {b["name"]: b for b in yaml.safe_load(f)["initial_blocks"]}

    lat = Lattice()
    lat.calibrate(target_blocks)  # to assert same offset and safe that offset

    bricks = []
    for b in target_blocks:
        x, y, z = b["pos"]
        yaw = int(b["rotation"][2])
        assert yaw in ORIENT, f"unexpected yaw {yaw} for {b['name']}"
        cells = lat.cells(x, y, b["type"], yaw)
        init = initial_by_name.get(b["name"])
        bricks.append(Brick(
            name=b["name"], type=b["type"], color=b["color"],
            layer=round(z / LAYER_PITCH), yaw=yaw, cells=cells,
            anchor=min(cells), center=(x, y, z),
            initial=tuple(init["pos"]) if init else None,
            initial_yaw=int(init["rotation"][2]) if init else None,
        ))

    # NOTE: no lattice for initial positions (initial positions not on grid)
    # lat_init = Lattice()
    # lat_init.calibrate(list(initial_by_name.values()))
    
    return bricks, lat


def supporters(bricks):
    """For each brick: list of (supporting brick, #shared stud cells) one layer below (empty for base-layer bricks). 
    A brick with >1 supporter rests on several bricks at once (bridge), note that coarse domain cannot express that).
    We compute brick support by looking at bricks one z-layer below and checking for common i, j grid coords.
    """
    by_layer = {}
    for b in bricks:
        by_layer.setdefault(b.layer, []).append(b)
    out = {}
    for b in bricks:
        sup = []
        for below in by_layer.get(b.layer - 1, []):
            # same i, j grid-position one layer apart
            shared = len(b.cells & below.cells)  # NOTE: '&' is intersection operator for sets
            if shared:
                sup.append((below, shared))
        out[b.name] = sorted(sup, key=lambda t: -t[1])
    return out


# distance matrix in #cells: ie calculate distance to each bricks cell take min cell into distance matrix

# TODO: obstructors ie horizontal obstructions
# - obstructors for both initial and target cells
# - recall TAMP regrounds and replans after each action execution
def obstructors(bricks, num_cells: int):
    # brick a obstructs brick b if distance from a to b is <= num_cells
    for b in bricks:
        # TODO
        raise NotImplementedError
