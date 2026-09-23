"""Task side: load a product YAML (lego_sim schema v1; ``products/`` holds 16 official
WorkBenchMark tasks converted to it) and derive what the PDDL problem needs from it."""
import re
from pathlib import Path

import numpy as np
import yaml

from .lego_scene import BRICK_SIZE

PRODUCTS = [Path(__file__).parent / "products"]

# Where the product's (0, 0, 0) sits on the TAMPanda table (robot base = world origin).
ASSEMBLY_ORIGIN = np.array([0.45, 0.50])


def pddl_name(block_id: str) -> str:
    name = re.sub(r"[^a-z0-9_]", "_", block_id.lower())
    return name if name[0].isalpha() else "b_" + name


def load_product(name_or_path: str) -> dict:
    """Blocks get PDDL-safe ids; the MuJoCo bodies use the same names."""
    path = Path(name_or_path)
    if not path.exists():
        path = next(d / f"{name_or_path}.yaml" for d in PRODUCTS if (d / f"{name_or_path}.yaml").exists())
    product = yaml.safe_load(path.read_text())
    for block in product["blocks"]:
        block["id"] = pddl_name(block["id"])
    if len({b["id"] for b in product["blocks"]}) != len(product["blocks"]):
        raise ValueError("block ids collide after PDDL name sanitising")
    return product


def footprint(block: dict) -> tuple[float, float]:
    sx, sy, _ = BRICK_SIZE[block["type"]]
    quarter = int(round(float(block["target"].get("yaw_deg", 0.0)))) % 180 == 90
    return (sy, sx) if quarter else (sx, sy)


def support_graph(product: dict) -> dict[str, set[str]]:
    """Bricks lying directly under each brick in the target product."""
    def overlaps(a, b, tol=0.002):
        (ax, ay, _), (bx, by, _) = a["target"]["position"], b["target"]["position"]
        (aw, ad), (bw, bd) = footprint(a), footprint(b)
        return abs(ax - bx) < (aw + bw) / 2 - tol and abs(ay - by) < (ad + bd) / 2 - tol

    supports = {}
    for upper in product["blocks"]:
        z = upper["target"]["position"][2]
        below = [b for b in product["blocks"]
                 if b["target"]["position"][2] < z - 1e-6 and overlaps(b, upper)]
        top = max((b["target"]["position"][2] for b in below), default=None)
        supports[upper["id"]] = {b["id"] for b in below
                                 if abs(b["target"]["position"][2] - top) < 1e-6}
    return supports


def world_target(block: dict, table_z: float) -> tuple[np.ndarray, float]:
    """Target centre of the brick in the world frame, and its target yaw (rad)."""
    x, y, z = block["target"]["position"]
    centre = np.array([ASSEMBLY_ORIGIN[0] + x, ASSEMBLY_ORIGIN[1] + y,
                       table_z + BRICK_SIZE[block["type"]][2] / 2 + z])
    return centre, float(np.radians(block["target"].get("yaw_deg", 0.0)))
