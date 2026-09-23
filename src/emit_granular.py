"""lego-granular problem emitter:

- cell = one stud position on one layer, named c_{ix}_{iy}_{layer}
- a brick's `at` anchor is its min-corner stud cell (as defined in yaml_loader)
- `footprint` facts list every stud cell the brick covers for that (type, orientation, anchor)
- `above` connects vertically adjacent stud cells
- `is_ground` marks the base layer

NOTE on the domain's `place` precondition: it requires EVERY footprint cell
to be supported. Real tasks violate that constantly (offset stacking with overhang)
=> expect "unsolvable" on most tasks until precondition is relaxed => change granular domain (not emitter bug)
"""
from yaml_loader import Brick, Lattice, ORIENT

DIM = {"brick_4x2": "t4x2", "brick_2x2": "t2x2"}

def _cn(ix, iy, layer):
    return f"c_{ix}_{iy}_{layer}".replace("-", "n")  # negative coord indicated with prefix n

def emit_problem(bricks: list[Brick], lat: Lattice, problem_name: str) -> str:
    target_cells = {(ix, iy, b.layer) for b in bricks for (ix, iy) in b.cells}
    min_layer = min(b.layer for b in bricks)
    pick_cell = {b.name: f"p_{b.name}" for b in bricks}

    objects = [
        f"    {' '.join(b.name for b in bricks)} - brick",
        f"    {' '.join(sorted({DIM[b.type] for b in bricks}))} - type",
        f"    {' '.join(sorted({b.color for b in bricks}))} - color",
        f"    {' '.join(sorted({ORIENT[b.yaw] for b in bricks} | {ORIENT[b.initial_yaw] for b in bricks if b.initial_yaw is not None}))} - orientation",
        # f"    {' '.join([_cn(*c) for c in sorted(target_cells)] + list(pick_cell.values()))} - cell",
        f"    {' '.join(list(pick_cell.values()))} - cell  ; pick cells",
    ]
    layer_cells = {}
    for (ix, iy, layer) in target_cells:
        layer_cells.setdefault(layer, set()).add((ix, iy, layer))
    for layer in sorted(layer_cells.keys()):
        objects.append(
            f"    {' '.join([_cn(*c) for c in sorted(layer_cells[layer])])} - cell  ; layer {layer}"
        )
        

    init = []   # in contrast to emit_coarse, hand_empty is derived
    for b in bricks:
        init += [f"    (shape {b.name} {DIM[b.type]})", f"    (is_color {b.name} {b.color})"]
        if b.initial is not None:
            init.append(f"    (at {b.name} {pick_cell[b.name]} {ORIENT[b.initial_yaw]})")
    seen = set()
    for b in bricks:
        key = (b.type, b.yaw, b.anchor, b.layer)
        if key in seen: continue
        seen.add(key)
        anchor_name = _cn(*b.anchor, b.layer)
        for (ix, iy) in sorted(b.cells):
            init.append(f"    (footprint {DIM[b.type]} {ORIENT[b.yaw]} {anchor_name} {_cn(ix, iy, b.layer)})")
    for (ix, iy, layer) in sorted(target_cells):
        if layer == min_layer:
            init.append(f"    (is_ground {_cn(ix, iy, layer)})")
        if (ix, iy, layer - 1) in target_cells:
            init.append(f"    (above {_cn(ix, iy, layer)} {_cn(ix, iy, layer - 1)})")

    goal = [f"(exists (?b - brick) (and (at ?b {_cn(*b.anchor, b.layer)} {ORIENT[b.yaw]}) "
            f"(shape ?b {DIM[b.type]}) (is_color ?b {b.color})))" for b in bricks]

    nl = "\n"
    return (f"(define (problem {problem_name})\n  (:domain lego-granular)\n"
            f"  (:objects\n{nl.join(objects)}\n  )\n"
            f"  (:init\n{nl.join(init)}\n  )\n"
            f"  (:goal (and\n{nl.join('    ' + g for g in goal)}\n  ))\n)\n")