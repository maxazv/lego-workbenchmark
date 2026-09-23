"""lego-coarse problem emitter. 
`location` = one per support-tree ROOT (base-layer bricks: bricks on the ground at assembly).
Every brick stacked on that tree shares the root's location via stacked_on chains.

LIMITATION: many upper bricks in the sampled tasks rest on several bricks or hang partly over air. 
The coarse domain's binary stacked_on(a, b) can only record ONE supporter, so when a brick rests on two, 
we keep the one with the most shared studs and warn.
"""
from yaml_loader import Brick, supporters

DIM = {"brick_4x2": "t4x2", "brick_2x2": "t2x2"}

def emit_problem(bricks: list[Brick], problem_name: str) -> str:
    sup = supporters(bricks)
    parent = {}  # child to parent (support brick) mapping
    for b in bricks:
        if sup[b.name]:
            parent[b.name] = sup[b.name][0][0]
            if len(sup[b.name]) > 1:
                names = [s.name for s, _ in sup[b.name]]
                print(f"  WARN {problem_name}: {b.name} rests on {names}; coarse domain keeps only {parent[b.name].name}")

    # build adjacency-list based on support (each parent has list of children it directly supports)
    roots = [b for b in bricks if b.name not in parent]
    children = {}
    for c, p in parent.items():
        children.setdefault(p.name, []).append(c)

    pick_loc = {b.name: f"pick_{b.name}" for b in bricks}
    root_loc = {r.name: f"asm_{r.name}" for r in roots}
    by_name = {b.name: b for b in bricks}

    objects = [
        f"    {' '.join(b.name for b in bricks)} - brick",
        f"    {' '.join(sorted({DIM[b.type] for b in bricks}))} - type",
        f"    {' '.join(sorted({b.color for b in bricks}))} - color",
        f"    {' '.join(list(pick_loc.values()) + list(root_loc.values()))} - location",
    ]
    init = ["    (hand-empty)"]
    for b in bricks:
        init += [f"    (shape {b.name} {DIM[b.type]})",
                 f"    (is_color {b.name} {b.color})",
                 f"    (at {b.name} {pick_loc[b.name]})"]

    goal_clauses = []
    for r in roots:
        tree = [r.name]
        i = 0
        # unfold support structure for current root into flattened tree
        while i < len(tree):
            tree += children.get(tree[i], [])
            i += 1
        var = {n: f"?b{k}" for k, n in enumerate(tree)}
        body = [f"(at {var[r.name]} {root_loc[r.name]})"]
        # for each child in tree, append its stacked_on pred wrt parent and its shape/color
        for n in tree:
            b = by_name[n]
            if n != r.name:
                body.append(f"(stacked_on {var[n]} {var[parent[n].name]})")
            body += [f"(shape {var[n]} {DIM[b.type]})", f"(is_color {var[n]} {b.color})"]
        goal_clauses.append(f"(exists ({' '.join(var.values())} - brick) (and {' '.join(body)}))")

    nl = "\n"
    return (f"(define (problem {problem_name})\n  (:domain lego-coarse)\n"
            f"  (:objects\n{nl.join(objects)}\n  )\n"
            f"  (:init\n{nl.join(init)}\n  )\n"
            f"  (:goal (and\n{nl.join('    ' + g for g in goal_clauses)}\n  ))\n)\n")