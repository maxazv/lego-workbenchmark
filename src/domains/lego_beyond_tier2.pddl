(define (domain lego-beyond-tier2)
  (:requirements :typing :negative-preconditions)

  (:types
    brick
    cell
  )

  (:predicates
    (at-target ?b - brick)  ; brick placed correctly in the structure (for root or stacked bricks)
    (top-clear ?b - brick)  ; nothing currently stacked on ?b
    (holding ?b - brick)
    (hand-empty)

    (is-2x2 ?b - brick)
    (is-4x2 ?b - brick)
    (supported ?b - brick ?c - cell)
    (target-cell ?b - brick ?c - cell)
    (placeable ?b - brick)
    (below-cell ?b - brick ?c - cell)
    (pickable ?b - brick)

    (diff ?c1 - cell ?c2 - cell)  ; needed for prepare_place_* otherwise planner can just put same supported cell into params
  )

  (:action pick
    :parameters (?b - brick)
    :precondition (and
      (hand-empty)
      (top-clear ?b)
      ; (not (at-target ?b))  ; engine doesnt support this...
      (pickable ?b)
    )
    :effect (and
      (not (hand-empty))
      (holding ?b)
    )
  )

  (:action support
    :parameters (?base - brick ?b - brick ?c - cell)
    :precondition (and
        (at-target ?base)
        (below-cell ?base ?c)
        (target-cell ?b ?c)
    )
    :effect (and
        (supported ?b ?c)
        (not (top-clear ?base))  ; just done so base cant be picked up again
    )
  )

  (:action prepare_place_2x2
    :parameters (?b - brick ?c1 - cell ?c2 - cell ?c3 - cell ?c4 - cell)
    :precondition (and
        (is-2x2 ?b)

        (diff ?c1 ?c2)
        (diff ?c1 ?c3)
        (diff ?c1 ?c4)
        (diff ?c2 ?c3)
        (diff ?c2 ?c4)
        (diff ?c3 ?c4)

        (target-cell ?b ?c1)
        (target-cell ?b ?c2)
        (target-cell ?b ?c3)
        (target-cell ?b ?c4)

        (supported ?b ?c1)
        (supported ?b ?c2)
        (supported ?b ?c3)
        (supported ?b ?c4)
    )
    :effect (placeable ?b)
  )

  ; if there are ever more shapes, use python script to generate the pddl domain file...
  (:action prepare_place_4x2
    :parameters (?b - brick ?c1 - cell ?c2 - cell ?c3 - cell ?c4 - cell ?c5 - cell ?c6 - cell ?c7 - cell ?c8 - cell)
    :precondition (and
        (is-4x2 ?b)

        (diff ?c1 ?c2)
        (diff ?c1 ?c3)
        (diff ?c1 ?c4)
        (diff ?c1 ?c5)
        (diff ?c1 ?c6)
        (diff ?c1 ?c7)
        (diff ?c1 ?c8)
        (diff ?c2 ?c3)
        (diff ?c2 ?c4)
        (diff ?c2 ?c5)
        (diff ?c2 ?c6)
        (diff ?c2 ?c7)
        (diff ?c2 ?c8)
        (diff ?c3 ?c4)
        (diff ?c3 ?c5)
        (diff ?c3 ?c6)
        (diff ?c3 ?c7)
        (diff ?c3 ?c8)
        (diff ?c4 ?c5)
        (diff ?c4 ?c6)
        (diff ?c4 ?c7)
        (diff ?c4 ?c8)
        (diff ?c5 ?c6)
        (diff ?c5 ?c7)
        (diff ?c5 ?c8)
        (diff ?c6 ?c7)
        (diff ?c6 ?c8)
        (diff ?c7 ?c8)

        (target-cell ?b ?c1)
        (target-cell ?b ?c2)
        (target-cell ?b ?c3)
        (target-cell ?b ?c4)
        (target-cell ?b ?c5)
        (target-cell ?b ?c6)
        (target-cell ?b ?c7)
        (target-cell ?b ?c8)

        (supported ?b ?c1)
        (supported ?b ?c2)
        (supported ?b ?c3)
        (supported ?b ?c4)
        (supported ?b ?c5)
        (supported ?b ?c6)
        (supported ?b ?c7)
        (supported ?b ?c8)
    )
    :effect (placeable ?b)
  )

  (:action place
    :parameters (?b - brick)
    :precondition (and
      (holding ?b)
      (placeable ?b)
    )
    :effect (and
      (not (holding ?b))
      (at-target ?b)
      (not (pickable ?b))
      (hand-empty)
    )
  )

)


; Problems:
; - prepare_place_4x2 will blow up because has 2+8 params which pyperplan builds full cartesian product of => (#cells)^8
; - really annoying because only reason on stud-cell-level is to guarantee that all supporting bricks placed... 
; - => we can just do the same thing but on brick-level instead of stud-level
; - using some predicate like supports(base, brick) => preprocessing and then requiring that all support bricks are placed instead...
; - then can look at dataset for maximum number of supporting bricks which likely less than the 8 cells

; - or even more cheesy: craft a custom domain using script for each task with individual place_brick actions...
;   - place_brick_<b.name> for each brick b with precon: (at-target supp) for supp in supps[b.name]
;   - planner now just reduces to figuring out some DAG ordering of the supporter tree
;   - idk if this defeats purpose because not single domain for dataset


; - no opportunity for TAMP => if plan fails, its over