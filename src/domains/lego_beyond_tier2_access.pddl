(define (domain lego-beyond-tier2-access)

  ; the whole idea is to force DAG ordering: place brick only if ALL supporters placed

  (:requirements :typing :negative-preconditions)

  (:types
    brick
  )

  (:predicates
    (at-target ?b - brick)  ; brick placed correctly in the structure (for root or stacked bricks)
    (holding-x ?b - brick)  ; how brick is held when placing at target
    (holding-y ?b - brick)
    (placeable ?b - brick)  ; action effect of prepare_place
    (hand-empty)
    (pickable ?b - brick)
    (unplaced ?b - brick)  ; used in place_[x, y] (different from pickable because nbr fillers)

    ; instead of using diff like beyond_tier2_v2 which forces the param binding
    ; in a way we collapsed the diff predicate into the supports predicate which also makes building bridge easier
    (supports1 ?s - brick ?b - brick)
    (supports2 ?s - brick ?b - brick)
    (supports3 ?s - brick ?b - brick)
    (supports4 ?s - brick ?b - brick)
    (supports5 ?s - brick ?b - brick)

    ; slot i represents ?bs i-th same layer neighbor along axis x/y (else we again use new filler brick if not enough x/y-neighbors)
    (nbrx1 ?n - brick ?b - brick)
    (nbrx2 ?n - brick ?b - brick)
    (nbrx3 ?n - brick ?b - brick)
    (nbry1 ?n - brick ?b - brick)
    (nbry2 ?n - brick ?b - brick)
    (nbry3 ?n - brick ?b - brick)

  )

  ; just like previous pick but now also saying: i WILL place brick along x-axis => look at x-neighbors
  (:action pick_x
    :parameters (?b - brick)
    :precondition (and
      (hand-empty)
      (pickable ?b)
    )
    :effect (and
      (not (hand-empty))
      (holding-x ?b)
    )
  )

  (:action pick_y
    :parameters (?b - brick)
    :precondition (and
      (hand-empty)
      (pickable ?b)
    )
    :effect (and
      (not (hand-empty))
      (holding-y ?b)
    )
  )

  (:action prepare_place
    :parameters (?b - brick ?b1 - brick ?b2 - brick ?b3 - brick ?b4 - brick ?b5 - brick)  ; see analysis.ipynb: max of 5 supps in dataset
    :precondition (and
        (supports1 ?b1 ?b)
        (supports2 ?b2 ?b)
        (supports3 ?b3 ?b)
        (supports4 ?b4 ?b)
        (supports5 ?b5 ?b)

        (at-target ?b1)
        (at-target ?b2)
        (at-target ?b3)
        (at-target ?b4)
        (at-target ?b5)
    )
    :effect (placeable ?b)
  )

  (:action place_x
    :parameters (?b - brick ?n1 - brick ?n2 - brick ?n3 - brick)
    :precondition (and
      (holding-x ?b) (placeable ?b)
      (nbrx1 ?n1 ?b) (nbrx2 ?n2 ?b) (nbrx3 ?n3 ?b)
      (unplaced ?n1) (unplaced ?n2) (unplaced ?n3)
    )
    :effect (and
      (not (holding-x ?b))
      (at-target ?b)
      (not (pickable ?b))
      (not (unplaced ?b))
      (hand-empty)
    )
  )

  (:action place_y
    :parameters (?b - brick ?n1 - brick ?n2 - brick ?n3 - brick)
    :precondition (and
      (holding-y ?b) (placeable ?b)
      (nbry1 ?n1 ?b) (nbry2 ?n2 ?b) (nbry3 ?n3 ?b)
      (unplaced ?n1) (unplaced ?n2) (unplaced ?n3)
    )
    :effect (and
      (not (holding-y ?b))
      (at-target ?b)
      (not (pickable ?b))
      (not (unplaced ?b))
      (hand-empty)
    )
  )

)
