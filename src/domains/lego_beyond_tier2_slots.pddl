(define (domain lego-beyond-tier2-slots)

  ; the whole idea is to force DAG ordering: place brick only if ALL supporters placed

  (:requirements :typing :negative-preconditions)

  (:types
    brick
  )

  (:predicates
    (at-target ?b - brick)  ; brick placed correctly in the structure (for root or stacked bricks)
    (holding ?b - brick)  ; action effect of pick
    (placeable ?b - brick)  ; action effect of prepare_place
    ; instead of using diff like beyond_tier2_v2 which forces the param binding
    ; in a way we collapsed the diff predicate into the supports predicate which also makes building bridge easier
    (supports1 ?s - brick ?b - brick)
    (supports2 ?s - brick ?b - brick)
    (supports3 ?s - brick ?b - brick)
    (supports4 ?s - brick ?b - brick)
    (supports5 ?s - brick ?b - brick)
    (hand-empty)

    (pickable ?b - brick)
  )

  (:action pick
    :parameters (?b - brick)
    :precondition (and
      (hand-empty)
      (pickable ?b)
    )
    :effect (and
      (not (hand-empty))
      (holding ?b)
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
