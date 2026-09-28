(define (domain lego-beyond-tier2)
  (:requirements :typing :negative-preconditions)

  (:types
    brick
  )

  (:predicates
    (at-target ?b - brick)  ; brick placed correctly in the structure (for root or stacked bricks)
    ; (top-clear ?b - brick)  ; nothing currently stacked on ?b
    (holding ?b - brick)  ; action effect of pick
    (placeable ?b - brick)  ; action effect of prepare_place
    (supports ?b1 - brick ?b2 - brick)  ; preprocessed from supporters
    (hand-empty)

    (pickable ?b - brick)
    (diff ?b1 - brick ?b2 - brick)  ; preprocessed: needed for prepare_place, otherwise planner can just put same brick into params
  )

  (:action pick
    :parameters (?b - brick)
    :precondition (and
      (hand-empty)
      ; (top-clear ?b)
      ; (not (at-target ?b))  ; engine doesnt support this...
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
        (diff ?b1 ?b2)
        (diff ?b1 ?b3)
        (diff ?b1 ?b4)
        (diff ?b1 ?b5)
        (diff ?b2 ?b3)
        (diff ?b2 ?b4)
        (diff ?b2 ?b5)
        (diff ?b3 ?b4)
        (diff ?b3 ?b5)
        (diff ?b4 ?b5)

        (supports ?b1 ?b)
        (supports ?b2 ?b)
        (supports ?b3 ?b)
        (supports ?b4 ?b)
        (supports ?b5 ?b)

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