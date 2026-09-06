(define (domain lego-tamp-simple)
  (:requirements :strips :typing)

  (:types
    brick
  )

  (:predicates
    (on_table ?b - brick)
    (stacked_on ?top - brick ?bottom - brick)
    (clear ?b - brick)
    (holding ?b - brick)
    (hand-empty)
  )

  (:action pick
    :parameters (?b - brick)
    :precondition (and
        (on_table ?b)
        (clear ?b)
        (hand-empty)
    )
    :effect (and
        (not (on_table ?b))
        (not (hand-empty))
        (holding ?b)
    )
  )

  (:action stack
    :parameters (?top - brick ?bottom - brick)
    :precondition (and
        (holding ?top)
        (clear ?bottom)
    )
    :effect (and
        (not (holding ?top))
        (not (clear ?bottom))
        (stacked_on ?top ?bottom)
        (hand-empty) 
        (clear ?top)
    )
  )
)