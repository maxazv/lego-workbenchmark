(define (domain lego-granular)
  (:requirements :adl)

  (:types
    cell
    brick
    type
  )

  (:predicates
    (shape ?b - brick ?d - type)
    (is_color ?b - brick ?c - color)
    (at ?b - brick ?anchor - cell)
    ; (clear ?l - location)
    (holding ?b - brick)
    (hand-empty)  ; can also be derived

    (occupied ?c - cell)
    (footprint ?d - type ?anchor - cell ?c - cell)  ; static: if brick of type d placed in cell ?anchor, then also occupies cell ?c

  )

  (:derived (occupied ?c - cell)
    (exists (?b - brick ?a - cell ?d - type) 
        (and (at ?b ?a) (shape ?b ?d) (footprint ?d ?a ?c))
    )
  )

  (:action pick
    :parameters (?b - brick ?from - location)
    :precondition (and
      (at ?b ?from)
      (hand-empty)
    )
    :effect (and
      (not (at ?b ?from))
      (not (hand-empty))
      (holding ?b)
    )
  )

  ;; Place the held item at a location that is geometrically free to receive it.
  (:action place
    :parameters (?b - brick ?to - location)
    :precondition (and
      (holding ?b)
      (forall (?c - cell) (imply (footprint ?b ?to ?c) (not (occupied ?c))))  ; for all cells our brick WILL occupy, that cell should be unoccupied
    )
    :effect (and
      (at ?b ?to)  ; NOTE: this also changes occupied (as it is derived predicate that depends on 'at' pred)
      (not (holding ?b))
      (hand-empty)
    )
  )

  (:action stack
    :parameters (?b - brick ?on - brick ?to - location)
    :precondition (and
      (at ?on ?to)
      (holding ?b)
      (not (exists (?c - brick) (stacked_on ?c ?on)))  ; maybe add predicate to indicate this (ie add pred to brick if stacked on)
    )
    :effect (and
      (not (holding ?b))
      (at ?b ?to)
      (stacked_on ?b ?on)
      (hand-empty)
    )
  )
)