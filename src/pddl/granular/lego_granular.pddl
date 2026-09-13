(define (domain lego-granular)
  (:requirements :adl :derived-predicates)

  (:types
    cell
    brick
    type
    color
    orientation  ; ie degrees in {0, 90}
  )

  (:predicates
    (shape ?b - brick ?d - type)  ; static
    (is_color ?b - brick ?c - color)  ; static
    (at ?b - brick ?anchor - cell ?o - orientation)  ; fluent (we change it with pick/place/stack)
    (holding ?b - brick) ; fluent
    (occupied ?c - cell) ; derived
    (hand_empty)  ; also derived

    (footprint ?d - type ?o - orientation ?anchor - cell ?c - cell)  ; static: if brick of type d placed in cell ?anchor, then also occupies cell ?c
    ; FIXME we rely on footprint fully defined FOR EVERY (type, orientation) pair brick could be placed at!!!
    ; IF NOT: then precondition of place action always true if eg some orientation missing
    ; => depends on parser
    (above ?c1 - cell ?c2 - cell)  ; static: c1 directly one layer above c2
    (is-ground ?c - cell) ; static: NOTE IF THIS NOT SET FOR SOME CELLS, THEN PROBLEM UNSOLVABLE

    (stacked_on ?b1 - brick ?b2 - brick) ; derived: b1 stacked on b2

  )

  (:derived (occupied ?c - cell)
    (exists (?b - brick ?a - cell ?o - orientation ?d - type) 
        (and (at ?b ?a ?o) (shape ?b ?d) (footprint ?d ?o ?a ?c))
    )
  )

  (:derived (hand_empty)
    (not (exists (?b - brick) (holding ?b)))
  )

  ; Intro2AI comin in clutch
  (:derived (stacked_on ?top - brick ?bot - brick)
  (exists (?c1 ?c2 - cell)
    (and (above ?c1 ?c2)
         (exists (?a - cell ?o - orientation ?d - type)
           (and (at ?bot ?a ?o) (shape ?bot ?d) (footprint ?d ?o ?a ?c2)))
         (exists (?a - cell ?o - orientation ?d - type)
           (and (at ?top ?a ?o) (shape ?top ?d) (footprint ?d ?o ?a ?c1))))))

  (:action pick
    :parameters (?b - brick ?from - cell ?o - orientation)
    :precondition (and
      (at ?b ?from ?o)
      (hand_empty)
    )
    :effect (and
      (not (at ?b ?from ?o))
      (holding ?b)  ; NOTE: changes hand_empty as it depends on holding rped
    )
  )

  ;; Place the held item at a location that is geometrically free to receive it.
  (:action place
    :parameters (?b - brick ?to - cell ?o - orientation)
    :precondition (and
      (holding ?b)
      ; NOTE: below requires all studs of brick to have support!!!!
      (forall (?c - cell)
        (imply 
            (exists (?d - type) (and (shape ?b ?d) (footprint ?d ?o ?to ?c)))           ; for all cells if they WILL belong to our footprint
            (and ((not occupied ?c))                                                    ; those cells must be unoccupied AND
                (or (and (is-ground ?c))                                                ; either be ground or
                    (exists (?below - cell) (and (above ?c ?below) (occupied ?below)))  ; have cells below that are occupied
                )
            )
        )
      )
    )
    :effect (and
      (at ?b ?to ?o)  ; NOTE: this also changes occupied (as it is derived predicate that depends on 'at' pred) AND stacked_on
      (not (holding ?b))
    )
  )
)



; some notes:
; - theoretically if overhang but cell beneath unoccupied, planner could place brick there, however not reachable!