(define (domain lego-coarse-v2)
  (:requirements :typing :negative-preconditions)

  (:types
    ; color
    ; location  ; NOTE: we dont need target locations anymore (we prolly dont need locations altogether)
    ;type

    brick
  )

  (:predicates
    ; (is_color ?b - brick ?c - color)
    ; (at ?b - brick ?l - location)
    ; (clear ?l - location)
    ; (shape ?b - brick ?d - type)

    (is-root ?b - brick)  ; not stacked on any brick
    (at-target ?b - brick)  ; brick placed correctly in the structure (for root or stacked bricks)
    (top-clear ?b - brick)  ; nothing currently stacked on ?b
    (stacked_on ?a - brick ?b - brick)
    (holding ?b - brick)
    (hand-empty)
  )

  (:action pick
    :parameters (?b - brick)
    :precondition (and
      (hand-empty)
      (top-clear ?b)
    )
    :effect (and
      ; (not (at ?b ?from))
      (not (hand-empty))
      (holding ?b)
      ; (clear ?from)
    )
  )

  ; (:action unstack
  ;   :parameters (?b - brick ?on - brick)
  ;   :precondition (and 
  ;     (stacked_on ?b ?on) 
  ;     (hand-empty) 
  ;     (top-clear ?b)
  ;   )
  ;   :effect (and 
  ;     (not (stacked_on ?b ?on)) 
  ;     (top-clear ?on)
  ;     (not (hand-empty)) 
  ;     (holding ?b))
  ;   )

  ; NOTE: place irrelevant now
  ; ; Place the held item at a location that is geometrically free to receive it.
  ; (:action place
  ;   :parameters (?b - brick ?to - location)
  ;   :precondition (and
  ;     (holding ?b)
  ;     (clear ?to)
  ;   )
  ;   :effect (and
  ;     (not (holding ?b))
  ;     (at ?b ?to)
  ;     (hand-empty)
  ;     (not (clear ?to))
  ;   )
  ; )

  ; should only be used for root bricks
  (:action place
    :parameters (?b - brick)
    :precondition (and
      (holding ?b)
      (is-root ?b)
    )
    :effect (and
      (not (holding ?b))
      (at-target ?b)
      (hand-empty)
    )
  )

  (:action stack
    :parameters (?b - brick ?on - brick)
    :precondition (and
      (holding ?b)
      (at-target ?on)
      (top-clear ?on)
    )
    :effect (and
      (not (holding ?b))
      (not (top-clear ?on))
      (stacked_on ?b ?on)
      (at-target ?b)
      (hand-empty)
    )
  )
)


; Problems:
;   - no opportunity for TAMP => if plan fails, its over