(define (domain lego-coarse-v2)
  (:requirements :typing :negative-preconditions)

  (:types
    type
    brick
    color
    location  ; NOTE: we dont need target locations anymore
  )

  (:predicates
    (shape ?b - brick ?d - type)
    (is_color ?b - brick ?c - color)
    (at ?b - brick ?l - location)
    (at-target ?b - brick)
    (stacked_on ?a - brick ?b - brick)
    (top-clear ?b - brick)  ; nothing currently stacked on ?b
    (clear ?l - location)
    (holding ?b - brick)
    (hand-empty)
  )

  (:action pick
    :parameters (?b - brick ?from - location)
    :precondition (and
      (at ?b ?from)
      (hand-empty)
      (top-clear ?b)
    )
    :effect (and
      (not (at ?b ?from))
      (not (hand-empty))
      (holding ?b)
      (clear ?from)
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
  (:action placeToTarget
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
      (hand-empty)
    )
  )
)

; when assembling it only matters how things are stacked... the order should not matter too much
; usually going layer by layer should workout and maybe some stability constraints