(define (domain lego-coarse)
  (:requirements :strips :typing :existential-preconditions)

  (:types
    ; item
    ; location
    ; grasp-config

    dim
    brick
    color
    location
  )

  (:predicates
    (shape ?b - brick ?d - dim)
    (is ?b - brick ?c - color)
    (at ?b - brick ?l - location)
    (stacked_on ?a - brick ?b - brick)
    (clear ?l - location)

    ; (config-for ?c - grasp-config ?o - brick) ; static: c is a candidate grasp for o
    ; (ik-feasible ?c - grasp-config)           ; geometric: grasp c is collision-free (sensed)
    ; (accessible ?o - brick)                   ; geometric: o has >=1 feasible grasp (sensed)
    ; (obstructs ?a - brick ?b - brick)         ; geometric: a blocks b's approach (sensed)
    ; (holding ?b - brick ?c - grasp-config)

    (holding ?b - brick)
    (hand-empty)
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
      (clear ?to)
    )
    :effect (and
      (not (holding ?b))
      (not (clear ?to))
      (at ?b ?to)
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
      (at ?o ?to)
      (stacked_on ?o ?on)
      (hand-empty)
    )
  )
)

; when assembling it only matters how things are stacked... the order should not matter too much
; usually going layer by layer should workout and maybe some stability constraints