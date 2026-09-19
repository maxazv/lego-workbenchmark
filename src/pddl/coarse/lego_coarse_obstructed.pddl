(define (domain lego-coarse)
  (:requirements :typing :existential-preconditions :negative-preconditions :derived-predicates)

  (:types
    ; item
    ; location
    grasp_config

    type
    brick
    color
    location
  )

  (:predicates
    (shape ?b - brick ?d - type)
    (is_color ?b - brick ?c - color)
    (at ?b - brick ?l - location)
    (stacked_on ?a - brick ?b - brick)
    (clear ?l - location)
    (holding ?b - brick ?c - grasp_config)

    (obstructs ?a - brick ?b - brick ?c - grasp_config)  ; sensed: brick a obstructs brick b's grasp_config c
    ; NOTE: would be sensible to make obstructs location dependent / geometric (similar to footprint in lego_granular.pddl)
    ; BUT we would have to be very pessimistic: ie if two locations are neighboring, then bricks will obstruct
    ; => better to do like ex4: sensed property based on GraspPlanner + optimism (essentially what current domain does)

    ; right now also assuming that once we place brick, then it wont obstruct approach of other bricks placements (optimism)

    (accessible ?b - brick ?c - grasp_config)  ; derivable from obstructs
    (hand-empty)
  )

  (:derived (clear ?l - location)
    (not (exists (?b - brick) (at ?b ?l)))
  )

  (:derived (accessible ?b - brick ?c - grasp_config)
    (not (exists (?a - brick) (obstructs ?a ?b ?c)))
  )

  (:action pick
    :parameters (?b - brick ?from - location ?c - grasp_config)
    :precondition (and
      (at ?b ?from)
      (hand-empty)
      (accessible ?b ?c)
    )
    :effect (and
      (not (at ?b ?from))
      (not (hand-empty))
      (forall (?a - brick)  ; clear all bricks that were obstructed by brick b
        (when (?exists (?d - grasp_config) (obstructs ?b ?a ?d)) (not (obstructs ?b ?a ?d)))
      )
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
      (at ?b ?to)
      (stacked_on ?b ?on)
      (hand-empty)
    )
  )
)

; when assembling it only matters how things are stacked... the order should not matter too much
; usually going layer by layer should workout and maybe some stability constraints