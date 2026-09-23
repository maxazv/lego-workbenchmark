(define (domain lego-coarse)
  (:requirements :typing :existential-preconditions :negative-preconditions)

  (:types
    ; item
    ; location
    ; grasp-config

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
    (holding ?b - brick)
    (hand-empty)
  )

  ;; "clear ?l" used to be a derived predicate, (not (exists (?b) (at ?b ?l))).
  ;; unified_planning (the parser behind TAMPanda's DomainBridge) has no support
  ;; for :derived, so the definition is inlined where it was used, in `place`.
  ;; Same semantics, and problem files never mention it, so they stay valid.

  (:action pick
    :parameters (?b - brick ?from - location)
    :precondition (and
      (at ?b ?from)
      (hand-empty)
      ;; nothing stacked on it: without this the planner stacks bricks in the
      ;; pick area and then "picks" the loaded base brick, which the robot cannot do
      (not (exists (?c - brick) (stacked_on ?c ?b)))
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
      (not (exists (?c - brick) (at ?c ?to)))   ; location is clear
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