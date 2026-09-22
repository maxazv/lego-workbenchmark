(define (domain lego-coarse)
  (:requirements :typing :existential-preconditions :negative-preconditions :derived-predicates)

  (:types
    type
    brick
    color
    location
    grasp-config
  )

  (:predicates
    (shape ?b - brick ?d - type)
    (is_color ?b - brick ?c - color)
    (at ?b - brick ?l - location)
    (stacked_on ?a - brick ?b - brick)
    (loc_clear ?l - location)
    (brick_clear ?b - brick)
    (holding ?b - brick ?c - grasp-config)
    (hand-empty)

    ; Add some geometric constraints
    (place-free ?b - brick ?l - location)      ; geometric: l can receive o (sensed)
    (config-for ?c - grasp-config ?b - brick)  ; static: c is a candidate grasp for o
    (ik-feasible ?c - grasp-config)           ; geometric: grasp c is collision-free (sensed)
    (accessible ?b - brick)                    ; geometric: o has >=1 feasible grasp (sensed)
    (obstructs ?b1 - brick ?b2 - brick)            ; geometric: b1 blocks b2's approach (sensed)
  )
  
  ; derived gibt bei Tampanda parser Probleme

  (:action pick
    :parameters (?b - brick ?from - location ?c - grasp-config)
    :precondition (and
      (at ?b ?from)
      (brick_clear ?b) ; Wir wollen keine bricks aufheben wo ein anderer drauf ist
      (accessible ?b) ; Analog zur BlocksWorld, gerade die initial ground truths sehen sehr eng zusammengelegt aus
      (hand-empty)
      (config-for ?c ?b)
    )
    :effect (and
      (not (at ?b ?from))
      (not (hand-empty))
      (holding ?b ?c)
      (loc_clear ?from)
    )
  )

  ;; Pick a brick b1 that is itself graspable AND is obstructing ?b2.  Removing it
  ;; OPTIMISTICALLY makes ?b2 accessible (validated geometrically after execution).
  (:action pick-to-clear
    :parameters (?b1 - brick ?from - location ?c - grasp-config ?b2 - brick)
    :precondition (and
      (at ?b1 ?from)
      (hand-empty)
      (accessible ?b1)
      (config-for ?c ?b1)
      (obstructs ?b1 ?b2)
    )
    :effect (and
      (not (at ?b1 ?from))
      (not (hand-empty))
      (holding ?b1 ?c)
      (loc_clear ?from)
      (not (obstructs ?b1 ?b2))
      (accessible ?b2)
    )
  )

  ;; Place the held item at a location that is geometrically free to receive it.
  (:action place
    :parameters (?b - brick ?to - location ?c - grasp-config) 
    :precondition (and
      (holding ?b ?c)
      (loc_clear ?to)
      (place-free ?b ?to)
    )
    :effect (and
      (not (holding ?b ?c))
      (at ?b ?to)
      (hand-empty)
      (not (loc_clear ?to))
      (brick_clear ?b)
    )
  )

  (:action stack
    :parameters (?b - brick ?on - brick ?to - location ?c - grasp-config)
    :precondition (and
      (at ?on ?to)
      (holding ?b ?c)
      (brick_clear ?on) ; Hab das entsprechende Prädikat eingeführt
    )
    :effect (and
      (not (holding ?b ?c))
      (at ?b ?to)
      (stacked_on ?b ?on)
      (hand-empty)
      (not (brick_clear ?on))
      (brick_clear ?b) ; Initialisiere den platzierten Brick als clear
    )
  )
)

; when assembling it only matters how things are stacked... the order should not matter too much
; usually going layer by layer should workout and maybe some stability constraints