(define (domain lego-tamp)
  (:requirements :strips :typing)

  (:types
    ; item
    ; location
    grasp-config

    dim
    brick
    grid_loc
    color
    orientation
  )

  (:predicates
    (type ?b - brick ?w - dim ?h - dim)
    (at ?o - brick ?l - grid_loc)
    (ori ?b - brick ?o - orientation)         ; orientation of brick (unnecessary for tier 2)
    (stacked_on ?a - brick ?b - brick)        ; a stacked on b

    (in_pick_area ?l grid_loc)
    (in_asm_area ?l grid_loc)

    (config-for ?c - grasp-config ?o - brick) ; static: c is a candidate grasp for o
    (ik-feasible ?c - grasp-config)           ; geometric: grasp c is collision-free (sensed)
    (accessible ?o - brick)                   ; geometric: o has >=1 feasible grasp (sensed)
    (obstructs ?a - brick ?b - brick)         ; geometric: a blocks b's approach (sensed)
    (holding ?o - brick ?c - grasp-config)
    (hand-empty)
  )

  ;; Pick an item that is currently graspable (accessible).
  (:action pick
    :parameters (?o - brick ?from - grid_loc ?c - grasp-config)
    :precondition (and
      (in_pick_area ?from)
      (at ?o ?from)
      (hand-empty)
      (accessible ?o)
      (config-for ?c ?o)
    )
    :effect (and
      (not (at ?o ?from))
      (not (hand-empty))
      (holding ?o ?c)
    )
  )

  ;; Pick an item that is itself graspable AND is obstructing ?b.  Removing it
  ;; OPTIMISTICALLY makes ?b accessible (validated geometrically after execution).
  (:action pick-to-clear
    :parameters (?o - brick ?from - grid_loc ?c - grasp-config ?b - brick)
    :precondition (and
      (in_pick_area ?from)
      (at ?o ?from)
      (hand-empty)
      (accessible ?o)
      (config-for ?c ?o)
      (obstructs ?o ?b)
    )
    :effect (and
      (not (at ?o ?from))
      (not (hand-empty))
      (holding ?o ?c)
      (not (obstructs ?o ?b))
      (accessible ?b)
    )
  )

  ;; Place the held item at a location that is geometrically free to receive it.
  (:action place
    :parameters (?o - brick ?to - location ?c - grasp-config)
    :precondition (and
      (in_asm_area ?to)
      (holding ?o ?c)
    )
    :effect (and
      (not (holding ?o ?c))
      (not (clear ?to))
      (at ?o ?to)
      (hand-empty)
    )
  )

  (:action stack
    :parameters (?o - brick ?on - brick ?to - grid_loc ?c - grasp-config)
    :precondition (and
      (in_asm_area ?to)
      (at ?on ?to)
      (holding ?o ?c)
    )
    :effect (and
      (not (holding ?o ?c))
      (not (clear ?to))
      (at ?o ?to)
      (stacked_on ?o ?on)
      (hand-empty)
    )
  )
)


; when assembling it only matters how things are stacked... the order should not matter too much
; usually going layer by layer should workout and maybe some stability constraints