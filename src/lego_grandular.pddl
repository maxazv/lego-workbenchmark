(define (domain lego)
  (:requirements :strips :typing)

  (:types brick color grid_loc layer)

  ;; layer l implies there are l bricks stacked

  (:predicates 
    (voxel ?x - grid_loc)
    (occ ?a - brick ?x - grid_loc ?l - layer)  ;; location x at layer l occupied by brick a
    (color ?a - brick)  ;; what color does brick (ie all voxels of brick) have

    (stacked_on ?a - brick ?b - brick)  ;; brick a stacked on brick b
    (graspable ?a - brick)  ;; brick a is graspable (eg no brick stacked on a)

    (clear ?l - grid_loc)  ;; brick can be placed in grid location l
    (in_pick_area ?l grid_loc)
    (in_asm_area ?l grid_loc)

    ;; (target_pose )
  )

  ;; TODO actions and predicates above

)


(define (domain lego)
  (:requirements :strips :typing)

  ;; we have bricks with varying width/heights (brick types) and colors
  ;; each brick can be in some (rasterized) grid location
  ;; at the same grid location the same brick may be placed in a different orientation
  (:types brick num color grid_loc orientation grasp-config)

  (:predicates 
    (type ?b - brick ?w - num ?h - num)
    ;; width always goes to the right and height always to the front
    ;; => zero-point clearly defined and we define position of brick to be this zero-point
    (at ?b - brick ?l grid_loc)
    (ori ?b - brick ?o - orientation)

    (color ?a - brick)  ;; what color does brick have
    (graspable ?a - brick)  ;; brick a is graspable (no brick in grasp corridor)

    (clear ?l - grid_loc)  ;; brick can be placed in grid location l
    (in_pick_area ?l grid_loc)
    (in_asm_area ?l grid_loc)

    (hand_empty)
  )

  ;; Pick up brick from a grid location in pick area.
  ;; Pre:  brick is at that location AND the gripper is free.
  ;; Post: gripper holds the brick, location becomes clear.
  (:action pick
    :parameters (?b - brick ?from - grid_loc ?c - grasp-config)
    :precondition (and 
      (at ?b ?from) 
      (hand_empty)
      (in_pick_area ?from)
      (config-for ?c ?b)
    )
    :effect (and
      (not (at ?b ?from))
      (not (hand_empty))
      (holding ?b)
      (clear ?from)
    )
  )

  ;; pick brick b to clear brick d
  (:action pick-to-clear
    :parameters (?b - brick ?from - grid_loc ?c - grasp-config ?d - brick)
    :precondition (and 
      (at ?b ?from) 
      (hand_empty)
      (accessible ?b)
      (config-for ?c ?b)
      (obstructs ?b ?d)
      (in_pick_area ?from) 
    )
    :effect (and
      (not (at ?b ?from))
      (not (hand_empty))
      (holding ?b)
      (clear ?from)
    )
  )

  (:action place
    :parameters (?b - brick ?l - grid_loc ?o - orientation)
    :precondition (and 
      (holding ?b) 
      (ori ?b ?o) 
      (not (obstructed ?b ?l ?o))
    )
    :effect (and
      (not (holding ?b))
      (hand_empty)
      (ori ?b ?to)
    )
  )

  (:action stack
    :parameters (?b - brick ?l - grid_loc ?o - orientation)
    :precondition (and 
      (holding ?b) 
      (ori ?b ?o) 
      (not (obstructed ?b ?l ?o))
    )
    :effect (and
      (not (holding ?b))
      (hand_empty)
      (ori ?b ?to)
    )
  )
)



;; when you place something, you place a GROUP of voxels on another GROUP of voxels
;; or zero-point of a brick can be placed on any voxel wrt some orientation
;; brick 4x2 equivalent to brick 2x4 (wxh)
;; we define brick width to always be leq to height
;; for up to tier 2 (maybe even 3) just using `stacked` predicate should be enough

;; => very simple version of this is just sheet 4 + some modifications with stacking and orientations of bricks



;; another idea is to do assembly-by-disassembly (MIGHT GO AGAINST TASK INTENTION!!!)
;; => initial state is completed build and planner finds disassebly sequence that puts parts in their initial configuration