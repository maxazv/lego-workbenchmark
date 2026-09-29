(define (domain lego-coarse-v2)
  (:requirements :typing)

  (:types
    ; color
    ; location  ; NOTE: we dont need target locations anymore (we prolly dont need locations altogether)
    ;type

    brick
    level
    num
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
    (holding-x)
    (holding-y)
    (supports ?b - brick ?s - brick)
    (loose ?b - brick) ; means a brick is not yet placed. counterpart to at-target, but we are not allowed to use negative preconditions

    ; Logic to enforce layer-wise precedence
    (on-level ?b - brick ?l - level)
    (next-level ?l1 - level ?l2 - level) ; l0 -> l1 -> l2 ... to make sure we ascend properly through the levels
    (succ ?n - num ?m - num) ; n0 -> n1 -> n2 ... to make sure we count properly the number of placed bricks in a level
    (zero ?n - num)

    (level-open ?l - level) ; We try to only open a level (height) after closing the previous one, in an attempt to enforce precedence. We'll see...
    (remaining ?l - level ?n - num) ; Count the remaining bricks left to place on a level until we can "unlock" the next one
  
    ; Logic to (naively) enforce valid order horizontally (placing/stacking only allowed if either no x-neighbor or y-neighbor placed to ensure gripper viability)
    (x-neighbor ?b1 brick ?b2 brick) 
    (y-neighbor ?b1 brick ?b2 brick)

    (selected ?b - brick) ; When we want to pick and then place a brick, we need to do check the x/y axis for the target position. This "starts" that process
    (x-to-be-checked ?b - brick ?n - brick) ; Set for any neighbor relationship at the beginning, a kind of "TODO" after a brick is selected
    (y-to-be-checked ?b - brick ?n - brick)
    (x-checks-remaining ?b - brick ?n - num) ; Countdown how many neighbor to be checked, initialize with number of x-neighbors in target position
    (y-checks-remaining ?b - brick ?n - num)

  )

  (:action select
    :parameters (?b - brick)
    :precondition (and
      (hand-empty)
      (loose ?b)
    )
    :effect (and
      (not (hand-empty))
      (selected ?b)
    )
  )

  (:action check-x
    :parameters (?b - brick ?n - brick ?k - num ?m - num)
    :precondition (and
      (selected ?b)
      (x-unchecked ?b ?n)
      (loose ?n)
      (x-checks-remaining ?b ?k)
      (succ ?m ?k)
    )
    :effect (and
      (not (x-unchecked ?b ?n))
      (not (x-left ?b ?k))
      (x-left ?b ?m)
    )
  )
  
  (:action check-y
    :parameters (?b - brick ?n - brick ?k - num ?m - num)
    :precondition (and
      (selected ?b)
      (y-unchecked ?b ?n)
      (loose ?n)
      (y-checks-remaining ?b ?k)
      (succ ?m ?k)
    )
    :effect (and
      (not (y-unchecked ?b ?n))
      (not (y-left ?b ?k))
      (y-left ?b ?m)
    )
  )



  (:action pick-x
    :parameters (?b - brick ?z - num)
    :precondition (and
      (selecting ?b)
      (x-left ?b ?z)
      (zero ?z)
    )
    :effect (and
      ; (not (at ?b ?from))
      (not (selecting ?b))
      (holding ?b)
      (holding-x)
      ; (clear ?from)
    )
  )

  (:action pick-y
    :parameters (?b - brick, ?z - num)
    :precondition (and
      (selecting ?b)
      (y-left ?b ?z)
      (zero ?z)
    )
    :effect (and
      ; (not (at ?b ?from))
      (not (selecting ?b))
      (holding ?b)
      (holding-y)
      ; (clear ?from)
    )
  )

  ; should only be used for root bricks
  (:action place-x
    :parameters (?b - brick ?l - level ?n - num ?m - num)
    :precondition (and
      (holding ?b)
      (is-root ?b)
      (on-level ?b ?l) ; Check if we are
      (level-open ?l)
      (remaining ?l ?n) 
      (holding-x)
      (succ ?m ?n) ; Check that m = n-1, i.e. we properly decrement remaining
    )
    :effect (and
      (not (holding ?b))
      (at-target ?b)
      (hand-empty)
      (not (remaining ?l ?n)) 
      (remaining ?l ?m)
      (not (loose ?b))
      (not (holding-x))
    )
  )

  (:action place-y
    :parameters (?b - brick ?l - level ?n - num ?m - num)
    :precondition (and
      (holding ?b)
      (is-root ?b)
      (on-level ?b ?l) ; Check if we are
      (level-open ?l)
      (remaining ?l ?n) 
      (holding-y)
      (succ ?m ?n) ; Check that m = n-1, i.e. we properly decrement remaining
    )
    :effect (and
      (not (holding ?b))
      (at-target ?b)
      (hand-empty)
      (not (remaining ?l ?n)) 
      (remaining ?l ?m)
      (not (loose ?b))
      (not (holding-y))
    )
  )

  (:action stack-x
    :parameters (?b - brick ?on - brick ?l - level ?n - num ?m - num)
    :precondition (and
      (holding ?b)
      (at-target ?on)
      (supports ?b ?on)
      (on-level ?b ?l) ; Do we still work on the correct level?
      (level-open ?l) ; Do we still work on the correct level? (Pt. 2)
      (remaining ?l ?n); Do we still have bricks left on this level?
      (succ ?m ?n); Ensure that n > 0, and also that we set the right remaining level
      (holding-x)
  ;    (top-clear ?on)
    )
    :effect (and
      (not (holding ?b))
      (not (top-clear ?on))
      (stacked_on ?b ?on)
      (at-target ?b)
      (hand-empty)
      (not (remaining ?l ?n)) 
      (remaining ?l ?m)
      (not (loose ?b))
      (not (holding-x))
    )
  )

  (:action stack-y
    :parameters (?b - brick ?on - brick ?l - level ?n - num ?m - num)
    :precondition (and
      (holding ?b)
      (at-target ?on)
      (supports ?b ?on)
      (on-level ?b ?l) ; Do we still work on the correct level?
      (level-open ?l) ; Do we still work on the correct level? (Pt. 2)
      (remaining ?l ?n); Do we still have bricks left on this level?
      (succ ?m ?n); Ensure that n > 0, and also that we set the right remaining level
      (holding-y)
  ;    (top-clear ?on)
    )
    :effect (and
      (not (holding ?b))
      (not (top-clear ?on))
      (stacked_on ?b ?on)
      (at-target ?b)
      (hand-empty)
      (not (remaining ?l ?n)) 
      (remaining ?l ?m)
      (not (loose ?b))
      (not (holding-y))
    )
  )

  (:action open-next-level
    :parameters (?l - level ?l2 - level ?z - num)
    :precondition (and 
      (level-open ?l) 
      (remaining ?l ?z) 
      (next-level ?l ?l2)
      (zero ?z)
    )
    :effect (and 
      (level-open ?l2)
    )
  )
)


; Problems:
;   - no opportunity for TAMP => if plan fails, its over