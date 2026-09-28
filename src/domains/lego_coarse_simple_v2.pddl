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
    (supports ?b - brick ?s - brick)
    (loose ?b - brick) ; means a brick is not yet placed. counterpart to at-target, but we are not allowed to use negative preconditions

    (on-level ?b - brick ?l - level)
    (next-level ?l1 - level ?l2 - level) ; l0 -> l1 -> l2 ... to make sure we ascend properly through the levels
    (succ ?n - num ?m - num) ; n0 -> n1 -> n2 ... to make sure we count properly the number of placed bricks in a level
    (zero ?n - num)

    (level-open ?l - level) ; We try to only open a level (height) after closing the previous one, in an attempt to enforce precedence. We'll see...
    (remaining ?l - level ?n - num) ; Count the remaining bricks left to place on a level until we can "unlock" the next one
  )

  (:action pick
    :parameters (?b - brick)
    :precondition (and
      (hand-empty)
      (loose ?b)
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
    :parameters (?b - brick ?l - level ?n - num ?m - num)
    :precondition (and
      (holding ?b)
      (is-root ?b)
      (on-level ?b ?l) ; Check if we are
      (level-open ?l)
      (remaining ?l ?n) 
      (succ ?m ?n) ; Check that m = n-1, i.e. we properly decrement remaining
    )
    :effect (and
      (not (holding ?b))
      (at-target ?b)
      (hand-empty)
      (not (remaining ?l ?n)) 
      (remaining ?l ?m)
      (not (loose ?b))
    )
  )

  (:action stack
    :parameters (?b - brick ?on - brick ?l - level ?n - num ?m - num)
    :precondition (and
      (holding ?b)
      (at-target ?on)
      (supports ?b ?on)
      (on-level ?b ?l) ; Do we still work on the correct level?
      (level-open ?l) ; Do we still work on the correct level? (Pt. 2)
      (remaining ?l ?n); Do we still have bricks left on this level?
      (succ ?m ?n); Ensure that n > 0, and also that we set the right remaining level
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