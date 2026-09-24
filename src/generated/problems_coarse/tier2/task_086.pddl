(define (problem tier2_task_086)
  (:domain lego-coarse)
  (:objects
    4x2_brick_1 2x2_brick_2 - brick
    t2x2 t4x2 - type
    blue red - color
    pick_4x2_brick_1 pick_2x2_brick_2 asm_4x2_brick_1 - location
  )
  (:init
    (hand-empty)
    (shape 4x2_brick_1 t4x2)
    (is_color 4x2_brick_1 blue)
    (at 4x2_brick_1 pick_4x2_brick_1)
    (shape 2x2_brick_2 t2x2)
    (is_color 2x2_brick_2 red)
    (at 2x2_brick_2 pick_2x2_brick_2)
  )
  (:goal (and
    (exists (?b0 ?b1 - brick) (and (at ?b0 asm_4x2_brick_1) (shape ?b0 t4x2) (is_color ?b0 blue) (stacked_on ?b1 ?b0) (shape ?b1 t2x2) (is_color ?b1 red)))
  ))
)
