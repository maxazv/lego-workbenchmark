(define (problem tier2_task_028)
  (:domain lego-coarse)
  (:objects
    4x2_brick_1 2x2_brick_2 - brick
    t2x2 t4x2 - type
    red yellow - color
    pick_4x2_brick_1 pick_2x2_brick_2 asm_4x2_brick_1 - location
  )
  (:init
    (hand-empty)
    (shape 4x2_brick_1 t4x2)
    (is_color 4x2_brick_1 red)
    (at 4x2_brick_1 pick_4x2_brick_1)
    (shape 2x2_brick_2 t2x2)
    (is_color 2x2_brick_2 yellow)
    (at 2x2_brick_2 pick_2x2_brick_2)
  )
  (:goal (and
    (exists (?b0 ?b1 - brick) (and (at ?b0 asm_4x2_brick_1) (shape ?b0 t4x2) (is_color ?b0 red) (stacked_on ?b1 ?b0) (shape ?b1 t2x2) (is_color ?b1 yellow)))
  ))
)
