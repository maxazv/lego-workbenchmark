(define (problem tier2_task_040)
  (:domain lego-coarse)
  (:objects
    4x2_brick_1 2x2_brick_2 2x2_brick_3 4x2_brick_4 - brick
    t2x2 t4x2 - type
    red yellow - color
    pick_4x2_brick_1 pick_2x2_brick_2 pick_2x2_brick_3 pick_4x2_brick_4 asm_4x2_brick_1 - location
  )
  (:init
    (hand-empty)
    (shape 4x2_brick_1 t4x2)
    (is_color 4x2_brick_1 yellow)
    (at 4x2_brick_1 pick_4x2_brick_1)
    (shape 2x2_brick_2 t2x2)
    (is_color 2x2_brick_2 yellow)
    (at 2x2_brick_2 pick_2x2_brick_2)
    (shape 2x2_brick_3 t2x2)
    (is_color 2x2_brick_3 yellow)
    (at 2x2_brick_3 pick_2x2_brick_3)
    (shape 4x2_brick_4 t4x2)
    (is_color 4x2_brick_4 red)
    (at 4x2_brick_4 pick_4x2_brick_4)
  )
  (:goal (and
    (exists (?b0 ?b1 ?b2 ?b3 - brick) (and (at ?b0 asm_4x2_brick_1) (shape ?b0 t4x2) (is_color ?b0 yellow) (stacked_on ?b1 ?b0) (shape ?b1 t2x2) (is_color ?b1 yellow) (stacked_on ?b2 ?b1) (shape ?b2 t2x2) (is_color ?b2 yellow) (stacked_on ?b3 ?b2) (shape ?b3 t4x2) (is_color ?b3 red)))
  ))
)
