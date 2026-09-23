
(define (problem rand_problem_186)
  (:domain lego-coarse)

  (:objects
    brick_0 brick_1 brick_2 brick_3 brick_4 - brick
    1x1 2x2 - type
    blue red - color
    pick_0 pick_1 pick_2 pick_3 pick_4 asm_0 asm_1 asm_2 - location
  )

  (:init
    (hand-empty)

    (shape brick_0 2x2)
    (is_color brick_0 blue)
    (at brick_0 pick_0)

    (shape brick_1 2x2)
    (is_color brick_1 red)
    (at brick_1 pick_1)

    (shape brick_2 2x2)
    (is_color brick_2 red)
    (at brick_2 pick_2)

    (shape brick_3 1x1)
    (is_color brick_3 blue)
    (at brick_3 pick_3)

    (shape brick_4 2x2)
    (is_color brick_4 red)
    (at brick_4 pick_4)
  )

  (:goal
    (exists (?b0 ?b1 ?b2 ?b3 - brick)
      (and
           (shape ?b0 2x2) (is_color ?b0 red) (at ?b0 asm_0)
           (shape ?b1 2x2) (is_color ?b1 red) (at ?b1 asm_1)
           (shape ?b2 2x2) (is_color ?b2 red) (at ?b2 asm_2)
           (shape ?b3 2x2) (is_color ?b3 blue) (stacked_on ?b3 brick_2)
      )
    )
  )
)
