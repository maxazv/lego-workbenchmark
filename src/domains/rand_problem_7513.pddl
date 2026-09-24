
(define (problem rand_problem_7513)
  (:domain lego-coarse)

  (:objects
    brick_0 brick_1 brick_2 - brick
    1x1 2x2 - type
    blue yellow red - color
    pick_0 pick_1 pick_2 asm_0 - location
  )

  (:init
    (hand-empty)

    (shape brick_0 1x1)
    (is_color brick_0 yellow)
    (at brick_0 pick_0)

    (shape brick_1 2x2)
    (is_color brick_1 blue)
    (at brick_1 pick_1)

    (shape brick_2 1x1)
    (is_color brick_2 red)
    (at brick_2 pick_2)
  )

  (:goal
    (exists (?b0 ?b1 ?b2 - brick)
      (and
           (shape ?b0 1x1) (is_color ?b0 yellow) (at ?b0 asm_0)
           (shape ?b1 1x1) (is_color ?b1 red) (stacked_on ?b1 brick_0)
           (shape ?b2 2x2) (is_color ?b2 blue) (stacked_on ?b2 brick_2)
      )
    )
  )
)
