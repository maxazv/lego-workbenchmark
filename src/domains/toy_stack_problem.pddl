(define (problem stack2-with-distractor)
  (:domain lego-coarse)
  
  (:objects
    brick_0 brick_1 brick_2 brick_3 - brick
    dim_2x2 dim_2x1 - type
    red yellow - color
    pick_a pick_b pick_c asm_base - location
  )

  (:init
    (hand-empty)

    (shape brick_0 dim_2x2)
    (is_color brick_0 red)
    (at brick_0 pick_a)  ; valid base

    (shape brick_1 dim_2x2)
    (is_color brick_1 yellow)
    (at brick_1 pick_b) ; valid top

    (shape brick_2 dim_2x2)
    (is_color brick_2 red)
    (at brick_2 pick_c) ; distractor that matches base (can choose brick_0 or brick_2)

    (shape brick_3 dim_2x1)  ; distractor, completely invalid
    (is_color brick_3 yellow)
    (at brick_3 pick_b)
  )

  (:goal
    (exists (?b0 ?b1 - brick)
      (and (at ?b0 asm_base) (shape ?b0 dim_2x2) (is_color ?b0 yellow)
           (stacked_on ?b1 ?b0) (shape ?b1 dim_2x2) (is_color ?b1 red)))
  )
)