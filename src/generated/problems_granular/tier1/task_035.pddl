(define (problem tier1_task_035)
  (:domain lego-granular)
  (:objects
    2x2_brick_1 2x2_brick_2 - brick
    t2x2 - type
    red - color
    deg0 - orientation
    p_2x2_brick_1 p_2x2_brick_2 - cell  ; pick cells
    c_n1_8_0 c_n1_9_0 c_0_8_0 c_0_9_0 - cell  ; layer 0
    c_n1_8_1 c_n1_9_1 c_0_8_1 c_0_9_1 - cell  ; layer 1
  )
  (:init
    (shape 2x2_brick_1 t2x2)
    (is_color 2x2_brick_1 red)
    (at 2x2_brick_1 p_2x2_brick_1 deg0)
    (shape 2x2_brick_2 t2x2)
    (is_color 2x2_brick_2 red)
    (at 2x2_brick_2 p_2x2_brick_2 deg0)
    (footprint t2x2 deg0 c_n1_8_0 c_n1_8_0)
    (footprint t2x2 deg0 c_n1_8_0 c_n1_9_0)
    (footprint t2x2 deg0 c_n1_8_0 c_0_8_0)
    (footprint t2x2 deg0 c_n1_8_0 c_0_9_0)
    (footprint t2x2 deg0 c_n1_8_1 c_n1_8_1)
    (footprint t2x2 deg0 c_n1_8_1 c_n1_9_1)
    (footprint t2x2 deg0 c_n1_8_1 c_0_8_1)
    (footprint t2x2 deg0 c_n1_8_1 c_0_9_1)
    (is_ground c_n1_8_0)
    (above c_n1_8_1 c_n1_8_0)
    (is_ground c_n1_9_0)
    (above c_n1_9_1 c_n1_9_0)
    (is_ground c_0_8_0)
    (above c_0_8_1 c_0_8_0)
    (is_ground c_0_9_0)
    (above c_0_9_1 c_0_9_0)
  )
  (:goal (and
    (exists (?b - brick) (and (at ?b c_n1_8_0 deg0) (shape ?b t2x2) (is_color ?b red)))
    (exists (?b - brick) (and (at ?b c_n1_8_1 deg0) (shape ?b t2x2) (is_color ?b red)))
  ))
)
