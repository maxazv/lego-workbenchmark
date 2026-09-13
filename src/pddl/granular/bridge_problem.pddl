(define (problem bridge-with-distractor)
  (:domain lego-granular)
  (:objects
    base_L base_R bridge distractor - brick
    t2x1 t4x1 - type
    green red - color
    deg0 deg90 - orientation
    p1 p2 p3 p4 - cell
    c0_0 c1_0 c2_0 c3_0 - cell   ; ground / layer 0
    c0_1 c1_1 c2_1 c3_1 - cell)  ; layer 1

  (:init
    (is_ground c0_0) (is_ground c1_0) (is_ground c2_0) (is_ground c3_0)
    (above c0_1 c0_0) (above c1_1 c1_0) (above c2_1 c2_0) (above c3_1 c3_0)

    (shape base_L t2x1)    (is_color base_L green)    (at base_L p1 deg0)
    (shape base_R t2x1)    (is_color base_R green)    (at base_R p2 deg0)
    (shape bridge t4x1)    (is_color bridge red)       (at bridge p3 deg0)
    (shape distractor t4x1) (is_color distractor red)  (at distractor p4 deg0)

    ; base_L anchored c0_0 covers c0_0, c1_0
    (footprint t2x1 deg0 c0_0 c0_0) (footprint t2x1 deg0 c0_0 c1_0)
    ; base_R anchored c2_0 covers c2_0, c3_0
    (footprint t2x1 deg0 c2_0 c2_0) (footprint t2x1 deg0 c2_0 c3_0)
    ; bridge anchored c0_1 covers all four layer-1 cells — needs BOTH bases
    (footprint t4x1 deg0 c0_1 c0_1) (footprint t4x1 deg0 c0_1 c1_1)
    (footprint t4x1 deg0 c0_1 c2_1) (footprint t4x1 deg0 c0_1 c3_1))

  (:goal
    (exists (?bl ?br ?top - brick)
      (and (at ?bl c0_0 deg0) (shape ?bl t2x1) (is_color ?bl green)
           (at ?br c2_0 deg0) (shape ?br t2x1) (is_color ?br green)
           (at ?top c0_1 deg0) (shape ?top t4x1) (is_color ?top red)))))