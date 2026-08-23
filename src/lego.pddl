(define (domain lego)
  (:requirements :strips :typing)

  (:types brick grid_loc layer)

  (:predicates 
    (stacked_on ?a - brick ?b - brick)  ;; brick a stacked on brick b
    (graspable ?a - brick)  ;; brick a is graspable (eg no brick stacked on a)
    (clear ?l - grid_loc)  ;; brick can be placed in grid location l
    (in_pick_area ?l grid_loc)
    (in_asm_area ?l grid_loc)
    ;; (target_pose )
  )

  ;; TODO actions and predicates above

)
