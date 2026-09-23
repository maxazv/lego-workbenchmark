;; Reference LEGO assembly domain used to test PDDL execution in lego_sim.
;; Placeholder until the team's own domain is ready: pddl_bridge.py executes any
;; domain whose actions are listed in its SKILL_FOR_ACTION table.
(define (domain lego-assembly)
  (:requirements :adl :typing)
  (:types brick)
  (:predicates
    (in_pick_area ?b - brick)     ; lying loose on the table, graspable
    (in_asm_area ?b - brick)      ; sitting at its target pose
    (holding ?b - brick)
    (handempty)
    (on_base ?b - brick)          ; static: target pose is on the baseplate
    (supports ?s ?b - brick))     ; static: ?s lies directly under ?b in the product

  (:action pick
    :parameters (?b - brick)
    :precondition (and (handempty) (in_pick_area ?b))
    :effect (and (holding ?b) (not (handempty)) (not (in_pick_area ?b))))

  (:action place
    :parameters (?b - brick)
    :precondition (and (holding ?b) (on_base ?b))
    :effect (and (in_asm_area ?b) (handempty) (not (holding ?b))))

  ;; A brick may rest on several bricks (the bridge beam), so every supporter
  ;; has to be in place first.
  (:action stack
    :parameters (?b - brick)
    :precondition (and (holding ?b) (not (on_base ?b))
                       (forall (?s - brick)
                         (imply (supports ?s ?b) (in_asm_area ?s))))
    :effect (and (in_asm_area ?b) (handempty) (not (holding ?b)))))
