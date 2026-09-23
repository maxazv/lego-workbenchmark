"""Execution of PDDL assembly plans in TAMPanda via DomainBridge + PickPlaceExecutor.

Run the scripts as modules from ``src/`` so that ``tampanda`` and the shared loader resolve:

  python -m lego_tampanda.run_task off_tier2_task_001 --view
  python -m lego_tampanda.run_batch t1.csv 0,1,2 off_tier1_task_001 off_tier1_task_002
  python -m lego_tampanda.test_pick_place
"""
