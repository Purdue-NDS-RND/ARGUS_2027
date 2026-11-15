"""The headless scenario runner (SIM-07): fly a built scenario, record it, report it.

sim/run_scenarios.sh drives it from the host. Inside the sim container,
fly.py records true poses and events while the mission flies; the modules
here hold the decisions, free of ROS so they can be tested anywhere.
"""
