"""Judges that score a sim run the way the organisers will (SIM-08).

`judge_run` scores one mission from a run's true positions and events. See
record.py for what a run records, missions.py for the scoring rules and
README.md for the files of a run folder.
"""

from .score import MISSIONS, default_settings, judge_run  # noqa: F401
