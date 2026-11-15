"""Scenario files and the builder that turns one into a run folder (SIM-01).

A scenario (`sim/scenarios/<mission>/<name>.yaml`) is one test case.
`build_run` writes everything a sim run of it needs; see README.md.
"""

from .build import build_run, default_run_dir  # noqa: F401
from .spec import BuildError, Scenario, load_scenario  # noqa: F401
