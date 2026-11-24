#!/usr/bin/env python3
"""Turn a flag profile into the value of the `feature_overrides` launch argument. No ROS needed.

  python3 tools/flag_profile.py profiles/comp_2026.yaml
  python3 tools/flag_profile.py profiles/<name>.yaml --sequence package_drop_live.yaml
  python3 tools/flag_profile.py --list

Prints the profile's flags, comma-separated (an empty line for a profile that
enables nothing), after checking the profile: flag names, no flag twice, and
the dependencies in FLAG_REQUIRES. With --sequence it also checks that the
sequence to be flown declares every flag, which the executor will insist on
at start-up.

Use it in two steps, so that a failure stops you before the launch:

  flags=$(python3 tools/flag_profile.py profiles/<name>.yaml --sequence package_drop_live.yaml) || exit
  ros2 launch drone_mission_demo package_drop_live.launch.py feature_overrides:="$flags"

If anything goes wrong (a bad profile, a bad option, a sequence that cannot
be read) it exits with code 2, says why on stderr, and prints PROFILE_ERROR
on stdout. That word is not a flag name, so even a launch line that ignores
the exit code is refused by the executor instead of starting with every flag
off while the operator believes the profile is on. The one case nothing here
can catch is the tool not running at all (a wrong path): hence the `|| exit`.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "drone_mission_core"))

from drone_mission_core.features import FeatureFlagError, load_profile, resolve_features  # noqa: E402

PROFILES_DIR = ROOT / "profiles"
PROFILE_ERROR = "PROFILE_ERROR"  # upper case: parse_feature_overrides refuses it
SEQUENCES_DIR = ROOT / "src" / "drone_mission_demo" / "config" / "sequences"


def declared_flags(sequence: str) -> dict:
    import yaml

    path = Path(sequence)
    if not path.is_file():
        path = SEQUENCES_DIR / sequence
    if not path.is_file():
        raise FeatureFlagError(f"no sequence file '{sequence}' (looked in {SEQUENCES_DIR})")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return dict((data.get("mission_sequence", data) or {}).get("features") or {})


def overrides_for(profile: Path, sequence: str | None = None) -> str:
    enabled = load_profile(profile)
    # A profile with node flags cannot be flown through feature_overrides alone: the node flags would be dropped,
    # and the mission flags would fly without them. Such a profile is started with tools/go.py (PLT-07).
    import yaml

    if (yaml.safe_load(Path(profile).read_text(encoding="utf-8")) or {}).get("nodes"):
        raise FeatureFlagError(f"{profile}: this profile has node flags (nodes:), which feature_overrides cannot carry: start the flight with tools/go.py")
    if sequence is not None:
        resolve_features(declared_flags(sequence), enabled)
    return ",".join(enabled)


def main(argv: list[str] | None = None) -> int:
    class Parser(argparse.ArgumentParser):
        def error(self, message):  # a bad option must not leave stdout empty either
            print(PROFILE_ERROR)
            super().error(message)

    parser = Parser(description="Print a flag profile as a feature_overrides value.")
    parser.add_argument("profile", nargs="?", type=Path)
    parser.add_argument("--sequence", help="a sequence file (or its name in drone_mission_demo's config) the profile will be flown with")
    parser.add_argument("--list", action="store_true", help="list the profiles in profiles/")
    args = parser.parse_args(argv)
    try:
        if args.list:
            import yaml

            for path in sorted(PROFILES_DIR.glob("*.yaml")):
                enabled = load_profile(path)
                description = (yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("description", "")
                print(f"{path.relative_to(ROOT)}: {', '.join(enabled) or 'no flags'}. {description}")
            return 0
        if args.profile is None:
            parser.error("give a profile file, or --list")
        print(overrides_for(args.profile, args.sequence))
    except Exception as error:  # noqa: BLE001  (whatever went wrong, the launch line must not get an empty string)
        print(f"flag_profile: {error}", file=sys.stderr)
        if not args.list:
            print(PROFILE_ERROR)  # see the module docstring
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
