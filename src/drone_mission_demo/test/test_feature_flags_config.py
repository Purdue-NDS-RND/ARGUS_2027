"""Every shipped params and sequence YAML keeps its feature flags off.

Flags are switched on deliberately in a later change, never by default. Params
files are ROS --params-file files: a top-level features: key would stop rcl
from parsing them, so node flags live under <node>.ros__parameters instead.
"""

import sys
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "drone_mission_core"))
for module_name in list(sys.modules):
    if module_name == "drone_mission_core" or module_name.startswith(
        "drone_mission_core."
    ):
        del sys.modules[module_name]

from drone_mission_core.registry import load_sequence_file  # noqa: E402

CONFIG = ROOT / "src" / "drone_mission_demo" / "config"
PARAMS_FILES = sorted((CONFIG / "params").glob("*.yaml"))
SEQUENCE_FILES = sorted((CONFIG / "sequences").glob("*.yaml"))


def enabled_features(node, path=()):
    """Yield (path, value) for every value under a features: mapping that isn't False."""
    if isinstance(node, dict):
        for key, value in node.items():
            here = path + (str(key),)
            if key == "features" and isinstance(value, dict):
                yield from _non_false_leaves(value, here)
            else:
                yield from enabled_features(value, here)
    elif isinstance(node, list):
        for index, item in enumerate(node):
            yield from enabled_features(item, path + (str(index),))


def _non_false_leaves(node, path):
    if isinstance(node, dict):
        for key, value in node.items():
            yield from _non_false_leaves(value, path + (str(key),))
    elif node is not False:
        yield path, node


class FeatureFlagsOffTest(unittest.TestCase):
    def test_config_files_found(self) -> None:
        self.assertTrue(PARAMS_FILES)
        self.assertTrue(SEQUENCE_FILES)

    def test_no_feature_is_on(self) -> None:
        for path in PARAMS_FILES + SEQUENCE_FILES:
            with self.subTest(file=path.relative_to(ROOT)):
                data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
                found = [f"{'.'.join(p)} = {v!r}" for p, v in enabled_features(data)]
                self.assertEqual(found, [], "features must ship as false")

    def test_params_files_have_no_top_level_features(self) -> None:
        for path in PARAMS_FILES:
            with self.subTest(file=path.relative_to(ROOT)):
                data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
                self.assertNotIn("features", data)

    def test_sequence_features_reach_every_mission_spec(self) -> None:
        for path in SEQUENCE_FILES:
            with self.subTest(file=path.relative_to(ROOT)):
                data = yaml.safe_load(path.read_text(encoding="utf-8"))
                expected = data["mission_sequence"].get("features") or {}
                for spec in load_sequence_file(str(path)):
                    self.assertEqual(spec.features, expected)

    def test_walker_catches_enabled_flags(self) -> None:
        data = {
            "node": {"ros__parameters": {"features": {"a": False, "b": True}}},
            "mission_sequence": {"features": {"c": "false", "d": 0, "e": None}},
            "list": [{"features": {"deep": {"f": True}}}],
        }
        found = {".".join(p) for p, _ in enabled_features(data)}
        self.assertEqual(
            found,
            {
                "node.ros__parameters.features.b",
                "mission_sequence.features.c",
                "mission_sequence.features.d",
                "mission_sequence.features.e",
                "list.0.features.deep.f",
            },
        )


if __name__ == "__main__":
    unittest.main()
