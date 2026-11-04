import unittest
from pathlib import Path

from drone_mission_core.features import get_feature_flag
from drone_mission_core.mission_api import MissionSpec


class GetFeatureFlagTest(unittest.TestCase):
    def test_returns_value_when_present(self) -> None:
        self.assertIs(get_feature_flag({"use_frame_time_pose": True}, "use_frame_time_pose"), True)
        self.assertIs(get_feature_flag({"use_frame_time_pose": False}, "use_frame_time_pose"), False)

    def test_missing_key_is_false(self) -> None:
        self.assertIs(get_feature_flag({"other": True}, "use_frame_time_pose"), False)

    def test_missing_block_is_false(self) -> None:
        self.assertIs(get_feature_flag({}, "use_frame_time_pose"), False)
        self.assertIs(get_feature_flag(None, "use_frame_time_pose"), False)

    def test_mission_spec_defaults_to_no_features(self) -> None:
        spec = MissionSpec(type_name="rtl", name="rtl", config={}, base_dir=Path("."))
        self.assertEqual(spec.features, {})
        self.assertIs(get_feature_flag(spec.features, "use_frame_time_pose"), False)


if __name__ == "__main__":
    unittest.main()
