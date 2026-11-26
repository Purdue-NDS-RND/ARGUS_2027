"""The fallback (FND-05): with every flag off, the June 2026 missions and the executor load, are built from their
sequences and are ticked without one 2027 module being imported.

The tag `comp-2026` is the code that flew, and with every flag off the drone must fly as it did then. That also
means a 2027 module that cannot be imported on the Jetson (a missing package, a syntax error only one Python
sees, a file that did not copy) must not stop a 2026 mission. So the 2026 files import their 2027 helpers only
where a flag is on, and only where the sequence is built, on the ground: a flag-on flight with a broken module
does not start, and a flags-off flight never touches it.

Each test runs in a fresh interpreter in which every 2027 module refuses to import. No ROS: the stand-ins are
the ones the 2026 mission tests and the executor tests use.
"""

import ast
import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
CORE = ROOT / "src" / "drone_mission_core" / "drone_mission_core"
MISSIONS = ROOT / "src" / "drone_mission_demo" / "drone_mission_demo" / "missions"
SEQUENCES = ROOT / "src" / "drone_mission_demo" / "config" / "sequences"

# What the tag comp-2026 has in the two packages (`git ls-tree -r --name-only comp-2026`), and features.py: the
# flag reader, which is how a mission knows a flag is off, and which main had before the 2027 build began.
CORE_2026 = ("__init__", "mission_api", "mission_context", "mission_executor", "registry", "tracking_projection", "features")
MISSIONS_2026 = ("__init__", "gps_waypoint_mission", "local_waypoint_mission", "package_delivery_mission", "package_drop_mission", "red_bullseye_mission_base", "rtl_mission", "takeoff_mission")
# 2027's own packages, and the two libraries only 2027 code uses in these packages.
OTHER_2027 = ("argus_camera", "argus_interfaces", "argus_localisation", "numpy", "cv2")
MISSIONS_2027 = ("fence_demo_mission", "package_recovery_mission", "search_pattern_mission", "step_probe_mission")
TYPES_2026 = ("gps_waypoint", "local_waypoint", "package_delivery", "package_drop", "rtl", "takeoff")
TYPES_2027 = ("command_step_probe", "fence_demo", "package_recovery", "search_pattern")

PRELUDE = '''
import importlib.abc, json, sys
sys.path[:0] = [{tests!r}, {core_tests!r}, {demo!r}, {core!r}]
import yaml  # the real one, before a test module can put its stand-in there

CORE_2026, MISSIONS_2026, OTHER_2027 = {core_2026!r}, {missions_2026!r}, {other_2027!r}


def is_2027(name):
    parts = name.split(".")
    if parts[0] == "drone_mission_core":
        return len(parts) > 1 and parts[1] not in CORE_2026
    if parts[:2] == ["drone_mission_demo", "missions"]:
        return len(parts) > 2 and parts[2] not in MISSIONS_2026
    return parts[0] in OTHER_2027


class Refuse2027(importlib.abc.MetaPathFinder):
    refused = []

    def find_spec(self, name, path=None, target=None):
        if is_2027(name):
            self.refused.append(name)
            raise {error}(f"{{name}} is a 2027 module, made unimportable for this test")
        return None


sys.meta_path.insert(0, Refuse2027())


def report(**found):
    print("REPORT " + json.dumps({{"refused": sorted(set(Refuse2027.refused)), "loaded_2027": sorted(name for name in sys.modules if is_2027(name)), **found}}))
'''


def blocked(code: str, error: str = "ModuleNotFoundError") -> dict:
    """Run `code` in a fresh interpreter where every 2027 module raises `error` on import; what it reported."""
    prelude = PRELUDE.format(
        tests=str(Path(__file__).resolve().parent), core_tests=str(ROOT / "src" / "drone_mission_core" / "test"), demo=str(ROOT / "src" / "drone_mission_demo"),
        core=str(ROOT / "src" / "drone_mission_core"), core_2026=CORE_2026, missions_2026=MISSIONS_2026, other_2027=OTHER_2027, error=error,
    )  # fmt: skip
    env = {name: value for name, value in os.environ.items() if name not in ("PYTHONPATH", "AMENT_PREFIX_PATH", "ROS_DISTRO", "ROS_VERSION")}
    ran = subprocess.run([sys.executable, "-c", prelude + textwrap.dedent(code)], capture_output=True, text=True, cwd=str(ROOT), env=env, timeout=120)
    lines = [line for line in ran.stdout.splitlines() if line.startswith("REPORT ")]
    assert ran.returncode == 0 and lines, f"exit {ran.returncode}\n{ran.stdout[-2000:]}\n{ran.stderr[-4000:]}"
    return json.loads(lines[-1][len("REPORT ") :])


def test_the_lists_are_the_files():
    """Every module of the two packages is on one side or the other: a new one is 2027's unless it is put here."""
    for name in CORE_2026:
        assert (CORE / f"{name}.py").is_file(), name
    for name in MISSIONS_2026 + MISSIONS_2027:
        assert (MISSIONS / f"{name}.py").is_file(), name
    new_core = sorted(path.stem for path in CORE.glob("*.py") if path.stem not in CORE_2026)
    assert len(new_core) > 30 and {"centring", "time_guard", "target_access", "gps_health", "estimator_feed", "safety_limits"} <= set(new_core)
    assert set(MISSIONS_2027) < {path.stem for path in MISSIONS.glob("*.py")} - set(MISSIONS_2026)


def test_the_refusal_refuses():
    found = blocked(
        """
        import importlib
        said = {}
        for name in ("drone_mission_core.centring", "drone_mission_core.time_guard", "drone_mission_core.estimator_feed", "numpy", "argus_camera"):
            try:
                importlib.import_module(name)
                said[name] = "imported"
            except ImportError as problem:
                said[name] = str(problem)
        import drone_mission_core.features, drone_mission_core.tracking_projection, drone_mission_core.mission_api
        report(said=said)
        """
    )
    assert all("made unimportable" in why for why in found["said"].values()) and len(found["said"]) == 5 and found["loaded_2027"] == []


def _top_level_imports(path: Path):
    """(module named, line) for every import a file makes when it is loaded: the module's own statements, not a function's."""
    tree = ast.parse(path.read_text())
    found = []

    def walk(statements):
        for node in statements:
            if isinstance(node, ast.Import):
                found.extend((alias.name, node.lineno) for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                found.append(("." * node.level + (node.module or ""), node.lineno))
                if not node.module:  # "from . import x": the names are the modules
                    found.extend(("." * node.level + alias.name, node.lineno) for alias in node.names)
            elif isinstance(node, (ast.If, ast.Try, ast.With, ast.For, ast.While)):
                if isinstance(node, ast.If) and "TYPE_CHECKING" in ast.dump(node.test):
                    continue  # never run
                for field in ("body", "orelse", "finalbody"):
                    walk(getattr(node, field, []))
                for handler in getattr(node, "handlers", []):
                    walk(handler.body)

    walk(tree.body)
    return found


@pytest.mark.parametrize("folder, package, allowed", [(CORE, "drone_mission_core", CORE_2026), (MISSIONS, "drone_mission_demo.missions", MISSIONS_2026)], ids=["core", "missions"])
def test_no_2026_file_imports_a_2027_module_when_it_is_loaded(folder, package, allowed):
    for name in allowed:
        for module, line in _top_level_imports(folder / f"{name}.py"):
            if module.startswith("."):
                module = f"{package}.{module.lstrip('.')}"
            parts = module.split(".")
            if parts[0] == "drone_mission_core" and len(parts) > 1:
                assert parts[1] in CORE_2026, f"{name}.py:{line} imports {module} when it is loaded"
            elif module.startswith("drone_mission_demo.missions.") and name != "__init__":
                assert parts[2] in MISSIONS_2026, f"{name}.py:{line} imports {module} when it is loaded"
            else:
                assert parts[0] not in OTHER_2027, f"{name}.py:{line} imports {module} when it is loaded"


def _imports_in_functions(path: Path):
    """(function, module named, line) for every import inside a function or method of a file."""
    found = []
    for function in ast.walk(ast.parse(path.read_text())):
        if isinstance(function, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for node in ast.walk(function):
                if isinstance(node, ast.ImportFrom):
                    found.append((function.name, "." * node.level + (node.module or "") + ("" if node.module else node.names[0].name), node.lineno))
                elif isinstance(node, ast.Import):
                    found.extend((function.name, alias.name, node.lineno) for alias in node.names)
    return found


def test_a_2026_mission_imports_a_2027_module_only_where_it_is_built_on_the_ground():
    """A mission is built when the executor starts, on the ground. An import that first ran in on_enter or update
    would be a module found broken in the air: a 2027 import in a 2026 mission is in its constructor and nowhere
    else. (The missions' _load_... helpers are called from on_enter, so they are not the constructor's.)"""
    for name in MISSIONS_2026[1:]:
        for function, module, line in _imports_in_functions(MISSIONS / f"{name}.py"):
            parts = module.lstrip(".").split(".")
            is_new = (module.startswith(".") and parts[0] not in MISSIONS_2026) or (parts[0] == "drone_mission_core" and len(parts) > 1 and parts[1] not in CORE_2026)
            if is_new:
                assert function == "__init__", f"{name}.py:{line}: {function}() imports {module}"


@pytest.mark.parametrize("tests, count", [("test_package_drop_mission", 4), ("test_gps_waypoint_mission", 9)])
def test_the_2026_tests_of_a_mission_pass_with_every_2027_module_unimportable(tests, count):
    """The two mission test files the tag has, as they are: they load the mission and tick it through its states with no flag on."""
    found = blocked(
        f"""
        import io, unittest
        import {tests} as tests
        result = unittest.TextTestRunner(stream=io.StringIO()).run(unittest.defaultTestLoader.loadTestsFromModule(tests))
        report(ran=result.testsRun, ok=result.wasSuccessful(), problems=[text for _test, text in result.errors + result.failures])
        """
    )
    assert found["ok"] and found["ran"] == count, found["problems"]
    assert found["loaded_2027"] == []
    # What was asked for and refused: only the 2027 missions the package offers beside the 2026 ones.
    assert set(found["refused"]) <= {f"drone_mission_demo.missions.{name}" for name in MISSIONS_2027}


BUILD_SEQUENCES = """
    import test_package_drop_mission  # the ROS stand-ins
    from pathlib import Path
    from drone_mission_core import registry
    registry.import_mission_modules(["drone_mission_demo.missions"])
    built = {}
    for file in sorted(Path(%r).glob("*.yaml")):
        try:
            specs = registry.load_sequence_file(str(file))
            flags = sorted(flag for flag, value in specs[0].features.items() if value)
            built[file.name] = {"flags": flags, "missions": [type(registry.create_mission(spec)).__name__ for spec in specs]}
        except Exception as problem:
            built[file.name] = {"error": f"{type(problem).__name__}: {problem}"}
    report(built=built, types=sorted(registry.MISSION_REGISTRY), failures=dict(registry.IMPORT_FAILURES))
"""


@pytest.mark.parametrize("error", ["ModuleNotFoundError", "RuntimeError"])
def test_every_2026_sequence_is_built_and_a_2027_mission_that_cannot_load_takes_only_itself(error):
    """The package offers the 2027 missions beside the 2026 ones. One that cannot be imported, whatever it raises,
    is left out and said; a sequence that names it does not start, and says why."""
    found = blocked(BUILD_SEQUENCES % str(SEQUENCES), error)
    assert found["types"] == sorted(TYPES_2026) and found["loaded_2027"] == []
    assert sorted(found["failures"]) == sorted(f"drone_mission_demo.missions.{name}" for name in MISSIONS_2027)
    assert all(why.startswith(f"{error}: ") and "made unimportable" in why for why in found["failures"].values())
    import yaml

    for file in sorted(SEQUENCES.glob("*.yaml")):
        data = yaml.safe_load(file.read_text())["mission_sequence"]
        types = {str(mission["type"]).strip().lower() for mission in data["missions"]}
        result = found["built"][file.name]
        assert not any((data.get("features") or {}).values()), f"{file.name} is committed with a flag on"
        if types <= set(TYPES_2026):
            assert "error" not in result and len(result["missions"]) == len(data["missions"]) and result["flags"] == [], (file.name, result)
        else:
            new = sorted(types & set(TYPES_2027))[0]
            assert f"Unknown mission type '{new}'" in result["error"] and "could not be imported" in result["error"] and "made unimportable" in result["error"], (file.name, result)
    for flown in ("package_drop_live.yaml", "package_delivery_live.yaml", "final_waypoint_drop.yaml", "final_curcuit_delivery.yaml", "waypoint_navigation_live.yaml", "circuit_time_trial_live.yaml"):
        assert "missions" in found["built"][flown], flown  # what flew in June 2026


DELIVERY = """
    import test_package_drop_mission as harness
    from types import SimpleNamespace
    from drone_mission_core import registry
    registry.import_mission_modules(["drone_mission_demo.missions"])
    spec = next(spec for spec in registry.load_sequence_file(%r) if spec.type_name == "package_delivery")
    mission = registry.create_mission(spec)


    class Context(harness.FakeContext):
        # A vehicle that does as it is told: it is where its setpoints put it, it sees the target under it, and it lands.

        def __init__(self):
            super().__init__()
            self.gripper, self.modes, self.climbs = [], [], 0
            self.image_size = (3840, 2160)

        def target_cv_control_ready(self):
            return True

        def set_target_cv_enabled(self, enabled, done_callback=None):
            if done_callback is not None:
                done_callback(harness.FakeFuture(success=True))

        def clear_target_tracking_state(self):
            pass

        def landing_state_available(self):
            return True

        def vehicle_is_landed(self):
            return self.local_pose.pose.position.z <= 0.05

        def mode_service_ready(self):
            return True

        def request_mode_change(self, mode, done_callback=None):
            self.modes.append(mode)
            future = harness.FakeFuture(success=True)
            if done_callback is not None:
                done_callback(future)
            return future

        def set_attitude_climb_rate_setpoint(self, *args, **kwargs):
            self.climbs += 1
            self.local_pose.pose.position.z += 0.05

        def set_local_velocity_setpoint(self, east, north, up=0.0, yaw_deg=90.0):
            super().set_local_velocity_setpoint(east, north, up, yaw_deg)
            self.local_pose.pose.position.z = max(0.0, self.local_pose.pose.position.z + up * 0.05)

        def set_global_position_setpoint(self, latitude, longitude, altitude_m, yaw_deg=90.0, lock_yaw=True):
            super().set_global_position_setpoint(latitude, longitude, altitude_m, yaw_deg, lock_yaw)
            here = self.local_pose.pose.position.z
            self.local_pose.pose.position.z = max(0.0, here + max(-0.05, min(0.05, altitude_m - here)))

        def command_gripper(self, *, release, done_callback=None):
            self.gripper.append(release)
            future = harness.FakeFuture(success=True)
            if done_callback is not None:
                done_callback(future)
            return future


    context = Context()
    context.global_gps = SimpleNamespace(latitude=spec.config["target_latitude"], longitude=spec.config["target_longitude"], altitude=0.0, status=SimpleNamespace(status=0))
    context.local_pose.pose.position.z = spec.config["transit_altitude_m"]
    mission.on_enter(context)
    states, status = [], None
    for tick in range(6000):
        # The target is in the middle of every frame, and every frame is this tick's.
        context.target_detection = SimpleNamespace(header=SimpleNamespace(stamp=context.now()), point=SimpleNamespace(x=0.0, y=0.0, z=0.0))
        status = mission.update(context)
        state = mission._state.name
        if not states or states[-1] != state:
            states.append(state)
        if status.name in ("SUCCESS", "FAILURE"):
            break
        context.advance(0.05)
    mission.on_exit(context)
    report(states=states, status=status.name, ticks=tick, gripper=context.gripper, climbs=context.climbs, setpoints=len(context.global_position_commands) + len(context.local_velocity_commands))
"""


def test_the_2026_delivery_flies_start_to_finish_with_every_2027_module_unimportable():
    """The tag has no test of the delivery. It is built here from the sequence that flew and ticked at 20 Hz on a
    vehicle that does as it is told, from the transit to the relaunch: no state's code may reach for a 2027 module."""
    found = blocked(DELIVERY % str(SEQUENCES / "package_delivery_live.yaml"))
    assert found["states"] == ["WAITING_FOR_CONNECTION", "WAITING_FOR_GPS", "TRANSIT_TO_TARGET", "ACQUIRE_TARGET", "TRACK_AND_DESCEND", "FINAL_FIXED_COLUMN_DESCENT", "GROUND_DWELL", "GUIDED_RELAUNCH", "COMPLETE"], found
    assert found["status"] == "SUCCESS" and found["gripper"] == [True] and found["climbs"] > 0  # fake_drop is false in the live sequence: the gripper is commanded once
    assert found["loaded_2027"] == []
    assert set(found["refused"]) <= {f"drone_mission_demo.missions.{name}" for name in MISSIONS_2027}


EXECUTOR = """
    import types
    from unittest import mock
    import test_executor_feature_overrides as harness

    stand_ins = {
        "rclpy": mock.MagicMock(), "rclpy.node": types.SimpleNamespace(Node=harness.FakeNode), "rclpy.parameter": mock.MagicMock(), "rclpy.qos": mock.MagicMock(),
        "rclpy.time": mock.MagicMock(), "rclpy.task": mock.MagicMock(), "rclpy.impl": mock.MagicMock(), "rclpy.impl.rcutils_logger": mock.MagicMock(),
        "geometry_msgs": mock.MagicMock(), "geometry_msgs.msg": mock.MagicMock(), "mavros_msgs": mock.MagicMock(), "mavros_msgs.msg": mock.MagicMock(),
        "mavros_msgs.srv": mock.MagicMock(), "sensor_msgs": mock.MagicMock(), "sensor_msgs.msg": mock.MagicMock(), "std_srvs": mock.MagicMock(), "std_srvs.srv": mock.MagicMock(),
    }
    sys.modules.update(stand_ins)
    from drone_mission_core import mission_executor

    harness.FakeNode.overrides = {"sequence_file": %r, "mission_modules": ["drone_mission_demo.missions"], "feature_overrides": %r}
    try:
        node = mission_executor.MissionExecutorNode()
    except Exception as problem:
        report(started=False, error=f"{type(problem).__name__}: {problem}")
    else:
        for _ in range(5):
            node._control_loop()
        errors = [str(call.args[0]) for call in node.logger.error.call_args_list]
        report(started=True, missions=[mission.spec.type_name for mission in node._sequence], errors=errors,
               attached=sorted(name for name in ("_pose_buffer", "_ground_height", "_height_estimator", "_throttle", "_home_height", "_mission_upload", "_localisation_control_client",
                                                  "_marker_tracks", "_gps_health", "_target_estimator", "_precision_loiter", "_time_health", "_takeover", "_actuation_lock")
                               if getattr(node, name) is not None))
"""


@pytest.mark.parametrize("sequence", ["package_delivery_live.yaml", "package_drop_live.yaml", "waypoint_navigation_live.yaml"])
def test_the_executor_starts_and_ticks_a_2026_sequence_with_every_2027_module_unimportable(sequence):
    found = blocked(EXECUTOR % (str(SEQUENCES / sequence), ""))
    assert found.get("started"), found
    assert found["missions"][0] == "takeoff" and found["missions"][-1] == "rtl" and found["attached"] == [] and found["loaded_2027"] == []
    # It says which 2027 missions it could not offer, once each, and nothing else is wrong.
    assert len(found["errors"]) == len(MISSIONS_2027) and all("could not be imported" in line for line in found["errors"]), found["errors"]
    assert set(found["refused"]) == {f"drone_mission_demo.missions.{name}" for name in MISSIONS_2027}


def test_with_a_flag_on_a_2027_module_that_cannot_load_stops_the_executor_on_the_ground():
    """Not in the air: the flag's modules are imported while the sequence is built, and the executor does not start."""
    for flags in ("use_frame_time_pose", "time_health_guard", "takeover_latch", "enforce_safety_limits"):
        found = blocked(EXECUTOR % (str(SEQUENCES / "package_delivery_live.yaml"), flags))
        assert found.get("started") is False and "made unimportable" in found["error"], (flags, found)


def test_with_a_flag_on_a_mission_reaches_for_its_2027_module_when_it_is_built():
    found = blocked(
        """
        import test_package_drop_mission as harness
        from pathlib import Path
        from drone_mission_core.mission_api import MissionSpec
        said = {}
        for flag in ("time_health_guard", "centre_on_estimator", "centring_retune", "drop_release_gates", "use_frame_time_pose", "smart_reacquire"):
            spec = MissionSpec(type_name="package_drop", name="package_drop", config={"target_latitude": 1.0, "target_longitude": 1.0}, base_dir=Path("."), features={flag: True})
            try:
                harness.PackageDropMission(spec)
                said[flag] = "built"
            except Exception as problem:
                said[flag] = f"{type(problem).__name__}: {problem}"
        report(said=said)
        """
    )
    assert all("made unimportable" in why for why in found["said"].values()), found["said"]


def test_the_flag_names_the_2026_missions_carry_are_their_modules_own():
    """A 2026 mission cannot import a flag's name from the flag's module without importing the module, so it carries
    the name. Each is held here to the constant in the module that owns it."""
    import importlib
    import re

    sys.path[:0] = [str(ROOT / "src" / "drone_mission_core")]
    carried = {
        "red_bullseye_mission_base.py": {
            "FRAME_TIME_FLAG": ("frame_time", "FRAME_TIME_FLAG"), "CENTRE_FLAG": ("centring", "CENTRE_FLAG"), "RETUNE_FLAG": ("centring_controller", "RETUNE_FLAG"),
            "WIND_FLAG": ("wind_compensation", "FLAG"), "REACQUIRE_FLAG": ("reacquire", "REACQUIRE_FLAG"), "TIME_HEALTH_FLAG": ("time_guard", "TIME_HEALTH_FLAG"),
        },
        "package_drop_mission.py": {"GATES_FLAG": ("release_gates", "GATES_FLAG")},
        "package_delivery_mission.py": {"SET_DOWN_FLAG": ("set_down", "SET_DOWN_FLAG")},
    }  # fmt: skip
    for file, names in carried.items():
        text = (MISSIONS / file).read_text()
        assert sorted(re.findall(r"^([A-Z_]+_FLAG) = \"", text, re.MULTILINE)) == sorted(names), file
        for name, (module, constant) in names.items():
            value = re.search(rf'^{name} = "([a-z_]+)"', text, re.MULTILINE).group(1)
            assert value == getattr(importlib.import_module(f"drone_mission_core.{module}"), constant), (file, name)


def test_a_mission_module_that_fails_after_its_class_leaves_nothing_registered(tmp_path):
    """The flight-safety review: a module registers its mission where its class is, so one that fell over further down
    would have left the type registered on a half-loaded module, and a sequence naming it would have started and
    failed in the air. A failed optional import takes its registrations back, and the sequence does not start."""
    (tmp_path / "half_loaded_mission.py").write_text(
        "from drone_mission_core.mission_api import BaseMission\n"
        "from drone_mission_core.registry import register_mission\n\n\n"
        "@register_mission('half_loaded')\n"
        "class HalfLoaded(BaseMission):\n"
        "    def on_enter(self, context):\n        pass\n\n"
        "    def update(self, context):\n        return None\n\n"
        "    def on_exit(self, context):\n        pass\n\n\n"
        "raise RuntimeError('fell over after its class')\n"
    )
    (tmp_path / "whole_mission.py").write_text((tmp_path / "half_loaded_mission.py").read_text().replace("half_loaded", "whole").replace("HalfLoaded", "Whole").replace("raise RuntimeError('fell over after its class')\n", ""))
    found = blocked(
        f"""
        sys.path.insert(0, {str(tmp_path)!r})
        from pathlib import Path
        from drone_mission_core import registry
        from drone_mission_core.mission_api import MissionSpec
        gone = registry.import_optional_mission("half_loaded_mission")
        kept = registry.import_optional_mission("whole_mission")
        try:
            registry.create_mission(MissionSpec(type_name="half_loaded", name="x", config={{}}, base_dir=Path(".")))
            said = "built"
        except ValueError as problem:
            said = str(problem)
        built = type(registry.create_mission(MissionSpec(type_name="whole", name="y", config={{}}, base_dir=Path(".")))).__name__
        report(gone=gone is None, kept=kept is not None, types=sorted(registry.MISSION_REGISTRY), failures=dict(registry.IMPORT_FAILURES), said=said, built=built)
        """
    )
    assert found["gone"] and found["kept"] and found["types"] == ["whole"] and found["built"] == "Whole"
    assert found["failures"] == {"half_loaded_mission": "RuntimeError: fell over after its class"}
    assert "Unknown mission type 'half_loaded'" in found["said"] and "half_loaded_mission could not be imported (RuntimeError: fell over after its class)" in found["said"]

