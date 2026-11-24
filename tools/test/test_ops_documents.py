"""OPS-09: the SOP, emergency procedures and checklists, written from the code (tools/ops_documents.py). No ROS."""

import dataclasses
import re
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))

import ops_documents as od  # noqa: E402
from drone_mission_core import actuation_lock  # noqa: E402  (on the path through ops_documents)
from drone_utils import preflight  # noqa: E402

DOCS = REPO / "docs" / "ops"
NAMES = ("sop.md", "emergency_procedures.md", "checklists.md")


def test_the_committed_documents_are_what_the_tool_writes_now(capsys):
    """The point of the row: a limit changed in the preflight, or a failsafe in the baseline, fails this until the documents are written again."""
    assert od.main(["--check"]) == 0
    assert capsys.readouterr().out.strip() == "documents: as generated"
    made = od.documents()
    assert set(made) == set(NAMES) and all((DOCS / name).read_text() == made[name] and made[name].startswith(od.HEADER) for name in NAMES)


def test_every_preflight_check_is_a_line_of_the_checklist_with_its_own_limit_and_remedy():
    text = od.checklists()
    position = 0
    for name, (meaning, remedy) in preflight.guide().items():
        line = f"- [ ] **{name}**: {meaning}"
        assert line in text and f"  - NO-GO: {remedy}" in text
        assert text.index(line) > position  # in the preflight's own order
        position = text.index(line)
    limits = preflight.Limits()
    assert f"HDOP {limits.max_hdop:g} or less, {limits.min_satellites} satellites or more" in text and f"{limits.min_cell_v:g} V a cell or more, resting, on a {limits.cells}-cell pack" in text
    assert set(preflight.GUIDE) == set(preflight.CHECKS) and all(len(entry) == 2 and all(entry) for entry in preflight.GUIDE.values())
    # Every limit the preflight has is in a line: none can be tuned without the checklist changing (the spec audit: the 100 ms step limit was in no line).
    meanings = " ".join(entry[0] for entry in preflight.GUIDE.values())
    for name in (field.name for field in dataclasses.fields(preflight.Limits)):
        assert "{" + name + ":" in meanings or "{" + name + "}" in meanings, name
    assert f"none over {limits.max_timesync_step_ms:g} ms" in text and "With none named, nothing is compared and the line is GO" in text
    # The SOP names the same checks in the same order.
    assert ", ".join(f"`{name}`" for name in preflight.CHECKS) in od.sop(od.baseline())


def test_a_limit_changed_in_the_preflight_changes_the_checklist_and_fails_the_check(monkeypatch, tmp_path, capsys):
    stricter = od.checklists(preflight.Limits(min_cell_v=4.0, max_hdop=1.2))
    assert "4 V a cell or more" in stricter and "HDOP 1.2 or less" in stricter and stricter != od.checklists()
    monkeypatch.setitem(preflight.GUIDE, "battery", ("{min_cell_v:g} V a cell or more, measured under load.", "Change the pack."))
    assert od.main(["--check"]) == 1
    out = capsys.readouterr().out
    assert "checklists.md: not what tools/ops_documents.py writes now" in out and out.strip().splitlines()[-1] == "documents: DRIFTED"
    # Written somewhere else, the three files appear, and the check there then passes.
    assert od.main(["--out", str(tmp_path)]) == 0 and sorted(path.name for path in tmp_path.iterdir()) == sorted(NAMES)
    assert od.main(["--check", "--out", str(tmp_path)]) == 0


def test_what_the_aircraft_does_by_itself_is_read_from_the_baselines_values():
    params = od.baseline()
    rows = {row["what"]: row for row in od.failsafes(params)}
    assert list(rows) == ["Fence breached", "Battery low", "Battery critical", "Transmitter lost", "Ground station lost", "Position estimate lost (EKF)"]
    # The baseline as committed (PLT-08), and as its sim flights showed: home on a low battery, a silent receiver or a lost ground station; down on a critical battery.
    assert "RTL" in rows["Battery low"]["does"] and "lands where it is" == rows["Battery critical"]["does"] and "RTL" in rows["Transmitter lost"]["does"] and "RTL" in rows["Ground station lost"]["does"]
    assert rows["Position estimate lost (EKF)"]["does"] == "lands where it is" and "RTL" in rows["Fence breached"]["does"]
    assert "under 21 V (3.50 V a cell) for 10 s" == rows["Battery low"]["when"] and "no heartbeat from it for 5 s" == rows["Ground station lost"]["when"]
    assert rows["Battery critical"]["when"] == "under 19.8 V (3.30 V a cell) for 10 s"  # ArduPilot's timer is for both levels
    assert rows["Fence breached"]["when"] == "outside the polygon or above 120 m"
    text = od.emergency_procedures(params)
    for row in rows.values():
        assert f"| {row['what']} | {row['when']} | {od._does(row)} | `{row['params']}` |" in text
    # A baseline that changes an action changes the table AND the row that says what the people do about it (the spec audit).
    landing = od.emergency_procedures({**params, "FS_GCS_ENABLE": 5.0})
    assert "| Ground station lost | no heartbeat from it for 5 s | lands where it is; not while" in landing
    assert "| Ground station lost (the aircraft lands where it is; not while" in landing and "Ground station lost (the aircraft returns" not in landing
    braking = od.emergency_procedures({**params, "FENCE_ACTION": 4.0})
    assert "| Fence breached (the aircraft brakes, or lands) |" in braking and "(RTL), or lands) | Watches" not in braking
    for name in ("Fence breached", "Ground station lost", "Transmitter lost", "Battery low", "Position estimate lost (EKF)"):
        assert f"| {name.replace(' (EKF)', '')} (the aircraft {od._does(rows[name])})" in text, name


def test_when_a_failsafe_does_not_act_is_read_from_the_baselines_options():
    """The spec audit: the baseline's FS_OPTIONS says a lost ground station does nothing while the safety pilot is flying; the document said "coming home by itself"."""
    params = od.baseline()
    assert int(params["FS_OPTIONS"]) == 16
    assert od.failsafe_exceptions(params) == {"Ground station lost": "not while a person is flying it in LOITER, ALT_HOLD, STABILIZE or another pilot mode"}
    rows = {row["what"]: row for row in od.failsafes(params)}
    assert rows["Ground station lost"]["params"] == "FS_GCS_ENABLE, FS_OPTIONS" and rows["Transmitter lost"]["unless"] == ""
    assert "returns to the takeoff point (RTL); not while a person is flying it" in od.sop(params)
    assert od.failsafe_exceptions({"FS_OPTIONS": 0.0}) == {}
    guided = od.failsafe_exceptions({"FS_OPTIONS": 4.0 + 8.0})
    assert guided["Transmitter lost"] == "not in GUIDED (the mission stack's mode); a landing already under way goes on" and guided["Battery low"] == "a landing already under way goes on"
    assert od.failsafe_exceptions({"FS_OPTIONS": 3.0}) == {"Transmitter lost": "not in AUTO", "Ground station lost": "not in AUTO"}
    with pytest.raises(od.DocumentError, match="FS_OPTIONS is 64"):
        od.failsafe_exceptions({"FS_OPTIONS": 64.0})
    # The fence's trigger is its type's, and a baseline with the fence off writes no procedures.
    assert od.fence_trigger({**params, "FENCE_TYPE": 7.0, "FENCE_RADIUS": 300.0}) == "outside the polygon or over 300 m from home or above 120 m"
    assert od.fence_trigger({**params, "FENCE_TYPE": 4.0}) == "outside the polygon"
    with pytest.raises(od.DocumentError, match="FENCE_ENABLE is not 1"):
        od.fence_trigger({**params, "FENCE_ENABLE": 0.0})
    with pytest.raises(od.DocumentError, match="FENCE_TYPE is 0"):
        od.fence_trigger({**params, "FENCE_TYPE": 0.0})


def test_a_baseline_the_tool_has_no_words_for_is_refused_and_a_failsafe_may_never_release_the_payload():
    params = od.baseline()
    with pytest.raises(od.DocumentError, match="FS_THR_ENABLE is 2, which this tool has no words for"):
        od.failsafes({**params, "FS_THR_ENABLE": 2.0})
    missing = dict(params)
    del missing["BATT_FS_LOW_ACT"]
    with pytest.raises(od.DocumentError, match="the baseline has no BATT_FS_LOW_ACT"):
        od.failsafes(missing)
    with pytest.raises(od.DocumentError, match="a failsafe would release the gripper"):
        od.emergency_procedures({**params, "FS_OPTIONS": 48.0})
    assert "`FS_OPTIONS` bit 5 is clear" in od.emergency_procedures(params)


def test_what_the_documents_name_is_in_the_repo():
    """Every tool, script and document a procedure sends a person to, and the stack's own names."""
    made = od.documents()
    named = set()
    for text in made.values():
        named |= set(re.findall(r"(?<![A-Za-z0-9_/>-])((?:tools|platform|sim|docs|src)/[A-Za-z0-9_./-]+\.(?:py|sh|md|param|yaml))", text))
        named |= {f"docs/ops/{name}" for name in re.findall(r"`([a-z_]+\.md)`", text)}
    assert len(named) >= 14
    for path in sorted(named):
        assert (REPO / path).is_file(), path
    emergency = made["emergency_procedures.md"]
    assert ", ".join(actuation_lock.RELEASE_STATES) in emergency
    takeover = (REPO / "src/drone_mission_core/drone_mission_core/takeover.py").read_text()
    assert "rearm_autonomy" in takeover and "/mission_executor/rearm_autonomy std_srvs/srv/Trigger" in emergency
    # The failure policies are the executor's two, by their own names.
    api = (REPO / "src/drone_mission_core/drone_mission_core/mission_api.py").read_text()
    for policy in re.findall(r"`((?:abort_and|continue)_[a-z_]+)`", emergency):
        assert f'"{policy}"' in api, policy
    assert emergency.count("`abort_and_rtl`") == 2 and emergency.count("`continue_to_next`") == 2
    # Every option a command line of the documents uses is one its tool takes.
    for tool, options in (("tools/field_setup.py", ["--for-flight"]), ("sim/fence_upload.py", ["--site", "--url"]), ("tools/dataflash_pull.py", ["--fc-url", "--out"]),
                          ("tools/bundle_attempt.py", ["--slot", "--mission", "--from-run"]), ("tools/freeze_check.py", ["--worktree"]), ("tools/slot_clock.py", ["--slot", "--watch"]),
                          ("tools/go.py", ["--profile", "--sequence", "--params", "--camera", "--param-dump", "--clock-source", "--record", "--override"])):  # fmt: skip
        source = (REPO / tool).read_text()
        for option in options:
            assert f'"{option}"' in source, (tool, option)
            assert any(option in text and tool.split("/")[-1] in text for text in made.values()), (tool, option)
    # What the documents say a command prints is what it prints.
    for tool, words in (("tools/freeze_check.py", "FREEZE HOLDS"), ("tools/bundle_attempt.py", '"COMPLETE"'), ("tools/post_flight.py", "NO HUMAN INPUT FOUND"), ("tools/field_setup.py", "summary.txt"),
                        ("platform/jetson/lib.sh", "CHECKLIST: $ok of $# ok")):  # fmt: skip
        assert words in (REPO / tool).read_text(), (tool, words)
    assert "`CHECKLIST: 3 of 3 ok`" in made["checklists.md"] and 'STEPS=(clone at_tag build)' in (REPO / "platform/jetson/fallback.sh").read_text()
    # The statements about the stack are the code's own (the spec audit found four that were not).
    recovery = " ".join((REPO / "src/drone_mission_demo/drone_mission_demo/missions/package_recovery_mission.py").read_text().split())
    assert "RTL while the gripper has never closed and the vehicle has a position to fly home on, LAND from the first close for the rest of the mission and whenever there is no pose, no GPS fix or GPS is not trusted" in recovery
    assert "RTL while the gripper has never closed and there is a position to fly home on; a landing from the first close on, and whenever there is no pose, no GPS fix or GPS is not trusted" in emergency
    executor = (REPO / "src/drone_mission_core/drone_mission_core/mission_executor.py").read_text()
    assert 'get_parameter("abort_rtl_mode")' in executor and "`abort_rtl_mode`" in emergency
    assert "with `continue_to_next` the executor flies the sequence's next mission" in emergency and "failure_policy: continue_to_next" in (REPO / "src/drone_mission_demo/config/sequences/package_delivery_live.yaml").read_text()
    limits_doc = (REPO / "src/drone_mission_core/drone_mission_core/safety_limits.py").read_text()
    assert "The June 2026 missions" in limits_doc and "do not import this module" in limits_doc and "the June 2026 missions (every flag off, the profile `comp_2026`) have no such limit" in emergency
    # The margin the document quotes is the one every 2027 mission module uses (two of them keep their own constant).
    for module in ("gps_route.py", "recovery_flight.py", "safety_limits.py"):
        source = (REPO / "src/drone_mission_core/drone_mission_core" / module).read_text()
        assert float(re.search(r"^GEOFENCE_MARGIN_M = ([0-9.]+)", source, re.M).group(1)) == od.safety_limits.GEOFENCE_MARGIN_M, module
    assert "each behind its flag" not in emergency and "ends by its failure policy (home)" not in emergency
    assert "adds `--engine <file>`: without it the `engine` check compares nothing" in made["sop.md"] and "no engine was named" in (REPO / "src/drone_utils/drone_utils/preflight.py").read_text()
    # The flags the procedures speak of are declared in the shipped sequences.
    sequence = (REPO / "src/drone_mission_demo/config/sequences/package_delivery_live.yaml").read_text()
    assert "takeover_latch: false" in sequence and "actuation_lock: false" in sequence
    for profile in re.findall(r"profile `([a-z0-9_]+)`", made["sop.md"]):
        assert (REPO / "profiles" / f"{profile}.yaml").is_file(), profile


def test_each_document_has_its_parts():
    made = od.documents()
    for heading in ("## Who", "## At the field, once", "## Each flight", "## Between flights", "## What the aircraft will do by itself"):
        assert heading in made["sop.md"]
    for heading in ("## What the aircraft does by itself", "## What the people do", "## Taking over, and giving back"):
        assert heading in made["emergency_procedures.md"]
    for heading in ("## Start of the day", "## Before each flight: by hand", "## Before each flight: the preflight", "## After each flight"):
        assert heading in made["checklists.md"]
    assert made["checklists.md"].count("- [ ]") >= 7 + 5 + len(preflight.CHECKS) + 6
    assert made["emergency_procedures.md"].count("\n| ") >= 6 + 11
