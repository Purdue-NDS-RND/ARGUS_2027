"""OPS-05: the slot clock (tools/slot_clock.py) and its line in the GO launcher. No ROS."""

import json
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import go  # noqa: E402
import slot_clock as sc  # noqa: E402

START = datetime(2027, 6, 4, 14, 5, 0, tzinfo=timezone.utc)
BUDGET = sc.check_budget({"slot_min": 20, "phases": {"setup": 4, "preflight": 2, "flight": 10, "pack_up": 4}, "list_due_after_slot_min": 30})


def at(minutes, seconds=0):
    return START + timedelta(minutes=minutes, seconds=seconds)


def test_the_committed_budget_is_usable_and_the_lists_deadline_is_the_rules():
    budget = sc.load_budget()
    assert [name for name, _length in budget["phases"]] == ["setup", "preflight", "flight", "pack_up"] and budget["list_due_s"] == 30 * 60
    assert sum(length for _name, length in budget["phases"]) <= budget["slot_s"]


@pytest.mark.parametrize(
    "raw, word",
    [
        ({"slot_min": 20, "phases": {"setup": 4, "preflight": 2, "flight": 12, "pack_up": 4}, "list_due_after_slot_min": 30}, "the phases add up to 22 min, and the slot is 20 min"),
        ({"slot_min": 20, "phases": {"preflight": 2, "setup": 4, "flight": 10, "pack_up": 4}, "list_due_after_slot_min": 30}, "in that order"),
        ({"slot_min": 20, "phases": {"setup": 4, "preflight": 2, "flight": 0, "pack_up": 4}, "list_due_after_slot_min": 30}, "phases.flight must be a number of minutes above zero"),
        ({"slot_min": 20, "phases": {"setup": 4, "preflight": 2, "flight": 10, "pack_up": 4}}, "expected slot_min, phases and list_due_after_slot_min"),
        ({"slot_min": True, "phases": {"setup": 4, "preflight": 2, "flight": 10, "pack_up": 4}, "list_due_after_slot_min": 30}, "slot_min must be a number"),
    ],
)
def test_a_budget_that_does_not_fit_its_slot_is_refused(raw, word):
    with pytest.raises(sc.ClockError, match=word):
        sc.check_budget(raw)


def test_the_timeline_is_the_phases_in_order_and_then_the_lists_deadline():
    rows = sc.timeline(BUDGET, START)
    assert [(name, (begins - START).total_seconds() / 60, (ends - START).total_seconds() / 60) for name, begins, ends in rows] == [
        ("setup", 0, 4), ("preflight", 4, 6), ("flight", 6, 16), ("pack_up", 16, 20), ("list due", 20, 50)]  # fmt: skip
    spare = sc.check_budget({"slot_min": 25, "phases": {"setup": 4, "preflight": 2, "flight": 10, "pack_up": 4}, "list_due_after_slot_min": 30})
    assert [name for name, _b, _e in sc.timeline(spare, START)] == ["setup", "preflight", "flight", "pack_up", "spare", "list due"]


@pytest.mark.parametrize(
    "now, line",
    [
        (at(-2), "SLOT 2: starts in 02:00"),
        (at(0, 1), "SLOT 2: 00:01 in | planned: setup, 03:59 left | slot ends in 19:59 | list due in 49:59"),
        (at(5), "SLOT 2: 05:00 in | planned: preflight, 01:00 left | slot ends in 15:00 | list due in 45:00"),
        (at(7, 12), "SLOT 2: 07:12 in | planned: flight, 08:48 left | slot ends in 12:48 | list due in 42:48"),
        (at(17), "SLOT 2: 17:00 in | planned: pack-up, 03:00 left | slot ends in 03:00 | list due in 33:00"),
        (at(26), "SLOT 2: 26:00 in | slot over by 06:00 | list due in 24:00"),
        (at(51, 30), "SLOT 2: 51:30 in | slot over by 31:30 | LIST DEADLINE PASSED 01:30 ago"),
        (at(81), "SLOT 2: 1:21:00 in | slot over by 1:01:00 | LIST DEADLINE PASSED 31:00 ago"),
    ],
)
def test_the_line_says_where_the_team_is_and_what_is_left(now, line):
    assert sc.line("2", BUDGET, START, now) == line


def test_the_launcher_is_warned_when_a_flight_no_longer_fits_the_slot():
    assert sc.flight_warning(BUDGET, START, at(6)) is None  # exactly the flight and its pack-up left
    assert sc.flight_warning(BUDGET, START, at(9)) == "SLOT CLOCK: 11:00 of the slot is left, and a flight with its pack-up is budgeted 14:00"
    assert sc.flight_warning(BUDGET, START, at(22)) == "SLOT CLOCK: the slot ended 02:00 ago"


def test_a_start_time_is_read_as_the_nearest_such_time_on_this_computers_clock():
    now = datetime(2027, 6, 4, 14, 10, 0).astimezone()
    assert sc.parse_start("14:05", now) == now.replace(minute=5) and sc.parse_start("14:05:30", now) == now.replace(minute=5, second=30)
    late = datetime(2027, 6, 5, 0, 5, 0).astimezone()
    assert sc.parse_start("23:58", late) == datetime(2027, 6, 4, 23, 58, 0).astimezone()  # started before midnight, read after it
    assert sc.parse_start("2027-06-04T14:05:00+00:00", now) == START
    for bad in ("25:00", "noon", "14", "14:61"):
        with pytest.raises(sc.ClockError, match="is not a time"):
            sc.parse_start(bad, now)


def test_start_show_and_stop_keep_the_slot_in_a_small_file(tmp_path, capsys):
    import yaml

    state = tmp_path / "attempts" / "slot_clock.json"
    (tmp_path / "budget.yaml").write_text(yaml.safe_dump(OWN_BUDGET, sort_keys=False))
    own = ["--budget", str(tmp_path / "budget.yaml")]  # the committed budget is the lead's to change: nothing here counts against it
    assert sc.main(["show", "--state", str(state), *own]) == 1 and "no slot is running" in capsys.readouterr().out
    assert sc.main(["start", "--slot", "2", "--state", str(state), *own], now=lambda: START) == 0
    out = capsys.readouterr().out
    assert out.splitlines()[0] == "SLOT 2: 00:00 in | planned: setup, 04:00 left | slot ends in 20:00 | list due in 50:00" and "<- now" in out
    assert f"on the aircraft: python3 tools/slot_clock.py start --slot 2 --at {START.astimezone().isoformat(timespec='seconds')}" in out
    assert json.loads(state.read_text()) == {"slot": "2", "start": START.isoformat()}
    assert sc.main(["show", "--state", str(state), *own], now=lambda: at(7, 12)) == 0
    out = capsys.readouterr().out
    assert out.splitlines()[0].startswith("SLOT 2: 07:12 in | planned: flight") and [row for row in out.splitlines() if "<- now" in row][0].split()[1] == "flight"
    # The watch: the line again each second.
    clock = iter([at(7, 12), at(7, 13), at(7, 14)])
    assert sc.main(["show", "--watch", "--state", str(state), *own], now=lambda: next(clock), sleep=lambda _s: None, ticks=3) == 0
    assert [row.split(" | ")[0] for row in capsys.readouterr().out.splitlines()] == ["SLOT 2: 07:12 in", "SLOT 2: 07:13 in", "SLOT 2: 07:14 in"]
    capsys.readouterr()
    # Stop keeps the last slot's file under another name and deletes nothing.
    assert sc.main(["stop", "--state", str(state), *own]) == 0 and not state.exists() and state.with_name("slot_clock.json.last").is_file()
    assert sc.main(["start", "--state", str(state), *own]) == 2 and "start needs --slot" in capsys.readouterr().err


def test_the_start_goes_to_the_aircraft_as_an_instant_whatever_zone_either_computer_keeps(tmp_path, capsys, monkeypatch):
    """The spec audit: the hand-off was a wall-clock time with no zone, and a laptop (or a container) on another zone gave a slot hours off."""
    import shlex
    import time

    laptop, aircraft = tmp_path / "laptop.json", tmp_path / "aircraft.json"
    monkeypatch.setenv("TZ", "America/Indiana/Indianapolis")
    time.tzset()
    try:
        assert sc.main(["start", "--slot", "2", "--at", "10:05", "--state", str(laptop)], now=lambda: at(1)) == 0  # 14:05 UTC is 10:05 there in June
        printed = [row for row in capsys.readouterr().out.splitlines() if row.startswith("on the aircraft: ")][0]
        command = shlex.split(printed.split(": ", 1)[1])
        assert command[:5] == ["python3", "tools/slot_clock.py", "start", "--slot", "2"] and command[-1] == "2027-06-04T10:05:00-04:00"
        # The aircraft keeps UTC: the printed command, as printed, gives it the same instant.
        monkeypatch.setenv("TZ", "UTC")
        time.tzset()
        assert sc.main(command[2:] + ["--state", str(aircraft)], now=lambda: at(1)) == 0
        assert sc.read_state(aircraft)["start"] == sc.read_state(laptop)["start"] == START
        assert sc.for_go(command[-1], "2", at(5), state=tmp_path / "none.json", budget_file=sc.BUDGET)[0].startswith("SLOT 2: 05:00 in")  # and the launcher's --slot-start takes it too
    finally:
        monkeypatch.undo()
        time.tzset()
    capsys.readouterr()
    # A start written by hand with no zone is not an instant: no slot is running, and nothing raises in the launcher or in show.
    naive = tmp_path / "naive.json"
    naive.write_text(json.dumps({"slot": "2", "start": "2027-06-04T14:05:00"}))
    assert sc.read_state(naive) is None and sc.for_go(None, None, at(5), state=naive) is None
    assert sc.main(["show", "--state", str(naive)], now=lambda: at(5)) == 1 and "no slot" in capsys.readouterr().out.lower()


OWN_BUDGET = {"slot_min": 20, "phases": {"setup": 4, "preflight": 2, "flight": 10, "pack_up": 4}, "list_due_after_slot_min": 30}  # the launcher's tests count against this, not the committed file: that one is the lead's to change


FLIGHT = ["--profile", "comp_2026", "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml", "--detector", "target_cv"]  # as tools/test/test_go.py plans one


@pytest.fixture(autouse=True)
def no_installed_workspace(monkeypatch):
    """Planned against the source tree, whatever workspace the machine running the tests has sourced."""
    monkeypatch.setattr(go, "installed_share", lambda: None)


def fly(arguments, state, preflight=0):
    import yaml

    ran = []
    budget = Path(state).parent / "own_budget.yaml"
    budget.write_text(yaml.safe_dump(OWN_BUDGET, sort_keys=False))
    code = go.main(FLIGHT + arguments, run=lambda command: ran.append("preflight") or preflight, start=lambda command: None, sleep=lambda _s: None,
                   launch=lambda command: ran.append("launch") or 0, stop=lambda process: None, slot_state=state, slot_budget=budget)  # fmt: skip
    return code, ran


def test_the_launcher_shows_the_slots_line_before_the_preflight_and_at_go_and_records_it(tmp_path, capsys):
    state = tmp_path / "slot_clock.json"
    state.write_text(json.dumps({"slot": "2", "start": (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()}) + "\n")
    code, ran = fly(["--record", str(tmp_path / "flight")], state)
    out = capsys.readouterr().out.splitlines()
    assert code == 0 and ran == ["preflight", "launch"]
    lines = [row for row in out if row.startswith("SLOT 2: 05:0")]
    assert len(lines) == 2 and "planned: preflight" in lines[0] and out.index(lines[0]) < out.index("GO") < len(out) - 1 - out[::-1].index(lines[1])
    recorded = json.loads((tmp_path / "flight" / "go.json").read_text())["slot_clock"]
    assert recorded["slot"] == "2" and recorded["planned_phase"] == "preflight" and 299 <= recorded["at_go_s"] <= 303 and 897 <= recorded["slot_left_s"] <= 901


def test_the_launcher_warns_and_still_flies_when_the_slot_is_nearly_gone_and_says_nothing_with_no_slot(tmp_path, capsys):
    state = tmp_path / "slot_clock.json"
    state.write_text(json.dumps({"slot": "3", "start": (datetime.now(timezone.utc) - timedelta(minutes=12)).isoformat()}) + "\n")
    code, ran = fly([], state)
    out = capsys.readouterr().out
    assert code == 0 and ran == ["preflight", "launch"] and out.count("SLOT CLOCK: 0") == 2 and "of the slot is left, and a flight with its pack-up is budgeted 14:00" in out
    # A NO-GO: the line was shown before the preflight, and nothing was launched.
    code, ran = fly([], state, preflight=1)
    assert code == 1 and ran == ["preflight"] and capsys.readouterr().out.count("SLOT 3:") == 1
    # No slot running: the launcher's output is what it was.
    code, ran = fly([], tmp_path / "none.json")
    assert code == 0 and "SLOT" not in capsys.readouterr().out
    # --slot-start gives the slot without the file; a start that cannot be read does not stop the flight.
    code, ran = fly(["--slot", "1", "--slot-start", f"{datetime.now().astimezone() - timedelta(minutes=1):%H:%M:%S}"], tmp_path / "none.json")
    assert code == 0 and "SLOT 1: 01:0" in capsys.readouterr().out
    code, ran = fly(["--slot-start", "noon"], tmp_path / "none.json")
    assert code == 0 and ran == ["preflight", "launch"] and "slot clock: 'noon' is not a time" in capsys.readouterr().out


def test_the_ground_stations_document_and_wrapper_show_the_clock():
    """The ground station is a terminal on the laptop: its document says how the clock is started and watched there, through the wrapper."""
    import shlex
    import subprocess

    repo = Path(__file__).resolve().parents[2]
    text = (repo / "docs" / "ops" / "ground_station.md").read_text()
    assert "tools/ground_station.sh slot_clock.py start --slot 2" in text and "tools/ground_station.sh slot_clock.py show --watch" in text
    printed = subprocess.run([str(repo / "tools" / "ground_station.sh"), "--print", "slot_clock.py", "show", "--watch"], capture_output=True, text=True, stdin=subprocess.DEVNULL)
    assert printed.returncode == 0 and shlex.split(printed.stdout)[-4:] == ["python3", "tools/slot_clock.py", "show", "--watch"]
