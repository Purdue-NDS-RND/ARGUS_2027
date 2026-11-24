"""PLT-12: one folder per attempt, with a manifest (tools/bundle_attempt.py). No ROS."""

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest
import yaml

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import bundle_attempt as ba  # noqa: E402
from bundle_attempt import BundleError  # noqa: E402

GIT = {"hash": "c0ffee", "branch": "platform/build-2027", "describe": "comp-2026-99-gc0ffee", "dirty": False}
NOW = datetime(2027, 5, 21, 14, 30, tzinfo=timezone.utc)


def make_bag(folder: Path, topics=None) -> Path:
    topics = topics or {"/mavros/local_position/pose": "geometry_msgs/msg/PoseStamped", "/mavros/state": "mavros_msgs/msg/State"}
    folder.mkdir(parents=True)
    (folder / "bag_0.mcap").write_bytes(b"\x89MCAP0\r\n" + b"x" * 400)
    listed = [{"topic_metadata": {"name": name, "type": kind, "serialization_format": "cdr"}, "message_count": 10} for name, kind in topics.items()]
    (folder / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {"storage_identifier": "mcap", "topics_with_message_count": listed}}))
    return folder


def make_frames(folder: Path, count=3) -> Path:
    folder.mkdir(parents=True)
    with open(folder / "index.jsonl", "w") as index:
        for number in range(count):
            (folder / f"frame_{number:06d}.jpg").write_bytes(b"\xff\xd8" + bytes([number]) * 50)
            index.write(json.dumps({"seq": number, "file": f"frame_{number:06d}.jpg", "bytes": 52}) + "\n")
    return folder


@pytest.fixture()
def flight(tmp_path):
    """Everything a delivery attempt leaves behind, in the places a flight leaves it."""
    src = tmp_path / "flight"
    src.mkdir()
    (src / "00000042.BIN").write_bytes(b"\xa3\x95" + b"d" * 2000)
    (src / "sequence.yaml").write_text("mission_sequence:\n  features: {}\n  missions: []\n")
    (src / "params.yaml").write_text("mission_executor:\n  ros__parameters: {loop_rate_hz: 20.0}\n")
    (src / "delivery_2027.yaml").write_text("description: x\nenable: [use_frame_time_pose]\nnodes: {target_cv: [stamp_from_image]}\n")
    (src / "baseline.param").write_text("FENCE_ENABLE,1\n")
    (src / "preflight.jsonl").write_text(json.dumps({"verdict": "NO-GO", "failed": ["gps"]}) + "\n" + json.dumps({"verdict": "GO", "failed": []}) + "\n")
    (src / "judges_list.csv").write_text("id,lat,lon\n3,40.1,-86.9\n")
    return {
        "bag": make_bag(src / "bag"), "dataflash": src / "00000042.BIN", "frames": [make_frames(src / "down")], "sequence": src / "sequence.yaml", "params": src / "params.yaml",
        "profile": src / "delivery_2027.yaml", "baseline": src / "baseline.param", "preflight_log": src / "preflight.jsonl", "judges_list": src / "judges_list.csv",
    }  # fmt: skip


def bundled(tmp_path, flight, mission="delivery", **changes):
    given = {**flight, **changes}
    return ba.bundle(ba.attempt_name("2027-05-21", "2", mission), tmp_path / "attempts", mission, git=GIT, now=NOW, **given)


def test_an_attempt_is_one_folder_named_by_date_slot_and_mission_with_every_part_and_a_manifest(tmp_path, flight):
    manifest = bundled(tmp_path, flight)
    folder = tmp_path / "attempts" / "2027-05-21_2_delivery"
    assert manifest == json.loads((folder / "manifest.json").read_text())
    assert manifest["attempt"] == "2027-05-21_2_delivery" and (manifest["date"], manifest["slot"], manifest["mission"]) == ("2027-05-21", "2", "delivery")
    assert manifest["created_utc"] == "2027-05-21T14:30:00Z" and manifest["git"] == GIT
    assert manifest["complete"] is True and manifest["owed"] == []
    assert manifest["missing"] == {"session": "not given"}  # said, and not owed: the session logger is optional
    for path in ("bag/bag_0.mcap", "bag/metadata.yaml", "dataflash/00000042.BIN", "frames/down/index.jsonl", "frames/down/frame_000002.jpg", "config/sequence.yaml",
                 "config/params.yaml", "config/profile.yaml", "config/fc_baseline.param", "preflight.jsonl", "judges_list.csv"):  # fmt: skip
        assert (folder / path).is_file(), path
    # The flag profile as flown, and what the preflight last said.
    assert manifest["profile"]["name"] == "delivery_2027" and manifest["profile"]["flags"] == ["use_frame_time_pose"] and manifest["profile"]["node_flags"] == {"target_cv": ["stamp_from_image"]}
    assert manifest["parts"]["preflight"]["records"] == 2 and manifest["parts"]["preflight"]["verdict"] == "GO"
    assert manifest["parts"]["bag"]["topics"]["/mavros/state"] == "mavros_msgs/msg/State"
    assert manifest["parts"]["frames"]["logs"][0]["frames"] == 3
    # Every file of every part is in the manifest with its size and sha256, and the source is still where it was (copied).
    mcap = next(item for item in manifest["parts"]["bag"]["files"] if item["file"] == "bag/bag_0.mcap")
    assert mcap["bytes"] == 408 and mcap["sha256"] == ba.sha256_of(flight["bag"] / "bag_0.mcap")
    assert flight["dataflash"].is_file() and ba.verify(folder) == []


def test_a_bag_with_images_in_it_is_refused_and_nothing_is_written(tmp_path, flight):
    with_images = make_bag(tmp_path / "bad_bag", {"/mavros/state": "mavros_msgs/msg/State", "/camera/image": "sensor_msgs/msg/Image", "/x/compressed": "sensor_msgs/msg/CompressedImage"})
    with pytest.raises(BundleError, match=r"holds images \(/camera/image, /x/compressed\)"):
        bundled(tmp_path, flight, bag=with_images)
    assert not (tmp_path / "attempts").exists()


def test_a_bag_that_was_not_closed_goes_in_as_it_is_and_is_owed(tmp_path, flight):
    """The spec audit: a recorder that was killed must not cost the attempt its dataflash log and config too."""
    (flight["bag"] / "metadata.yaml").unlink()
    manifest = bundled(tmp_path, flight)
    folder = tmp_path / "attempts" / "2027-05-21_2_delivery"
    assert (folder / "bag" / "bag_0.mcap").is_file() and (folder / "dataflash" / "00000042.BIN").is_file()
    assert manifest["complete"] is False and manifest["owed"] == ["bag"] and "no metadata.yaml" in manifest["missing"]["bag"] and "reindex" in manifest["missing"]["bag"]
    assert manifest["parts"]["bag"]["closed"] is False and manifest["parts"]["bag"]["storage"] is None
    assert ba.verify(folder) == []


def test_the_manifest_says_what_the_bag_is_stored_as_and_warns_when_it_is_not_mcap(tmp_path, flight):
    manifest = bundled(tmp_path / "a", flight)
    assert manifest["parts"]["bag"]["storage"] == "mcap" and manifest["parts"]["bag"]["closed"] is True and manifest["warnings"] == []
    metadata = flight["bag"] / "metadata.yaml"
    metadata.write_text(metadata.read_text().replace("storage_identifier: mcap", "storage_identifier: sqlite3"))
    manifest = bundled(tmp_path / "b", flight)
    assert manifest["parts"]["bag"]["storage"] == "sqlite3" and manifest["warnings"] == ["the bag is sqlite3, not MCAP: the recorder found no MCAP plugin where it ran"]
    assert manifest["complete"] is True  # said, not owed


def test_config_that_is_not_what_the_flights_own_records_name_is_a_warning(tmp_path, flight):
    """The spec audit: the operator names the profile again after the flight; the flight's own word is in its folder."""
    flight["preflight_log"].write_text(json.dumps({"verdict": "GO", "failed": [], "profile": "/repo/profiles/comp_2026.yaml", "sequence": "/x/package_delivery_live.yaml",
                                                   "baseline_sha256": "0" * 64}) + "\n")  # fmt: skip
    manifest = bundled(tmp_path / "a", flight)
    assert manifest["warnings"] == [
        "the profile bundled is delivery_2027.yaml; the flight's own record names comp_2026.yaml",
        "the sequence bundled is sequence.yaml; the flight's own record names package_delivery_live.yaml",
        "the flight-controller baseline bundled is not the one the preflight held the aircraft to",
    ]
    # The GO command's record (go.json) is believed before the preflight's, and agreeing with both is silence.
    flight["preflight_log"].write_text(json.dumps({"verdict": "GO", "failed": [], "profile": "/repo/profiles/delivery_2027.yaml", "sequence": "/x/sequence.yaml",
                                                   "baseline_sha256": ba.sha256_of(flight["baseline"])}) + "\n")  # fmt: skip
    assert bundled(tmp_path / "b", flight)["warnings"] == []
    manifest = bundled(tmp_path / "c", flight, flown={"profile": "/repo/profiles/other.yaml", "sequence": "/x/sequence.yaml", "mission_flags": "a,b"})
    assert manifest["warnings"] == ["the profile bundled is delivery_2027.yaml; the flight's own record names other.yaml"] and manifest["flown"]["mission_flags"] == "a,b"


def test_a_missing_part_is_said_and_the_bundle_is_still_made(tmp_path, flight):
    manifest = bundled(tmp_path, flight, dataflash=None, frames=[], judges_list=tmp_path / "nowhere.csv", params=None)
    assert manifest["complete"] is False and manifest["owed"] == ["dataflash", "config"]  # a delivery owes no frames and no list
    assert manifest["missing"]["dataflash"] == "not given" and manifest["missing"]["frames"] == "not given" and "nowhere.csv is not there" in manifest["missing"]["judges_list"]
    assert manifest["missing"]["config"] == "without params.yaml"
    assert "judges_list" not in manifest["owed"]  # a delivery writes no list
    assert set(manifest["parts"]) == {"bag", "config", "preflight"} and (tmp_path / "attempts" / "2027-05-21_2_delivery" / "bag" / "bag_0.mcap").is_file()


def test_what_a_mission_owes_depends_on_the_mission(tmp_path, flight):
    assert bundled(tmp_path / "a", flight, mission="localisation", judges_list=None)["owed"] == ["judges_list"]
    assert bundled(tmp_path / "c", flight, mission="localisation", frames=[])["owed"] == ["frames"]
    assert bundled(tmp_path / "b", flight, mission="time_trial", frames=[], judges_list=None)["complete"] is True  # no camera, no list
    assert set(ba.EXPECTED) == {"localisation", "drop", "delivery", "recovery", "waypoint_navigation", "time_trial", "geofence"}
    assert all(set(parts) <= set(ba.PARTS) and {"bag", "dataflash", "config", "preflight"} <= set(parts) for parts in ba.EXPECTED.values())


def test_an_attempts_folder_is_never_overwritten(tmp_path, flight):
    bundled(tmp_path, flight)
    with pytest.raises(BundleError, match="never overwritten"):
        bundled(tmp_path, flight)


def test_move_takes_the_large_parts_and_copies_the_config(tmp_path, flight):
    bundled(tmp_path, flight, move=True)
    assert not flight["bag"].exists() and not flight["dataflash"].exists() and not flight["frames"][0].exists()
    assert flight["sequence"].is_file() and flight["profile"].is_file()  # the repo's files stay


def test_verify_says_what_is_no_longer_what_was_bundled(tmp_path, flight):
    bundled(tmp_path, flight)
    folder = tmp_path / "attempts" / "2027-05-21_2_delivery"
    (folder / "dataflash" / "00000042.BIN").write_bytes(b"other")
    (folder / "judges_list.csv").unlink()
    with open(folder / "frames" / "down" / "index.jsonl", "a") as index:
        index.write("{}\n")
    problems = ba.verify(folder)
    assert len(problems) == 3 and any("00000042.BIN is not the file" in p for p in problems) and any("judges_list.csv is gone" in p for p in problems) and any("index.jsonl" in p for p in problems)
    assert ba.verify(tmp_path / "nowhere")[0].startswith("no readable manifest.json")


@pytest.mark.parametrize("date, slot, mission, word", [("21-05-2027", "2", "delivery", "YYYY-MM-DD"), ("2027-05-21", "Slot 2", "delivery", "slot"), ("2027-05-21", "2", "racing", "mission is one of")])
def test_a_name_that_is_not_date_slot_mission_is_refused(date, slot, mission, word):
    with pytest.raises(BundleError, match=word):
        ba.attempt_name(date, slot, mission)


def test_two_frame_logs_with_one_name_are_refused_and_a_folder_with_no_index_is_not_a_frame_log(tmp_path, flight):
    with pytest.raises(BundleError, match="same folder name"):
        bundled(tmp_path, flight, frames=[flight["frames"][0], make_frames(tmp_path / "other" / "down")])
    not_a_log = tmp_path / "pictures"
    not_a_log.mkdir()
    manifest = bundled(tmp_path, flight, mission="localisation", frames=[not_a_log])
    assert "no index.jsonl" in manifest["missing"]["frames"] and manifest["owed"] == ["frames"]


def test_the_command_bundles_prints_what_is_in_and_what_is_owed_and_says_so_with_its_exit_code(tmp_path, flight, capsys):
    arguments = ["--slot", "2", "--mission", "delivery", "--date", "2027-05-21", "--out", str(tmp_path / "attempts"), "--bag", str(flight["bag"]), "--dataflash", str(flight["dataflash"]),
                 "--frames", str(flight["frames"][0]), "--profile", str(flight["profile"]), "--sequence", str(flight["sequence"]), "--params", str(flight["params"]),
                 "--baseline", str(flight["baseline"]), "--preflight-log", str(flight["preflight_log"]), "--note", "touched down 0.3 m short"]  # fmt: skip
    assert ba.main(arguments) == 0
    out = capsys.readouterr().out
    assert "attempt 2027-05-21_2_delivery" in out and out.strip().splitlines()[-1] == "COMPLETE" and "[   in] bag" in out and "[   -] judges_list" in out
    manifest = json.loads((tmp_path / "attempts" / "2027-05-21_2_delivery" / "manifest.json").read_text())
    assert manifest["notes"] == ["touched down 0.3 m short"] and manifest["git"]["hash"]  # the real repo's commit
    assert ba.main(["--verify", str(tmp_path / "attempts" / "2027-05-21_2_delivery")]) == 0
    # The same again is refused (exit 2); one with no dataflash log is made and says INCOMPLETE (exit 1).
    assert ba.main(arguments) == 2 and "never overwritten" in capsys.readouterr().err
    short = [item for item in arguments if item not in ("--dataflash", str(flight["dataflash"]))]
    short[short.index("2")] = "3"
    assert ba.main(short) == 1
    out = capsys.readouterr().out
    assert out.strip().splitlines()[-1] == "INCOMPLETE: dataflash" and "[OWED] dataflash" in out


def test_a_sim_runs_folder_gives_what_an_attempt_holds(tmp_path, flight):
    run = tmp_path / "run"
    run.mkdir()
    make_bag(run / "bag")
    (run / "sequence.yaml").write_text("mission_sequence: {features: {}, missions: []}\n")
    (run / "params.yaml").write_text("{}\n")
    (run / "preflight.jsonl").write_text(json.dumps({"verdict": "GO", "failed": []}) + "\n")
    (run / "fc_baseline.param").write_text("FENCE_ENABLE,1\n")
    (run / "dataflash").mkdir()
    (run / "dataflash" / "00000007.BIN").write_bytes(b"log")
    make_frames(run / "frames" / "down")
    (run / "scenario.json").write_text(json.dumps({"preflight": {"profile": str(flight["profile"])}, "list_file": None}))
    found = ba.from_run(run)
    assert found["bag"] == run / "bag" and found["dataflash"] == run / "dataflash" / "00000007.BIN" and found["frames"] == [run / "frames" / "down"]
    assert found["profile"] == flight["profile"] and found["baseline"] == run / "fc_baseline.param" and found["judges_list"] is None
    # A flight's own log folder (go --record) has no scenario.json: the GO command's go.json names what was flown.
    (run / "scenario.json").unlink()
    plain = ba.from_run(run)
    assert plain["bag"] == run / "bag" and plain["dataflash"] == run / "dataflash" / "00000007.BIN" and plain["profile"] is None and plain["sequence"] == run / "sequence.yaml"
    (run / "sequence.yaml").unlink()
    (run / "go.json").write_text(json.dumps({"profile": str(flight["profile"]), "sequence": str(flight["sequence"]), "params": str(flight["params"]), "mission_flags": ""}))
    from_go = ba.from_run(run)
    assert from_go["profile"] == flight["profile"] and from_go["sequence"] == flight["sequence"] and from_go["params"] == run / "params.yaml"  # the folder's own file first
    assert from_go["flown"]["profile"] == str(flight["profile"])
    (run / "go.json").unlink()
    (run / "sequence.yaml").write_text("mission_sequence: {features: {}, missions: []}\n")
    (run / "scenario.json").write_text(json.dumps({"preflight": {"profile": str(flight["profile"])}, "list_file": None}))
    assert ba.main(["--from-run", str(run), "--slot", "sim", "--mission", "delivery", "--date", "2026-11-23", "--out", str(tmp_path / "attempts")]) == 0
    assert (tmp_path / "attempts" / "2026-11-23_sim_delivery" / "dataflash" / "00000007.BIN").read_bytes() == b"log"


def test_the_session_loggers_folder_goes_in_when_it_is_given(tmp_path, flight):
    session = tmp_path / "session_20270521"
    (session / "images").mkdir(parents=True)
    (session / "commands.csv").write_text("t,vx\n0,1\n")
    (session / "images" / "a.jpg").write_bytes(b"jpg")
    manifest = bundled(tmp_path, flight, session=session)
    assert manifest["parts"]["session"] == {"count": 2, "bytes": 12} and (tmp_path / "attempts" / "2027-05-21_2_delivery" / "session" / "images" / "a.jpg").is_file()
    assert "session" not in bundled(tmp_path / "other", flight)["owed"]  # never owed


def test_the_commit_is_read_from_the_checkouts_files_when_git_cannot_be_asked(tmp_path):
    """In the sim's container git refuses a checkout owned by another user; an aircraft may have no git at all."""
    repo = tmp_path / "repo"
    (repo / ".git" / "refs" / "heads" / "platform").mkdir(parents=True)
    (repo / ".git" / "HEAD").write_text("ref: refs/heads/platform/build-2027\n")
    (repo / ".git" / "refs" / "heads" / "platform" / "build-2027").write_text("f6ed068aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n")
    assert ba.head_from_files(repo) == ("f6ed068aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", "platform/build-2027")
    (repo / ".git" / "refs" / "heads" / "platform" / "build-2027").unlink()
    (repo / ".git" / "packed-refs").write_text("# pack-refs with: peeled fully-peeled sorted\nabc123 refs/heads/main\ndef456 refs/heads/platform/build-2027\n")
    assert ba.head_from_files(repo) == ("def456", "platform/build-2027")
    (repo / ".git" / "HEAD").write_text("0123abcd\n")  # a detached head
    assert ba.head_from_files(repo) == ("0123abcd", None)
    assert ba.head_from_files(tmp_path / "nowhere") == (None, None)
    state = ba.git_state(tmp_path / "nowhere")
    assert state["hash"] is None and state["dirty"] is None
    assert ba.git_state()["hash"] == ba.head_from_files(ba.ROOT)[0]  # the two ways agree on this checkout
