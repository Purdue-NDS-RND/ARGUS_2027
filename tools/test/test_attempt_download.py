"""PLT-14: attempt bundle download (tools/attempt_download.py), against a stand-in for ssh and rsync. No ROS, no network."""

import shutil
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import attempt_download as dl  # noqa: E402
import test_post_flight as flights  # noqa: E402  (a bundle built as the bundler builds one)

NAME = "2027-05-21_2_localisation"


class Aircraft:
    """ssh and rsync, played from a folder: what the tool is given to run, and a copy made as rsync would make it."""

    def __init__(self, attempts: Path, fail_copy: int = 0, cut_after: int = None) -> None:
        self.attempts, self.fail_copy, self.cut_after, self.commands = attempts, fail_copy, cut_after, []

    def __call__(self, command, **_kwargs):
        self.commands.append(command)
        if command[0] == "ssh":
            return SimpleNamespace(returncode=0, stdout="\n".join(sorted(path.name for path in self.attempts.iterdir())) + "\n", stderr="")
        assert command[0] == "rsync" and "--partial" in command
        source, target = command[-2].split(":", 1)[1], Path(command[-1])
        relative = source.split("attempts/", 1)[1]
        here = self.attempts / relative
        if self.fail_copy:
            self.fail_copy -= 1
            return SimpleNamespace(returncode=23)
        if here.is_file():
            shutil.copy2(here, target / here.name)
        elif here.is_dir():
            files = sorted(path for path in here.rglob("*") if path.is_file())
            for path in files[: self.cut_after]:
                (target / path.relative_to(here)).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, target / path.relative_to(here))
            if self.cut_after is not None:
                return SimpleNamespace(returncode=30)
        else:
            return SimpleNamespace(returncode=23)
        return SimpleNamespace(returncode=0)


@pytest.fixture()
def aircraft(tmp_path):
    made = flights.make_bundle(tmp_path / "air", flights.clean_flight(), mission="localisation")
    (made / "judges_list.csv").write_text("ID,Latitude,Longitude\n3,34.1757000,-118.4818000\n")
    assert made.name == NAME
    return Aircraft(made.parent)


def test_an_attempt_is_copied_and_checked_against_its_own_manifest(tmp_path, aircraft, capsys):
    out = tmp_path / "ground"
    assert dl.main(["pilot@aircraft", NAME, "--out", str(out)], run=aircraft) == 0
    assert capsys.readouterr().out.strip() == f"DOWNLOADED AND VERIFIED: {out / NAME}"
    assert aircraft.commands == [["rsync", "-a", "--partial", "--info=stats1", "-e", "ssh -o BatchMode=yes", f"pilot@aircraft:ARGUS_2027/attempts/{NAME}/", f"{out / NAME}/"]]
    assert (out / NAME / "manifest.json").is_file() and sorted(path.name for path in (out / NAME / "dataflash").iterdir()) == ["00000012.BIN"]
    # The aircraft's copy is as it was: nothing there is changed.
    assert (aircraft.attempts / NAME / "manifest.json").is_file()


def test_the_list_comes_first_when_asked_and_the_rest_follows(tmp_path, aircraft, capsys):
    out = tmp_path / "ground"
    assert dl.main(["pilot@aircraft", NAME, "--out", str(out), "--list-first"], run=aircraft) == 0
    lines = capsys.readouterr().out.strip().splitlines()
    assert lines[0].startswith(f"THE LIST IS HERE: {out / NAME / 'judges_list.csv'}") and lines[-1].startswith("DOWNLOADED AND VERIFIED")
    assert aircraft.commands[0][-2].endswith(f"{NAME}/judges_list.csv") and aircraft.commands[1][-2].endswith(f"{NAME}/")
    # A bundle with no list: said, and the bundle still comes.
    (aircraft.attempts / NAME / "judges_list.csv").unlink()
    again = tmp_path / "again"
    assert dl.main(["pilot@aircraft", NAME, "--out", str(again), "--list-first"], run=aircraft) == 0
    assert "judges_list.csv could not be copied first" in capsys.readouterr().err


def test_a_copy_that_is_not_whole_is_found_and_a_copy_that_stopped_says_to_run_it_again(tmp_path, aircraft, capsys):
    out = tmp_path / "ground"
    cut = Aircraft(aircraft.attempts, cut_after=3)
    assert dl.main(["pilot@aircraft", NAME, "--out", str(out)], run=cut) == 2 and "Run the same command again: it goes on where it stopped" in capsys.readouterr().err
    assert dl.main(["pilot@aircraft", NAME, "--out", str(out)], run=aircraft) == 0  # the second run finishes it
    capsys.readouterr()
    # A file that changed on the way (or on the laptop's disk) is found by its sha256.
    log = out / NAME / "dataflash" / "00000012.BIN"
    log.write_bytes(log.read_bytes()[:-1] + bytes([log.read_bytes()[-1] ^ 0xFF]))
    still = Aircraft(tmp_path / "nothing")  # an rsync that copies nothing new
    (tmp_path / "nothing" / NAME).mkdir(parents=True)
    assert dl.main(["pilot@aircraft", NAME, "--out", str(out)], run=still) == 1 and "NOT WHOLE: dataflash: dataflash/00000012.BIN is not the file that was bundled" in capsys.readouterr().out
    # A folder with no manifest was never bundled.
    assert dl.main(["pilot@aircraft", NAME, "--out", str(tmp_path / "empty")], run=still) == 1 and "has no manifest.json" in capsys.readouterr().out


def test_the_attempts_on_the_aircraft_are_listed_and_only_attempts(tmp_path, aircraft, capsys):
    (aircraft.attempts / "slot_clock.json").write_text("{}")
    assert dl.main(["pilot@aircraft", "--list"], run=aircraft) == 0
    assert capsys.readouterr().out.strip() == NAME and aircraft.commands == [["ssh", "-o", "BatchMode=yes", "pilot@aircraft", "ls -1 ARGUS_2027/attempts"]]


@pytest.mark.parametrize(
    "arguments, word",
    [
        (["pilot@aircraft; rm -rf x", NAME], "is not a host"),
        (["-oProxyCommand=x", NAME], None),  # argparse takes it for an option: a usage error, never a host
        (["pilot@aircraft", "../../etc"], "is not an attempt's name"),
        (["pilot@aircraft", NAME + "; ls"], "is not an attempt's name"),
        (["pilot@aircraft", NAME, "--remote-dir", "attempts/../.."], "is not a folder this tool will read from"),
        (["pilot@aircraft", NAME, "--remote-dir", "x; ls"], "is not a folder this tool will read from"),
        (["pilot@aircraft"], "name an attempt, or --list"),
    ],
)
def test_a_host_or_a_name_that_could_be_a_command_is_refused_before_anything_runs(tmp_path, capsys, arguments, word):
    ran = []
    if word is None:
        with pytest.raises(SystemExit):
            dl.main(arguments, run=lambda *a, **k: ran.append(a))
    else:
        assert dl.main(arguments + ["--out", str(tmp_path)], run=lambda *a, **k: ran.append(a)) == 2 and word in capsys.readouterr().err
    assert ran == []
