"""sim/fence_upload.py: putting the site's geofence on the flight controller.

The upload is the MAVLink mission protocol with mission type FENCE, then the
FENCE_* parameters, then a read-back. The tests play the autopilot's side of
that conversation, including the ways it goes wrong: a lost message, a
rejected upload, a parameter that does not stick. No ROS and no pymavlink.
"""

import subprocess
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import fence_upload  # noqa: E402
import site_file  # noqa: E402

APOLLO = SIM / "sites" / "apollo_xi.yaml"
ACCEPTED, ERROR = 0, 1


class FakeAutopilot:
    """The autopilot's half of the fence upload, as a fence_upload link."""

    def __init__(self, drop_first_request=False, ack=ACCEPTED, param_error=None, stored_count=None, unknown_params=()):
        self.inbox = []
        self.items = {}
        self.params = {}
        self.expected = 0
        self.sends = []
        self.drop_first_request = drop_first_request
        self.ack = ack
        self.param_error = param_error or {}
        self.stored_count = stored_count
        self.unknown_params = set(unknown_params)

    # what the uploader calls
    def send_count(self, count):
        self.sends.append(("count", count))
        self.expected = count
        self.items = {}
        if self.drop_first_request:
            self.drop_first_request = False
            return
        self.inbox.append({"type": "MISSION_REQUEST_INT", "seq": 0})

    def send_item(self, item):
        self.sends.append(("item", item["seq"]))
        self.items[item["seq"]] = item
        if len(self.items) < self.expected:
            self.inbox.append({"type": "MISSION_REQUEST_INT", "seq": max(self.items) + 1})
        else:
            self.inbox.append({"type": "MISSION_ACK", "result": self.ack})

    def set_param(self, name, value):
        self.sends.append(("param", name))
        if name in self.unknown_params:
            return
        self.params[name] = value + self.param_error.get(name, 0.0)
        self.inbox.append({"type": "PARAM_VALUE", "name": name, "value": self.params[name]})

    def request_count(self):
        self.sends.append(("list",))
        count = len(self.items) if self.stored_count is None else self.stored_count
        self.inbox.append({"type": "MISSION_COUNT", "count": count})

    def recv(self, types, timeout_s):
        for i, message in enumerate(self.inbox):
            if message["type"] in types:
                return self.inbox.pop(i)
        return None


@pytest.fixture
def plan():
    return site_file.fence_plan(site_file.load_site(APOLLO))


def test_upload_sends_every_vertex_then_the_parameters_then_reads_back(plan):
    autopilot = FakeAutopilot()
    report = fence_upload.upload(autopilot, plan, timeout_s=0.01)
    assert [autopilot.items[seq] for seq in sorted(autopilot.items)] == plan.items
    assert autopilot.params == {name: float(value) for name, value in plan.params.items()}
    assert report == {"vertices": len(plan.items), "params": autopilot.params}
    kinds = [send[0] for send in autopilot.sends]
    # The fence is switched on only after the polygon is in place and read back.
    assert kinds.index("count") < kinds.index("item") < kinds.index("list") < kinds.index("param")
    assert max(i for i, kind in enumerate(kinds) if kind == "item") < kinds.index("list")


def test_upload_retries_when_the_first_request_is_lost(plan):
    autopilot = FakeAutopilot(drop_first_request=True)
    fence_upload.upload(autopilot, plan, timeout_s=0.01)
    assert [send for send in autopilot.sends if send[0] == "count"] == [("count", len(plan.items))] * 2
    assert len(autopilot.items) == len(plan.items)


def test_upload_gives_up_when_the_autopilot_never_answers(plan):
    class Silent(FakeAutopilot):
        def send_count(self, count):
            self.sends.append(("count", count))

    autopilot = Silent()
    with pytest.raises(fence_upload.UploadError, match="no answer"):
        fence_upload.upload(autopilot, plan, timeout_s=0.01)
    assert len([send for send in autopilot.sends if send[0] == "count"]) == fence_upload.RETRIES
    assert not autopilot.params  # the fence is not switched on without a polygon


def test_upload_fails_when_the_autopilot_rejects_the_fence(plan):
    autopilot = FakeAutopilot(ack=ERROR)
    with pytest.raises(fence_upload.UploadError, match="rejected.*1"):
        fence_upload.upload(autopilot, plan, timeout_s=0.01)
    assert not autopilot.params


def test_upload_fails_when_the_read_back_count_differs(plan):
    autopilot = FakeAutopilot(stored_count=len(plan.items) - 1)
    with pytest.raises(fence_upload.UploadError, match="holds"):
        fence_upload.upload(autopilot, plan, timeout_s=0.01)
    assert not autopilot.params


def test_upload_fails_when_a_parameter_does_not_stick(plan):
    autopilot = FakeAutopilot(param_error={"FENCE_ALT_MAX": -20.0})
    with pytest.raises(fence_upload.UploadError, match="FENCE_ALT_MAX"):
        fence_upload.upload(autopilot, plan, timeout_s=0.01)


def test_upload_fails_when_a_parameter_does_not_exist(plan):
    autopilot = FakeAutopilot(unknown_params={"FENCE_TYPE"})
    with pytest.raises(fence_upload.UploadError, match="FENCE_TYPE"):
        fence_upload.upload(autopilot, plan, timeout_s=0.01)


def test_fence_enable_is_the_last_parameter_set(plan):
    autopilot = FakeAutopilot()
    fence_upload.upload(autopilot, plan, timeout_s=0.01)
    assert [send[1] for send in autopilot.sends if send[0] == "param"][-1] == "FENCE_ENABLE"


def test_cli_dry_run_prints_the_plan_without_a_flight_controller():
    result = subprocess.run(
        [sys.executable, str(SIM / "fence_upload.py"), "--site", str(APOLLO), "--dry-run"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    plan = site_file.fence_plan(site_file.load_site(APOLLO))
    assert f"{len(plan.items)} vertices" in result.stdout
    assert "FENCE_ENABLE = 1" in result.stdout
    assert "dry run" in result.stdout


def test_cli_refuses_a_bad_site_before_connecting(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: nowhere\n")
    result = subprocess.run(
        [sys.executable, str(SIM / "fence_upload.py"), "--site", str(bad)], capture_output=True, text=True
    )
    assert result.returncode == 2
    assert "bad.yaml" in result.stderr


def test_cli_refuses_to_upload_an_unverified_site_unless_told_to():
    """A placeholder fence must not reach a real flight controller by accident."""
    assert site_file.load_site(APOLLO).verified is False
    result = subprocess.run(
        # A port nothing listens on: if the guard failed, this would hang on the heartbeat, not exit 2.
        [sys.executable, str(SIM / "fence_upload.py"), "--site", str(APOLLO), "--url", "udpin:127.0.0.1:14599"],
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode == 2
    assert "not verified" in result.stderr


def test_the_sim_start_up_is_the_only_caller_that_allows_an_unverified_site():
    repo = SIM.parent
    users = set()
    for folder in ("sim", "scripts", "tools", "src", "profiles", ".github"):
        for path in (repo / folder).rglob("*"):
            if path.suffix not in (".sh", ".py", ".yaml", ".yml", ".xml") or not path.is_file():
                continue
            if any(part in (".state", ".ws", "__pycache__", "test") for part in path.parts):
                continue
            if "--allow-unverified" in path.read_text(errors="replace"):
                users.add(path.relative_to(repo).as_posix())
    assert users == {"sim/container/start.sh", "sim/fence_upload.py"}
