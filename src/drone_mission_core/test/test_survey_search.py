"""The executor's side of the survey mission (LOC-3): the client for the localisation pipeline's service and the two
calls a mission makes through the context, on the real executor and the real MissionContext with rclpy stood in for.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_executor_feature_overrides import FakeNode, executor_module, start  # noqa: E402,F401  (fixtures)
from test_executor_target_estimator import wiring  # noqa: E402,F401  (fixture)

FLAGS = "use_frame_time_pose,use_height_estimator,survey_search"
SURVEY = str(Path(__file__).resolve().parents[2] / "drone_mission_demo" / "config" / "sequences" / "search_pattern_demo.yaml")
SERVICE = "/loc/set_enabled"


class Client:
    def __init__(self, name):
        self.name, self.ready, self.requests = name, True, []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        # (The stand-in message classes hand out one request object for every call: its fields are copied as sent.)
        fields = ("data", "command", "param1", "param2", "param3", "broadcast", "confirmation")
        self.requests.append(SimpleNamespace(**{name: getattr(request, name) for name in fields}))
        return SimpleNamespace(callbacks=[], add_done_callback=lambda callback: None)


@pytest.fixture
def clients(monkeypatch):
    made = {}

    def create_client(self, service_type, name):
        made[name] = Client(name)
        return made[name]

    monkeypatch.setattr(FakeNode, "create_client", create_client, raising=False)
    return made


def test_the_flag_and_the_service_name(executor_module):
    from drone_mission_core import survey_search
    from drone_mission_core.features import FLAG_REQUIRES

    assert survey_search.SEARCH_FLAG == "survey_search" and FLAG_REQUIRES["survey_search"] == ("use_height_estimator",)
    assert survey_search.LOCALISATION_SERVICE == SERVICE and survey_search.DO_CHANGE_SPEED == 178


def test_with_the_flag_off_the_executor_makes_no_client_and_the_context_says_so(executor_module, monkeypatch, wiring, clients):
    node = start(executor_module, monkeypatch, sequence_file=SURVEY)  # the survey's own sequence, as shipped: every flag false
    assert [mission.spec.type_name for mission in node._sequence] == ["takeoff", "search_pattern", "rtl"]
    assert node._localisation_control_client is None and SERVICE not in clients
    context = node._mission_context
    assert context.localisation_control_ready() is False and context.set_localisation_enabled(True) is None
    for module in ("coverage_planner", "survey_height", "frame_projection"):  # nor is any of the survey's own code loaded
        assert f"drone_mission_core.{module}" not in sys.modules, module


def test_with_the_flag_on_the_pipeline_is_started_and_stopped_through_its_service(executor_module, monkeypatch, wiring, clients):
    with pytest.raises(ValueError, match="survey_search requires use_height_estimator"):
        start(executor_module, monkeypatch, sequence_file=SURVEY, feature_overrides="use_frame_time_pose,survey_search")
    node = start(executor_module, monkeypatch, sequence_file=SURVEY, feature_overrides=FLAGS)
    client = clients[SERVICE]
    assert node._localisation_control_client is client
    context = node._mission_context
    assert context.localisation_control_ready() is True
    client.ready = False
    assert context.localisation_control_ready() is False
    done = []
    future = context.set_localisation_enabled(True, done_callback=done.append)
    assert future is not None and client.requests[-1].data is True
    context.set_localisation_enabled(False)
    assert [request.data for request in client.requests] == [True, False]


def test_the_survey_speed_goes_to_the_flight_controller_as_do_change_speed(executor_module, monkeypatch, wiring, clients):
    node = start(executor_module, monkeypatch, sequence_file=SURVEY, feature_overrides=FLAGS)
    command = clients["/mavros/cmd/command"]
    node._mission_context.command_change_speed(4.0)
    request = command.requests[-1]
    assert (request.command, request.param1, request.param2, request.param3) == (178, 1.0, 4.0, -1.0)  # ground speed, m/s, throttle untouched
    assert request.broadcast is False and request.confirmation == 0
