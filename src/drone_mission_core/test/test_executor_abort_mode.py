"""The executor's abort asks for the mode a mission named, when one has (REC-2, REC-7; the lead's rule of 23 November 2026).

Driven through the real node class on the stand-ins of test_executor_feature_overrides.py. With no mission naming
a mode, which is every mission but the recovery once its gripper has closed or it has no position, the abort is 2026's: the
`abort_rtl_mode` parameter, RTL.
"""

import sys
import types
from pathlib import Path
from unittest import mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_executor_feature_overrides import executor_module, start  # noqa: E402,F401  (the fixture and the node's start)


def aborting(node, mode_now="GUIDED"):
    """Put the node where a failed mission leaves it and run the abort once; returns the modes it asked for."""
    asked = []
    node._mission_context.request_mode_change = lambda mode, callback=None: asked.append(mode) or mock.MagicMock()
    node._mavros_state = types.SimpleNamespace(mode=mode_now)
    node._abort_requested = True
    node._handle_abort_rtl()
    return asked


def test_with_no_mode_named_the_abort_is_the_parameters_rtl_as_in_2026(executor_module, monkeypatch):  # noqa: F811
    node = start(executor_module, monkeypatch)
    assert node._abort_mode_override is None
    assert aborting(node) == ["RTL"]
    again = start(executor_module, monkeypatch, abort_rtl_mode="SMART_RTL")  # and still whatever the parameter says
    assert aborting(again) == ["SMART_RTL"]


def test_a_mission_that_named_land_gets_a_landing_and_giving_it_back_gets_rtl_again(executor_module, monkeypatch):  # noqa: F811
    node = start(executor_module, monkeypatch)
    assert node._mission_context.named_abort_mode() is None
    node._mission_context.set_abort_mode("LAND")
    assert node._abort_mode_override == "LAND" and node._mission_context.named_abort_mode() == "LAND"
    assert aborting(node) == ["LAND"]
    other = start(executor_module, monkeypatch)
    other._mission_context.set_abort_mode("LAND")
    other._mission_context.set_abort_mode(None)
    assert aborting(other) == ["RTL"]


def test_the_abort_is_confirmed_by_the_named_mode_not_by_rtl(executor_module, monkeypatch):  # noqa: F811
    node = start(executor_module, monkeypatch)
    node._mission_context.set_abort_mode("LAND")
    assert aborting(node, mode_now="LAND") == []  # already landing: nothing more is asked
    assert aborting(node, mode_now="RTL") == ["LAND"]  # RTL with the bucket on is not what was asked for


@pytest.mark.parametrize("mode", ["RTL", "AUTO", "GUIDED", "land", ""])
def test_only_land_can_be_named(executor_module, monkeypatch, mode):  # noqa: F811
    node = start(executor_module, monkeypatch)
    with pytest.raises(ValueError, match="LAND"):
        node._mission_context.set_abort_mode(mode)
    assert node._abort_mode_override is None
