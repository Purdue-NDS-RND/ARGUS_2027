"""The marker tracker's tracks as argus_interfaces messages (LOC-7). No ROS imports: the message classes are handed in.

`MarkerTrackArray` goes out on /loc/tracks after every processed frame.
A `MarkerTrack` has no class, so only `marker` tracks are in it; the
boundary markers are the tracker's own business (they decide `outside`).
Positions are in the local ENU frame MAVROS's pose is in.
"""

from __future__ import annotations

from typing import Any, Iterable

TRACKS_TOPIC = "/loc/tracks"
LOCAL_FRAME = "map"


def _stamp(stamp: Any, stamp_ns: int) -> None:
    stamp.sec, stamp.nanosec = int(stamp_ns) // 1_000_000_000, int(stamp_ns) % 1_000_000_000


def encode_tracks(states: Iterable[Any], stamp_ns: int, messages: Any) -> Any:
    """A MarkerTrackArray of the marker tracks. `messages` has MarkerTrack and MarkerTrackArray (argus_interfaces.msg)."""
    array = messages.MarkerTrackArray()
    _stamp(array.header.stamp, stamp_ns)
    array.header.frame_id = LOCAL_FRAME
    tracks = []
    for state in states:
        if state.class_id != "marker":
            continue
        track = messages.MarkerTrack()
        _stamp(track.header.stamp, state.updated_ns)
        track.header.frame_id = LOCAL_FRAME
        track.track_id = int(state.track_id)
        track.state = int(state.state)
        track.position_enu.x, track.position_enu.y, track.position_enu.z = float(state.east_m), float(state.north_m), float(state.up_m)
        track.covariance_en = [float(value) for row in state.covariance for value in row]
        track.digit_log_prob = [float(value) for value in state.digit_log_prob]
        track.digit = int(state.digit)
        track.sightings = int(state.sightings)
        track.outside = bool(state.outside)
        tracks.append(track)
    array.tracks = tracks
    return array
