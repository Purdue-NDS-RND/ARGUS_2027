"""The judges' list (LOC-10): the survey's tracks as `ID,Latitude,Longitude`, and how it reaches the disk. No ROS.

Behind the node flag `features.judges_list` (with the tracker's). The list
is the localisation mission's whole result, and it must exist before the
vehicle lands, so it is written during the flight and rewritten as it
changes:

  - **Rows.** A row is a listed track (a confirmed marker inside the search
    area) that has a digit: its ID is the digit the vote gave it (LOC-8),
    its place the track's reported position (LOC-9). A listed track with no
    digit yet is "unread": it is counted and drawn on the map, and is not a
    row, because a row with a wrong ID costs points and a missing row costs
    none.
  - **Latitude and longitude.** A track's position is metres east and north
    in MAVROS's local frame. `OriginEstimate` finds where that frame's (0, 0)
    is on the earth from the vehicle's own GPS fixes, each paired with the
    local position of the same moment; it is the median of what the pairs
    say, so a fix paired with the wrong moment does not move it.
    `local_to_latlon` is the mission's `latlon_to_local_enu` backwards.
  - **Files.** `judges_list.csv` (seven decimal places: a centimetre), a
    KML with a placemark a row, and a map image. Each is written to a
    temporary file beside it, synced, and renamed onto it (`write_atomic`):
    a crash leaves the old file or the new one, never half of either.
  - **When.** `JudgesList.write` writes only when the text has changed.
    `ListWriter` holds the newest list and writes it on its own thread, at
    most once a period, so the disk is never waited for where frames are
    handled; stopping it writes what is waiting.
  - **Status text.** `status_text` is the line for the ground station: the
    count of rows, and of markers found and not yet read.

The standard library and numpy; cv2 for the map image only.
"""

from __future__ import annotations

import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Deque, List, Optional, Sequence, Tuple
from xml.sax.saxutils import escape

import numpy as np
from drone_mission_core.target_access import METRES_PER_DEGREE

FLAG = "judges_list"
LIST_HEADER = "ID,Latitude,Longitude"
CSV_NAME, KML_NAME, MAP_NAME = "judges_list.csv", "judges_list.kml", "judges_list.png"
DECIMALS = 7
STATUSTEXT_MAX_CHARS = 50  # MAVLink's STATUSTEXT
MIN_ORIGIN_PAIRS = 5  # the origin is not taken from fewer fixes than this
MAX_ORIGIN_PAIRS = 400  # the newest this many are kept
WRITE_PERIOD_S = 1.0  # the node's writer rewrites a changing list at most this often
STATUSTEXT_PERIOD_S = 2.0  # the count goes to the ground station at most this often: the link is the flight controller's
LISTED_STATES = (1, 2)  # MarkerTrack.msg: CONFIRMED, COASTING
MAP_PX = (800, 800)
MAP_MARGIN_PX = 60
MAP_MARKER_BGR = (0, 0, 200)
MAP_UNREAD_BGR = (160, 160, 160)


def _finite(*values: Any) -> bool:
    return all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) for value in values)


def _is_place(latitude: Any, longitude: Any) -> bool:
    return _finite(latitude, longitude) and abs(latitude) <= 90.0 and abs(longitude) <= 180.0


def local_to_latlon(east_m: Any, north_m: Any, origin_latitude: Any, origin_longitude: Any) -> Optional[Tuple[float, float]]:
    """(latitude, longitude) of a point east and north of the local frame's origin. None for numbers that are not usable.

    `latlon_to_local_enu` backwards: metres to degrees with the same
    constant, and east scaled at the middle latitude, as ArduPilot does.
    """
    if not _finite(east_m, north_m) or not _is_place(origin_latitude, origin_longitude):
        return None
    latitude = float(origin_latitude) + float(north_m) / METRES_PER_DEGREE
    scale = math.cos(math.radians(0.5 * (latitude + float(origin_latitude))))
    if abs(latitude) > 90.0 or scale <= 1.0e-6:
        return None
    longitude = float(origin_longitude) + float(east_m) / (METRES_PER_DEGREE * scale)
    return latitude, (longitude + 180.0) % 360.0 - 180.0


class OriginEstimate:
    """Where the local frame's (0, 0) is, from GPS fixes paired with the local position of the same moment."""

    def __init__(self) -> None:
        self._origins: Deque[Tuple[float, float]] = deque(maxlen=MAX_ORIGIN_PAIRS)
        self._lock = threading.Lock()

    @property
    def pairs(self) -> int:
        return len(self._origins)

    def add(self, latitude: Any, longitude: Any, east_m: Any, north_m: Any) -> bool:
        """One fix and where the vehicle was in the local frame at that moment. False when it is not usable."""
        if not _is_place(latitude, longitude) or not _finite(east_m, north_m):
            return False
        back = local_to_latlon(-float(east_m), -float(north_m), latitude, longitude)
        if back is None:
            return False
        with self._lock:
            self._origins.append(back)
        return True

    def origin(self) -> Optional[Tuple[float, float]]:
        """(latitude, longitude) of the local frame's origin: the median of the pairs. None before MIN_ORIGIN_PAIRS."""
        with self._lock:
            origins = list(self._origins)
        if len(origins) < MIN_ORIGIN_PAIRS:
            return None
        values = np.array(origins)
        return float(np.median(values[:, 0])), float(np.median(values[:, 1]))


@dataclass(frozen=True)
class ListRow:
    marker_id: str
    latitude: float
    longitude: float
    east_m: float
    north_m: float
    track_id: int


@dataclass(frozen=True)
class Unread:
    """A listed track with no row: no digit yet, or no origin to give it a latitude by."""

    track_id: int
    east_m: float
    north_m: float


def rows_from_tracks(tracks: Sequence[Any], origin: Optional[Tuple[float, float]]) -> Tuple[List[ListRow], List[Unread]]:
    """(the list's rows by ID, the listed tracks that are not rows) from the tracker's tracks."""
    rows: List[ListRow] = []
    unread: List[Unread] = []
    for track in sorted(tracks, key=lambda item: int(item.track_id)):
        if track.class_id != "marker" or track.state not in LISTED_STATES or getattr(track, "outside", False):
            continue
        east, north = float(track.reported_east_m), float(track.reported_north_m)
        digit = int(getattr(track, "digit", -1))
        place = local_to_latlon(east, north, origin[0], origin[1]) if origin is not None else None
        if digit < 0 or place is None:
            unread.append(Unread(int(track.track_id), east, north))
            continue
        rows.append(ListRow(str(digit), place[0], place[1], east, north, int(track.track_id)))
    rows.sort(key=lambda row: (int(row.marker_id), row.track_id))
    return rows, unread


def boundary_of(tracks: Sequence[Any]) -> List[Tuple[float, float]]:
    """The search area's corner markers that are on the tracker's list, in order round their middle (for the map's outline)."""
    corners = [(float(track.reported_east_m), float(track.reported_north_m)) for track in tracks if track.class_id == "boundary_marker" and track.state in LISTED_STATES]
    if len(corners) < 3:
        return corners
    middle = (sum(point[0] for point in corners) / len(corners), sum(point[1] for point in corners) / len(corners))
    return sorted(corners, key=lambda point: math.atan2(point[1] - middle[1], point[0] - middle[0]))


def format_csv(rows: Sequence[ListRow]) -> str:
    lines = [LIST_HEADER] + [f"{row.marker_id},{row.latitude:.{DECIMALS}f},{row.longitude:.{DECIMALS}f}" for row in rows]
    return "\n".join(lines) + "\n"


def format_kml(rows: Sequence[ListRow]) -> str:
    marks = "".join(
        f"    <Placemark><name>{escape(row.marker_id)}</name><Point><coordinates>{row.longitude:.{DECIMALS}f},{row.latitude:.{DECIMALS}f},0</coordinates></Point></Placemark>\n" for row in rows
    )
    return '<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2">\n  <Document>\n    <name>ARGUS localisation list</name>\n' + marks + "  </Document>\n</kml>\n"


def status_text(rows: Sequence[ListRow], unread: Sequence[Unread], not_written: bool = False) -> str:
    """The count for the ground station, within a STATUSTEXT's 50 characters. `not_written`: the list is not reaching the disk."""
    text = f"LOC list: {len(rows)} marker{'' if len(rows) == 1 else 's'}"
    if unread:
        text += f", {len(unread)} more unread"
    if not_written:
        text = f"LOC list NOT WRITTEN: {len(rows)} found"
    return text[:STATUSTEXT_MAX_CHARS]


def render_map(rows: Sequence[ListRow], unread: Sequence[Unread], boundary: Sequence[Tuple[float, float]]) -> Tuple[np.ndarray, Callable[[float, float], Tuple[float, float]]]:
    """(a picture of the list, north up; the function from metres east and north to its pixels).

    The search area's corner markers as an outline, each row as a dot with
    its ID, each unread marker as a grey dot with a question mark, and a
    10 m scale bar.
    """
    import cv2

    width, height = MAP_PX
    points = [(row.east_m, row.north_m) for row in rows] + [(item.east_m, item.north_m) for item in unread] + [(float(e), float(n)) for e, n in boundary]
    if not points:
        points = [(0.0, 0.0)]
    east = [point[0] for point in points]
    north = [point[1] for point in points]
    span = max(max(east) - min(east), max(north) - min(north), 10.0)
    scale = (min(width, height) - 2 * MAP_MARGIN_PX) / span
    middle = (0.5 * (max(east) + min(east)), 0.5 * (max(north) + min(north)))

    def place(east_m: float, north_m: float) -> Tuple[float, float]:
        return width / 2.0 + (east_m - middle[0]) * scale, height / 2.0 - (north_m - middle[1]) * scale

    def pixel(east_m: float, north_m: float) -> Tuple[int, int]:
        x, y = place(east_m, north_m)
        return int(round(x)), int(round(y))

    picture = np.full((height, width, 3), 255, dtype=np.uint8)
    if len(boundary) >= 2:
        outline = np.array([pixel(float(e), float(n)) for e, n in boundary], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(picture, [outline], True, (120, 120, 120), 1, cv2.LINE_AA)
    for item in unread:
        cv2.circle(picture, pixel(item.east_m, item.north_m), 6, MAP_UNREAD_BGR, -1)
        cv2.putText(picture, "?", (pixel(item.east_m, item.north_m)[0] + 10, pixel(item.east_m, item.north_m)[1] + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90, 90, 90), 1, cv2.LINE_AA)
    for row in rows:
        cv2.circle(picture, pixel(row.east_m, row.north_m), 6, MAP_MARKER_BGR, -1)
        cv2.putText(picture, row.marker_id, (pixel(row.east_m, row.north_m)[0] + 10, pixel(row.east_m, row.north_m)[1] + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
    bar = int(round(10.0 * scale))
    cv2.line(picture, (MAP_MARGIN_PX, height - 25), (MAP_MARGIN_PX + bar, height - 25), (0, 0, 0), 2)
    cv2.putText(picture, "10 m", (MAP_MARGIN_PX + bar + 8, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.arrowedLine(picture, (width - 35, 60), (width - 35, 25), (0, 0, 0), 2, cv2.LINE_AA, tipLength=0.3)
    cv2.putText(picture, "N", (width - 42, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(picture, f"{len(rows)} listed, {len(unread)} unread", (MAP_MARGIN_PX, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    return picture, place


def write_atomic(path: Any, data: bytes) -> None:
    """Replace `path` with `data`, whole or not at all: a temporary file beside it, synced, then renamed onto it."""
    path = Path(path)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        with open(temporary, "wb") as file:
            file.write(data)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise
    try:  # the rename itself, to the disk
        folder = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(folder)
        finally:
            os.close(folder)
    except OSError:
        pass


class JudgesList:
    """The list's three files in one folder, rewritten when the list changes."""

    def __init__(self, folder: Any) -> None:
        if not str(folder or "").strip():
            raise ValueError("judges_list_dir must be set: the folder the judges' list is written in")
        self.folder = Path(folder)
        self.csv_path = self.folder / CSV_NAME
        self.written = 0  # times the list was (re)written
        self.error = ""  # the last failure to write, cleared by the next write that works
        self.rows = 0
        self.kept_earlier = ""  # where a list that was already in the folder was put aside
        self._last: Optional[str] = None

    def write(self, rows: Sequence[ListRow], unread: Sequence[Unread], boundary: Sequence[Tuple[float, float]]) -> bool:
        """Write the list if it differs from what was last written. True when it was written. Never raises."""
        text = format_csv(rows)
        key = text + repr([(item.track_id, round(item.east_m, 1), round(item.north_m, 1)) for item in unread])
        if key == self._last:
            return False
        try:
            self.folder.mkdir(exist_ok=True)  # the folder itself, not the way to it
            if self._last is None:
                self._keep_earlier()
            write_atomic(self.csv_path, text.encode("ascii"))  # the judges' file first
            write_atomic(self.folder / KML_NAME, format_kml(rows).encode("utf-8"))
            import cv2

            encoded, png = cv2.imencode(".png", render_map(rows, unread, boundary)[0])
            if not encoded:
                raise OSError("the map image could not be encoded")
            write_atomic(self.folder / MAP_NAME, png.tobytes())
        except Exception as error:  # noqa: BLE001  (the list's failure is said; it must not stop the survey)
            self.error = f"the judges' list could not be written in {self.folder} ({type(error).__name__}: {error})"
            return False
        self._last, self.error, self.rows = key, "", len(rows)
        self.written += 1
        return True


    def _keep_earlier(self) -> None:
        """Before this run's first write: a list already in the folder (an earlier flight's, or this flight's before the
        node was restarted) is renamed aside, not written over. A restarted node starts with no tracks, and its
        first list is a header alone."""
        try:
            earlier = self.csv_path.read_text()
        except OSError:
            return
        if earlier.strip() in ("", LIST_HEADER):
            return
        stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime(self.csv_path.stat().st_mtime))
        kept = self.folder / f"judges_list.earlier-{stamp}.csv"
        number = 1
        while kept.exists():
            number += 1
            kept = self.folder / f"judges_list.earlier-{stamp}-{number}.csv"
        os.replace(self.csv_path, kept)
        self.kept_earlier = str(kept)


class ListWriter:
    """Holds the newest list and writes it on a thread of its own, at most once a period."""

    def __init__(self, judges_list: JudgesList, period_s: float = WRITE_PERIOD_S) -> None:
        self.judges_list = judges_list
        self.period_s = max(0.01, float(period_s))
        self._lock = threading.Lock()
        self._newest: Optional[Tuple[Any, Any, Any]] = None
        self._wake = threading.Event()
        self._stopping = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def offer(self, rows: Sequence[ListRow], unread: Sequence[Unread], boundary: Sequence[Tuple[float, float]]) -> None:
        """The list as it now stands. Writes nothing: the writer's thread (or `flush`) does."""
        with self._lock:
            self._newest = (list(rows), list(unread), list(boundary))

    def wake(self) -> None:
        """Have the writer's thread write now, without waiting out its period."""
        self._wake.set()

    def flush(self) -> bool:
        """Write the newest list offered, if there is one. True when the files were written."""
        with self._lock:
            newest, self._newest = self._newest, None
        return False if newest is None else self.judges_list.write(*newest)

    def _run(self) -> None:
        while not self._stopping.is_set():
            self._wake.wait(self.period_s)
            self._wake.clear()
            self.flush()
        self.flush()  # what was offered last, before the thread ends

    def start(self) -> None:
        if self.running:
            return
        self._stopping.clear()
        self._thread = threading.Thread(target=self._run, name="judges-list", daemon=True)
        self._thread.start()

    def stop(self, wait_s: float = 2.0) -> None:
        """Write what is waiting and end the thread. Waits at most `wait_s` for the disk."""
        thread = self._thread
        if thread is None:
            return
        self._stopping.set()
        self._wake.set()
        thread.join(timeout=max(0.0, float(wait_s)))
        self._thread = None
