#!/usr/bin/env python3
"""The judges' list viewer (PLT-14): read the localisation list before it is handed in.

  python3 tools/judges_list_view.py attempts/<attempt>/judges_list.csv
  python3 tools/judges_list_view.py <list> --site field/<name>/site.yaml       # held against the day's survey area
  python3 tools/judges_list_view.py <list> --site ... --map list.html          # a page to look at: the area, the markers, their IDs
  python3 tools/judges_list_view.py <list> --against <an earlier list>         # what changed

The list is the localisation mission's whole result: `ID,Latitude,Longitude`,
a row a marker (src/argus_localisation, LOC-10). A wrong row costs points and
a missing one costs none, so before it goes to the judges somebody looks at
it. This prints it as a table and says what is wrong with it:

  - a header or a row that is not the format; an ID that is not one digit;
    the same ID twice; a place that is not on the earth;
  - with the site file: a marker outside the survey area (the tracker should
    not have listed it), and each marker's distance and bearing from the
    takeoff point, to hold against where the team saw it lie;
  - two markers closer together than the markers are (--min-apart-m): one
    marker listed twice under two IDs.

The map is one HTML file with the picture drawn in it (no network, no other
file): it opens in any browser on the ground laptop.

Exit code: 0 the list is clean, 1 it has problems, 2 it could not be read.

No ROS imports; the standard library and yaml (for the site file).
"""

from __future__ import annotations

import argparse
import html
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

HEADER = "ID,Latitude,Longitude"  # argus_localisation.judges_list.LIST_HEADER (a test holds the two together)
METRES_PER_DEGREE = 111_319.49  # of latitude; east is scaled by the cosine of the latitude
MIN_APART_M = 1.0

Row = Dict[str, Any]


class ListError(ValueError):
    """The list or the site file cannot be read. The message says why."""


def read_list(path: Path) -> Tuple[List[Row], List[str]]:
    """(the rows that could be read, what is wrong with the file). A row: {"id", "latitude", "longitude", "line"}."""
    try:
        lines = Path(path).read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as problem:
        raise ListError(f"{path}: {problem}") from problem
    problems: List[str] = []
    if not lines or lines[0].strip() != HEADER:
        problems.append(f"line 1: the header must be exactly '{HEADER}', and is '{lines[0].strip() if lines else ''}'")
    rows: List[Row] = []
    for number, line in enumerate(lines[1:], start=2):
        if not line.strip():
            problems.append(f"line {number}: an empty line")
            continue
        parts = [part.strip() for part in line.split(",")]
        if len(parts) != 3:
            problems.append(f"line {number}: expected ID,Latitude,Longitude and found {len(parts)} value(s)")
            continue
        try:
            latitude, longitude = float(parts[1]), float(parts[2])
        except ValueError:
            problems.append(f"line {number}: '{parts[1]},{parts[2]}' is not a latitude and a longitude")
            continue
        if not (math.isfinite(latitude) and math.isfinite(longitude) and -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0) or (latitude == 0.0 and longitude == 0.0):
            problems.append(f"line {number}: {parts[1]},{parts[2]} is not a place on the earth")
            continue
        if not (len(parts[0]) == 1 and parts[0].isdigit()):
            problems.append(f"line {number}: the ID '{parts[0]}' is not one digit")
        rows.append({"id": parts[0], "latitude": latitude, "longitude": longitude, "line": number})
    seen: Dict[str, int] = {}
    for row in rows:
        if row["id"] in seen:
            problems.append(f"line {row['line']}: ID {row['id']} is also on line {seen[row['id']]}")
        seen.setdefault(row["id"], row["line"])
    return rows, problems


def metres(latitude: float, longitude: float, origin: Tuple[float, float]) -> Tuple[float, float]:
    """(east, north) of a place from an origin, in metres."""
    north = (latitude - origin[0]) * METRES_PER_DEGREE
    east = (longitude - origin[1]) * METRES_PER_DEGREE * math.cos(math.radians(0.5 * (latitude + origin[0])))
    return east, north


def read_site(path: Path) -> Dict[str, Any]:
    """The takeoff point, the survey area and the flight boundary of a site file (sim/sites/, or tools/field_setup.py's site.yaml)."""
    import yaml

    try:
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        place = lambda item: (float(item["latitude_deg"]), float(item["longitude_deg"]))  # noqa: E731
        site = {"name": str(raw.get("name", Path(path).stem)), "takeoff": place(raw["takeoff_point"]), "survey_area": [place(item) for item in raw.get("survey_area") or []],
                "flight_boundary": [place(item) for item in raw.get("flight_boundary") or []], "verified": bool(raw.get("verified", False))}  # fmt: skip
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError, AttributeError) as problem:
        raise ListError(f"{path}: not a site file ({problem})") from problem
    if len(site["survey_area"]) < 3:
        raise ListError(f"{path}: the site file has no survey_area to hold the list against")
    return site


def _inside(point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]) -> bool:
    east, north = point
    inside = False
    for (e1, n1), (e2, n2) in zip(polygon, list(polygon[1:]) + [polygon[0]]):
        if (n1 > north) != (n2 > north) and east < e1 + (north - n1) * (e2 - e1) / (n2 - n1):
            inside = not inside
    return inside


def place_rows(rows: Sequence[Row], site: Dict[str, Any], min_apart_m: float = MIN_APART_M) -> List[str]:
    """Give each row its east, north, distance and bearing from the takeoff point and whether it is in the survey area; return what is wrong."""
    problems = []
    area = [metres(lat, lon, site["takeoff"]) for lat, lon in site["survey_area"]]
    for row in rows:
        row["east_m"], row["north_m"] = metres(row["latitude"], row["longitude"], site["takeoff"])
        row["distance_m"] = math.hypot(row["east_m"], row["north_m"])
        row["bearing_deg"] = math.degrees(math.atan2(row["east_m"], row["north_m"])) % 360.0
        row["in_area"] = _inside((row["east_m"], row["north_m"]), area)
        if not row["in_area"]:
            problems.append(f"line {row['line']}: ID {row['id']} is outside the survey area ({row['distance_m']:.0f} m from the takeoff point, bearing {row['bearing_deg']:.0f})")
    for index, first in enumerate(rows):
        for second in rows[index + 1 :]:
            apart = math.hypot(first["east_m"] - second["east_m"], first["north_m"] - second["north_m"])
            if apart < min_apart_m:
                problems.append(f"lines {first['line']} and {second['line']}: IDs {first['id']} and {second['id']} are {apart:.2f} m apart: one marker listed twice?")
    return problems


def changes(rows: Sequence[Row], earlier: Sequence[Row]) -> List[str]:
    """What differs from an earlier list: IDs added, gone, and moved (by how far)."""
    now, before = {row["id"]: row for row in rows}, {row["id"]: row for row in earlier}
    out = [f"ID {key}: new" for key in now if key not in before] + [f"ID {key}: gone" for key in before if key not in now]
    for key in now:
        if key in before:
            east, north = metres(now[key]["latitude"], now[key]["longitude"], (before[key]["latitude"], before[key]["longitude"]))
            if math.hypot(east, north) >= 0.05:
                out.append(f"ID {key}: moved {math.hypot(east, north):.2f} m")
    return sorted(out)


def table(rows: Sequence[Row]) -> str:
    placed = bool(rows) and "distance_m" in rows[0]
    lines = ["ID  Latitude      Longitude" + ("       From takeoff     In the area" if placed else "")]
    for row in sorted(rows, key=lambda item: item["id"]):
        line = f"{row['id']:<3} {row['latitude']:<13.7f} {row['longitude']:<15.7f}"
        if placed:
            line += f" {row['distance_m']:6.1f} m at {row['bearing_deg']:3.0f}   {'yes' if row['in_area'] else 'NO'}"
        lines.append(line.rstrip())
    return "\n".join(lines)


def map_html(rows: Sequence[Row], site: Dict[str, Any], title: str, problems: Sequence[str]) -> str:
    """One page with the survey area, the flight boundary, the takeoff point and the markers, drawn as SVG. North is up."""
    area = [metres(lat, lon, site["takeoff"]) for lat, lon in site["survey_area"]]
    fence = [metres(lat, lon, site["takeoff"]) for lat, lon in site["flight_boundary"]]
    points = area + [(row["east_m"], row["north_m"]) for row in rows] + [(0.0, 0.0)]
    east_min, east_max = min(e for e, _n in points) - 10.0, max(e for e, _n in points) + 10.0
    north_min, north_max = min(n for _e, n in points) - 10.0, max(n for _e, n in points) + 10.0
    scale = 900.0 / max(east_max - east_min, north_max - north_min)
    width, height = (east_max - east_min) * scale, (north_max - north_min) * scale

    def xy(point: Tuple[float, float]) -> str:
        return f"{(point[0] - east_min) * scale:.1f},{(north_max - point[1]) * scale:.1f}"

    shapes = []
    if len(fence) >= 3:
        shapes.append(f'<polygon points="{" ".join(xy(p) for p in fence)}" fill="none" stroke="#b00" stroke-width="2" stroke-dasharray="8 6"/>')
    shapes.append(f'<polygon points="{" ".join(xy(p) for p in area)}" fill="#eef6ee" stroke="#383" stroke-width="2"/>')
    x0, y0 = xy((0.0, 0.0)).split(",")
    shapes.append(f'<circle cx="{x0}" cy="{y0}" r="6" fill="#06c"/><text x="{float(x0) + 9:.1f}" y="{float(y0) + 4:.1f}" font-size="13">takeoff</text>')
    for row in rows:
        x, y = xy((row["east_m"], row["north_m"])).split(",")
        colour = "#111" if row["in_area"] else "#b00"
        shapes.append(f'<circle cx="{x}" cy="{y}" r="9" fill="#fff" stroke="{colour}" stroke-width="2"/><text x="{x}" y="{float(y) + 5:.1f}" font-size="14" font-weight="bold" text-anchor="middle" fill="{colour}">'
                      f"{html.escape(row['id'])}</text>")  # fmt: skip
    bar = 10.0 * scale
    shapes.append(f'<line x1="20" y1="{height - 20:.1f}" x2="{20 + bar:.1f}" y2="{height - 20:.1f}" stroke="#111" stroke-width="3"/><text x="20" y="{height - 26:.1f}" font-size="12">10 m</text>')
    shapes.append(f'<text x="{width - 30:.1f}" y="24" font-size="16">N ↑</text>')
    wrong = "".join(f"<li>{html.escape(problem)}</li>" for problem in problems) or "<li>nothing found wrong</li>"
    body = "".join(f"<tr><td>{html.escape(row['id'])}</td><td>{row['latitude']:.7f}</td><td>{row['longitude']:.7f}</td><td>{row['distance_m']:.1f} m at {row['bearing_deg']:.0f}</td>"
                   f"<td>{'yes' if row['in_area'] else 'NO'}</td></tr>" for row in sorted(rows, key=lambda item: item["id"]))  # fmt: skip
    return (f"<!doctype html><html><head><meta charset=\"utf-8\"><title>{html.escape(title)}</title>"
            "<style>body{font-family:sans-serif;margin:1.5em}td,th{padding:2px 10px;text-align:left}svg{border:1px solid #999}</style></head><body>"
            f"<h1>{html.escape(title)}</h1><p>{len(rows)} marker(s); site {html.escape(site['name'])}"
            f"{'' if site['verified'] else ' (NOT VERIFIED: its coordinates are placeholders)'}. Green: the survey area. Dashed red: the flight boundary. North is up.</p>"
            f"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{width:.0f}\" height=\"{height:.0f}\">{''.join(shapes)}</svg>"
            f"<h2>What is wrong</h2><ul>{wrong}</ul><h2>The list</h2><table><tr><th>ID</th><th>Latitude</th><th>Longitude</th><th>From takeoff</th><th>In the area</th></tr>{body}</table>"
            "</body></html>\n")  # fmt: skip


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Read the judges' list before it is handed in.")
    parser.add_argument("list")
    parser.add_argument("--site", help="the day's site file: the list is held against its survey area")
    parser.add_argument("--map", help="write a page with the map here (needs --site)")
    parser.add_argument("--against", help="an earlier list: say what changed")
    parser.add_argument("--min-apart-m", type=float, default=MIN_APART_M)
    args = parser.parse_args(argv)
    try:
        rows, problems = read_list(Path(args.list))
        site = read_site(Path(args.site)) if args.site else None
        if args.map and site is None:
            raise ListError("--map needs --site: the map is drawn on the site's survey area")
        if site is not None:
            problems += place_rows(rows, site, args.min_apart_m)
        differences = changes(rows, read_list(Path(args.against))[0]) if args.against else None
    except ListError as problem:
        print(f"judges_list_view: {problem}", file=sys.stderr)
        return 2
    print(table(rows))
    print(f"\n{len(rows)} marker(s)" + ("" if site is None else f"; site {site['name']}" + ("" if site["verified"] else " (NOT VERIFIED: its coordinates are placeholders)")))
    if differences is not None:
        print("\nAgainst the earlier list:" + ("".join(f"\n  {line}" for line in differences) if differences else " no change"))
    if args.map:
        Path(args.map).write_text(map_html(rows, site, f"Judges' list: {Path(args.list).name}", problems), encoding="utf-8")
        print(f"\nmap: {args.map}")
    print("\n" + ("\n".join(f"PROBLEM: {problem}" for problem in problems) if problems else "LIST CLEAN: nothing found wrong" + ("" if site else " (no site file given: the places were not checked)")))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
