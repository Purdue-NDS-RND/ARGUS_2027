# PLT-14: Ground station tools

Flag: none (tools and a document for the ground laptop) · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-14:' platform/build-2027`

## What it solves

After a localisation flight the team has 30 minutes to hand in a list that costs points for every wrong row, and the list and its evidence are on the aircraft. The laptop had no way to bring the attempt down and know it arrived whole, and nothing to read the list with but a text editor.

## Architecture

- `tools/judges_list_view.py` (new, no ROS): the list as a table, what is wrong with it, a one-file map page, the difference from an earlier list.
- `tools/attempt_download.py` (new; run on the laptop itself, also through the wrapper: the image has no ssh and no rsync): the attempt's bundle from the aircraft by rsync over ssh, the list first when asked, then checked against the bundle's own manifest.
- `tools/ground_station.sh` (new): runs one tool, or any command with ROS sourced, in the repository's Humble image on a Jazzy laptop.
- `docs/ops/ground_station.md` (new): QGroundControl's setup and its place in an autonomous flight, the clock, the download, the list.

## Flow

- After the landing: the aircraft bundles the attempt → `attempt_download.py pilot@aircraft <attempt> --list-first` → `THE LIST IS HERE` → `judges_list_view.py <list> --site <site.yaml> --map list.html` → `LIST CLEAN` or each problem by line, and the map in a browser → the rest of the bundle arrives → `DOWNLOADED AND VERIFIED`.

## Code

- The list's checks (format, one digit, an ID twice, a place on the earth): `tools/judges_list_view.py:50`; against the site (outside the survey area, two IDs on one spot): `:119`; the map: `:161`.
- The host and the attempt's name checked before they go into a command: `tools/attempt_download.py:48`; the copy and the check against the manifest: `:67`.
- The command's words handed to the container as arguments, not as shell text: `tools/ground_station.sh:70`.

## Evidence

```
$ pytest tools/test/test_judges_list_view.py tools/test/test_attempt_download.py tools/test/test_ground_station.py   (as scripts/check.sh runs them)
14 passed, 11 passed, 4 passed
$ tools/ground_station.sh rsync --version; tools/ground_station.sh ssh -V      (in the image)
ground_station: line 1: exec: rsync: not found
ground_station: line 1: exec: ssh: not found
$ tools/ground_station.sh judges_list_view.py <a sim flight's judges_list.csv>      (in the Humble container, on this laptop)
ID  Latitude      Longitude
0   34.1752848    -118.4821071
...
8   34.1752185    -118.4820729

8 marker(s)

LIST CLEAN: nothing found wrong (no site file given: the places were not checked)
```

The download's tests play ssh and rsync from a folder: a bundle built by the real bundler is copied, cut off, taken up again, changed by one byte and found. The tests were written after the tools.

## Spec check

spec-auditor, two rounds: INCOMPLETE on one point each. Round one: the bundle download could not run in the container (the image has no ssh and no rsync, run and seen; the mounted keys would not have been found). It now runs on the laptop itself through the same wrapper, a DEVIATION from "all in the Humble container" recorded for the lead. Round two: that accepted as recorded; the wrapper's test and the document named the slot clock, which is a later feature's, and no longer do (OPS-05 adds its section with its own commit). The viewer, the download's logic, the QGroundControl document and the wrapper DONE.

## Not done

- Nothing has been run on a club laptop, against the aircraft, or over the telemetry radio (why PARTIAL; "Needs a person"). The download has met only a stand-in for ssh and rsync.
- QGroundControl's setup is a document, not a settings file: the session has no QGroundControl.
- The viewer checks a list; it does not edit one. Taking a row out is done by hand.
- The radios' frequencies and powers are organiser question 22.
