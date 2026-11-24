# The ground station (PLT-14)

The ground laptop does three things in a slot: it shows the aircraft (QGroundControl), brings the attempt's bundle down after the landing, and lets a person read the judges' list before it is handed in. Everything here runs on the club's Jazzy laptops through the repository's own Humble container, so that every laptop runs the same thing:

```bash
sim/run_sim.sh --build                                   # once per laptop: makes the image argus-sim:humble
tools/ground_station.sh <a tool of tools/> [arguments]   # runs it in that image, in this checkout, as you
tools/ground_station.sh ros2 bag info attempts/<attempt>/bag
```

The Python tools have no ROS in them and also run on the laptop itself (`python3 tools/<tool>`); the container is the way that does not depend on the laptop. One tool is always run on the laptop itself, by the same command: `attempt_download.py` needs ssh, rsync and your own keys, and the image has no ssh and no rsync. None of this has been run on a club laptop or against the aircraft yet.

## QGroundControl

QGroundControl runs on the laptop itself, not in the container.

- **Link:** the telemetry radio's serial port, at the radio's rate. Turn off QGroundControl's automatic connection to anything else (UDP, other serial devices): one link, the radio.
- **It is the ground station the flight controller listens for.** QGroundControl's MAVLink system ID stays 255. With the 2027 baseline, when its heartbeat stops for 5 s the aircraft returns to the takeoff point (`FS_GCS_ENABLE`), unless a person is flying it. Do not close QGroundControl, unplug the radio or put the laptop to sleep while the aircraft is armed.
- **During an autonomous flight it is a display.** No mode button, no "pause", no "go to", no arming from it: the flight controller logs every command with its sender, the post-flight audit finds it, and the autonomy bonus is lost (`tools/post_flight.py`).
- **Before the flight:** the fence drawn on the map is the day's (`field/<name>/fence.plan` is the file to compare with); the parameters are saved to a file for the preflight's `--param-dump`.
- **What to watch:** the mode, the battery, the messages. The stack's own lines arrive as status text: `LOC list: <n> markers` (and how many are found and not yet read) during a survey; the preflight's and the missions' lines are in the launch's own terminal on the aircraft.
- **Radio frequencies and powers** allowed at the competition are organiser question 22: the radios are the current ones until that is answered.

## After the landing: the bundle

On the aircraft the attempt's folder is made by `tools/bundle_attempt.py` (the checklist's "After each flight"). From the laptop:

```bash
tools/ground_station.sh attempt_download.py pilot@aircraft --list
tools/ground_station.sh attempt_download.py pilot@aircraft <attempt> --list-first
```

The download runs on the laptop itself (python3, ssh and rsync must be installed there), with the keys and known hosts of whoever runs it. It never answers a prompt: before the first slot, run `ssh pilot@aircraft true` once by hand, accept the aircraft's host key, and make sure the key needs no passphrase typed (an agent, or a key of its own).

`--list-first` brings `judges_list.csv` before the gigabytes of frames and says where it is. The rest follows, and every file is checked against the bundle's manifest: `DOWNLOADED AND VERIFIED`, or what is not whole. A copy that stops is taken up again by the same command. Nothing on the aircraft is changed.

## The judges' list

```bash
tools/ground_station.sh judges_list_view.py attempts/<attempt>/judges_list.csv --site field/<name>/site.yaml --map list.html
```

It prints the list by ID with each marker's distance and bearing from the takeoff point, and says what is wrong with it: a row that is not the format, an ID twice, a marker outside the survey area, two IDs on one spot. `list.html` is one page with the area and the markers drawn on it: open it in the browser and hold it against what the team saw on the field. A wrong row costs points and a missing row costs none: a row nobody believes is taken out by hand before the list is handed in, and that is the lead's call.

Then `tools/post_flight.py attempts/<attempt>` for the report and the autonomy audit.
