# Code freeze, field tuning and rollback (OPS-06)

Three things, in the order they are needed: the freeze before the competition, what may still change on the field, and the two ways back when the 2027 code misbehaves.

## The freeze (around 19 May 2027)

1. The lead picks the commit on `main` that passed the gates (G0 to G6 green, G8's real flight done) and tags it: `git tag -a comp-2027-rc1 -m "release candidate" <commit>`, then pushes the tag. Nothing else marks the freeze.
2. On the aircraft and on the ground laptop: check out that tag, build it, and make the 2026 fallback ready beside it: `platform/jetson/fallback.sh --apply` (it clones this checkout at `comp-2026` into `~/ARGUS_2026_fallback` and builds it; no network needed).
3. From then on, before any flight: `python3 tools/freeze_check.py --worktree`. It must say `FREEZE HOLDS`.

## What may change after it

| Kind | Files | How |
| --- | --- | --- |
| Tuning | A value in a config YAML: `src/<package>/config/**.yaml` | Commit the change with `[tuning: <reason>]` in the commit message before it is flown: `freeze_check.py` breaks the freeze on a tuned value with no such commit, or one edited and not committed. The pull request, with the same marker in its description (the PR gate asks for it), follows when there is a network, and is reviewed as any other. |
| The day's own | A flag profile (`profiles/*.yaml`), a day file (`field/*.yaml`), the scoring model's inputs (`tools/score_model.yaml`) | As above, without the waiver. |
| Documents | `docs/`, any `.md` | As above. |

Anything else is code and waits for after the competition. `tools/freeze_check.py` lists every changed tuned value as `key: old -> new`: that list is the field's tuning log, and it is how a value is put back (`git checkout comp-2027-rc1 -- <file>`).

A code change that cannot wait (a crash found on the field) is the lead's decision, made knowing that `freeze_check.py` will say `FREEZE BROKEN` and name the file, and that the gates were not flown on the result.

## The two ways back

**First: fly the 2026 behaviour from the 2027 workspace.** Every 2027 behaviour is behind a flag, and with none on the nodes run the 2026 code:

```bash
python3 tools/go.py --profile comp_2026 --sequence package_delivery_live.yaml --params live_target_mission.yaml \
    --start-camera mipi --detector target_cv --camera /camera/image=30 --param-dump <today's dump>
```

This is the way back from a flag that misbehaves. It takes as long as typing the line. Gate G0 and its re-checks are the proof that flags off is `comp-2026`.

**Second: the 2026 workspace.** When the 2027 workspace itself is in doubt (a build that fails, a file half edited, something that misbehaves with every flag off):

```bash
platform/jetson/fallback.sh --check                                  # [ok] clone, at_tag, build
platform/jetson/fallback.sh launch package_delivery_live.launch.py   # the June 2026 code, from its own build
```

`launch` starts nothing unless the fallback is a clean clone at the tag with a build of that commit, and the boot settings are the 2026 start-up. The 2026 launch files open the camera themselves and expect MAVROS as 2026 started it, and the 2027 stack runs with its own settings, so on an aircraft that was flying 2027 code this is the step that comes first:

```bash
sudoedit /etc/argus/boot.env          # ARGUS_BOOT_MAVROS=apm   and   ARGUS_BOOT_CAMERA=off   (the shipped values)
sudo systemctl restart argus-mavros argus-camera
```

If it has not been done, `launch` says which setting is 2027's and stops. The preflight is not part of the 2026 code: read the ground station as in 2026.

What the fallback does not have: the fence and failsafe baseline are the flight controller's and stay as loaded (PLT-08); nothing of 2027's missions, tools or checks.

## Rehearse it

Before the competition, time both ways back once on the aircraft's bench: from "something is wrong" to the 2026 delivery's nodes running. The second way's `--apply` (the clone and the build) is done at the freeze, not on the field.
