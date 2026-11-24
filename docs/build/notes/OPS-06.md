# OPS-06: Code freeze, field tuning and rollback

Flag: none (a check, a script that keeps a second workspace, and a procedure) · Status: PARTIAL · Commits: `git log --oneline --grep '^OPS-06:' platform/build-2027`

## What it solves

A freeze that is only a date is broken by the first field edit, and a fallback that has not been built is no fallback. This makes the freeze something a command can check, makes the field's tuning a list that can be read back, and keeps the June 2026 code built beside the 2027 workspace, launched only when it is exactly that code.

## Architecture

- `tools/freeze_check.py` (new, no ROS): what differs from the tag `comp-2027-rc1`, sorted into tuning (each changed value listed, with the reason its commit gives), the day's own files, documents, and code. Code breaks the freeze, and so does a tuned value with no reason.
- `platform/jetson/fallback.sh` (new, on the Jetson scripts' step runner): `--check`, `--apply` and `launch` for `~/ARGUS_2026_fallback`, a clone of the checkout at `comp-2026` with its own build.
- `docs/ops/freeze_and_rollback.md` (new): the freeze, what may change after it and how, the two ways back.

## Flow

- The freeze: the lead tags `comp-2027-rc1` → `fallback.sh --apply` on the aircraft → before every flight `python3 tools/freeze_check.py --worktree` → `FREEZE HOLDS`, with the tuned values listed, or `FREEZE BROKEN` naming the file.
- The way back from a flag: `tools/go.py --profile comp_2026 ...` (the 2027 workspace, every flag off).
- The way back from the workspace: `fallback.sh launch package_delivery_live.launch.py` → the three checks (a clone; at the tag with nothing changed; built from that commit) and the boot settings (the 2026 start-up) → ROS and the fallback's own install are sourced → `ros2 launch drone_mission_demo ...` there.

## Code

- A file's kind: `tools/freeze_check.py:54`; the changed files against the tag: `:76`; the values that differ in a tuning file: `:103`; the verdict, with each tuning commit's reason: `:138`.
- The fallback's checks: `platform/jetson/fallback.sh:129` and `:165`; the launch, which starts nothing from a fallback that is not ready or under 2027's boot settings: `:222`.

## Evidence

```
$ pytest tools/test/test_freeze_check.py platform/jetson/test/test_fallback.py   (as scripts/check.sh runs them)
15 passed, 12 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ python3 tools/freeze_check.py; echo "exit $?"
freeze_check: there is no tag comp-2027-rc1 yet. At the freeze, on the commit that passed the gates: git tag -a comp-2027-rc1 -m "release candidate" <commit>
exit 2
```

Both test files run the real thing: the freeze check against small git repositories made in the test, the fallback script against a fake machine with the real git, a repository tagged `comp-2026`, and stand-ins for colcon and `ros2`. The tests were written after the code.

## Spec check

- spec-auditor, round 1: INCOMPLETE on three rows, all fixed. The fallback's build could be marked as the tag's when it was built from an edited clone (the build step ran even when the clone was not clean): it is now built only from a clone at the tag with nothing changed, and a folder that is not a clone is not built in. The procedure gave no step from the 2027 boot settings to the 2026 ones: `launch` now refuses under 2027's settings and says the two lines to change. "Fly, and open the pull request that evening" flew a tuned value with no reason on record: the reason is now asked for in the commit, before the flight, and the freeze check breaks without it. No second round: each is a tested rule.
- No flight-guardian review: no mission, executor or detector code and no config under `src/` changed.

## Not done

- The freeze itself (why PARTIAL): the tag is the lead's to set in May, and the session sets no tags.
- The fallback has not been made on any real machine: the session changes nothing outside the repo, and `comp-2026` has not been built on the Jetson by this script. "Launched in minutes" is untimed ("Needs a person").
- The `[tuning: ...]` waiver in a pull request is the gate's and is unchanged. `freeze_check.py` asks for the same marker in the commit that tunes, which is what there is on a field with no network; whether the reason is a good one is the reviewer's.
- Nothing stops a flight when the freeze is broken: `go` does not run `freeze_check.py`. The procedure asks for it before every flight.
