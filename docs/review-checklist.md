# Pull request review checklist

The lead reviews every pull request against these rules; `/review-pr` walks through them in this order. Each finding cites the rule id and a `file:line`.

CI already enforces these, so they aren't repeated here: `scripts/check.sh` passes, a Python change under `src/` or `tools/` comes with a test change, no existing value in a config YAML changes without a `[tuning: ...]` marker, no added file is over 5 MB, and no AI attribution or local machine paths in commits, the PR title, the description or added lines. The flight-guardian covers invariants F1 to F10 in depth; the rules below ask whether its findings hold and what it can't judge.

Severity: **blocking** findings make the verdict BLOCKED. **Suggestion** findings make it CHANGES SUGGESTED.

## scope

- The diff does what the description says, and nothing it doesn't say. Read the diff before the description; a mismatch in either direction is a blocking finding.
- One issue per pull request. Drive-by refactors, renames or fixes elsewhere are blocking; they belong under "Not done" or in a new issue.
- The branch is named `<area>/<short-name>` (centring, localisation, recovery, sim, platform). Suggestion.
- Nothing that must not be in a public repo: passwords, tokens, keys, datasets, ROS bags, flight logs, training runs, TensorRT engines. Blocking.
- Every template section is filled in, with "Not done" listing what was noticed but left. Suggestion.

## tests

- The new test exercises the new behaviour: it would fail if the change were reverted or broken, and the description says the author broke the code once and saw it fail. Blocking if the test can't fail for the change it claims to cover.
- Every new test file has a `run_test` line in `scripts/check.sh`, inside the module-check block if it imports `cv2`, `numpy` or `yaml`. Blocking: otherwise the test never runs in CI.
- Logic lives in plain Python modules with no ROS imports; ROS nodes stay thin wrappers. Suggestion, blocking if new logic can only be tested with ROS running.
- The flag-off path is tested too, where the change adds a branch. Suggestion.

## flags

- New behaviour is behind a flag that defaults to `false`. Blocking.
- Mission flags are in the `features:` block under `mission_sequence:` and read with `get_feature_flag`, never a bare dictionary lookup. Blocking.
- Node flags are ROS parameters named `features.<flag>`, under the node's `ros__parameters`, declared with a default of `false`. Never `features:` at the top level of a params file. Blocking.
- The PR's "Flag" section names the flag and the sequence or params files where it appears (still `false`). Suggestion.

## interfaces

- `on_enter`, `update`, `on_exit`, the 20 Hz executor tick, `fake_drop` and the failure policies are unchanged. Blocking.
- Topic names, message types, service names and parameter names that other nodes or launch files use are unchanged, or every user is updated in the same PR. Blocking.

## tuning

- A `[tuning: ...]` waiver gives a reason that the linked issue actually asks for. Blocking if the issue doesn't ask for it.
- A changed tuned value comes with the measurement that justifies it, in `docs/results/`. Suggestion, blocking for gains, altitudes and thresholds used in flight.
- A new parameter's default reproduces today's behaviour. Blocking.

## timing

- Nothing in `update()`, a timer or a subscription callback blocks: no sleeps, synchronous service calls, file or network I/O, or heavy compute on the 20 Hz path. Blocking.
- Ages are computed from capture stamps (`header.stamp`), not receive time or `now()`, and stale inputs are rejected. Blocking.
- New per-frame work in the CV nodes states its cost (latency or FPS) in the description or `docs/results/`. Suggestion.

## safety

- Every flight-guardian BLOCKER is resolved or shown, with code, to be wrong. An unresolved one is blocking.
- Guardian CONCERNs are answered in the PR. Suggestion.
- Anything touching release, descent, RTL, geofence or altitude limits: the 25 ft localisation floor, the 6 ft release rule and the maximum recovery altitude still hold with every flag off and with the new flag on. Blocking.
- Live sequence files keep `fake_drop` as it was. Blocking.

## results

- "How I tested it" shows the commands actually run and their output, not a summary. Suggestion, blocking if a ticked checkbox isn't backed by output.
- Measured numbers are in `docs/results/YYYY-MM-DD-<topic>.md` with the command that produced them. Blocking for a PR that claims an improvement.
- Claims in the description (faster, more accurate, fixed) are backed by those numbers. Blocking if not.
