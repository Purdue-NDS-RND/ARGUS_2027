# Flight-guardian calibration cases

Replay cases for testing `.claude/agents/flight-guardian.md`. Each case is a short diff and the finding the guardian should produce. The guardian must never read this file; a hook in its definition blocks it.

To replay a case: start Claude Code in the repo, then ask it to run the flight-guardian on the diff from one case, passing only the diff (paste it; don't point the guardian at this file). Compare its report with the expected result. A good guardian names the right invariant, cites the right line, and gives no BLOCKER on the control case.

The diffs are illustrative and don't need to apply cleanly; line numbers are approximate.

## Case 1: flag read with a bare lookup

```diff
--- a/src/drone_mission_demo/drone_mission_demo/missions/package_drop_mission.py
+++ b/src/drone_mission_demo/drone_mission_demo/missions/package_drop_mission.py
@@ def update(self, context):
-        stamp = detection.header.stamp
+        if self.spec.features.get("use_frame_time_pose", True):
+            stamp = self._frame_time_pose(detection)
+        else:
+            stamp = detection.header.stamp
```

Expected: BLOCKER, F1. The flag defaults to True when it is missing, so the new path runs in every sequence that doesn't list the flag, and the comp-2026 behaviour is gone. It should use `get_feature_flag(self.spec.features, "use_frame_time_pose")`.

## Case 2: sleeping in update()

```diff
@@ def update(self, context):
+        if self._waiting_for_gripper:
+            time.sleep(0.5)  # give the servo time to move
```

Expected: BLOCKER, F4. `update()` runs inside the 20 Hz `_control_loop`; a 0.5 s sleep stalls the tick, so setpoints stop streaming for 10 ticks and OFFBOARD/GUIDED can drop out.

## Case 3: a second setpoint publisher

```diff
--- a/src/drone_target_cv/drone_target_cv/target_cv_node.py
+++ b/src/drone_target_cv/drone_target_cv/target_cv_node.py
@@ def __init__(self):
+        self._nudge_pub = self.create_publisher(
+            PoseStamped, "/mavros/setpoint_position/local", 10)
@@ def _on_detection(self, centre):
+        if self._centring_assist:
+            self._nudge_pub.publish(self._nudge_towards(centre))
```

Expected: BLOCKER, F3. The CV node now publishes position setpoints alongside the executor, so the two compete for the same topic. (Also F1 if `_centring_assist` defaults to true, but the finding to name is F3.)

## Case 4: capture stamp mixed with now()

```diff
@@ def _detection_is_fresh(self, context):
-        age = context.seconds_since(Time.from_msg(detection.header.stamp))
+        self._last_seen = context.now()
+        age = context.seconds_since(self._last_seen)
```

Expected: BLOCKER, F7. Age is measured from when the mission looked, not from when the frame was captured, so it is always about zero and a stale detection passes as fresh. (F6 follows from it, but the invariant broken here is F7.)

## Case 5: feet used as metres

```diff
@@ def __init__(self, spec):
+        # Competition rule: no localisation below 25 ft.
+        self._localisation_floor_m = float(config.get("localisation_floor", 25.0))
```

Expected: BLOCKER, F8. The comment says feet, the name says metres, and 25 is used as metres (82 ft). It should be `25 * FEET_TO_METERS` or a parameter named in feet. (F5 is also touched; the root error is the unit, so F8.)

## Case 6: features at the top of a params file

```diff
--- a/src/drone_target_cv/config/target_cv_live.yaml
+++ b/src/drone_target_cv/config/target_cv_live.yaml
@@
+features:
+  hsv_auto_tune: false
 target_cv:
   ros__parameters:
```

Expected: BLOCKER, F9. A top-level key that isn't a node name breaks the params file, and no node in that launch starts. The flag belongs under `target_cv: ros__parameters:` as `features.hsv_auto_tune: false`.

## Case 7: keep descending when the target is lost

```diff
@@ def _update_descent(self, context):
         if not self._target_visible(context):
-            return self._hold_and_search(context)
+            # Keep going on the last good estimate; the target rarely moves.
+            return self._descend_towards(context, self._last_target_enu)
```

Expected: BLOCKER, F6. With the target lost, the aircraft keeps descending on a stale position instead of holding or climbing to search.

## Case 8: an exception swallowed in update()

```diff
@@ def update(self, context):
-        projection = self._project(context, detection)
+        try:
+            projection = self._project(context, detection)
+        except Exception:
+            return MissionStatus.RUNNING
```

Expected: BLOCKER, F10. A projection error now returns RUNNING, so the executor never applies the failure policy and the aircraft holds whatever setpoint was last set, for as long as the error repeats.

## Case 9 (control): flag-gated logging

```diff
@@ def update(self, context):
+        if get_feature_flag(self.spec.features, "log_centring_error"):
+            context.logger.debug(
+                f"[{self.name}] centring error {projection.horizontal_error_m:.2f} m")
```

Expected: NO FINDINGS (a CONCERN about per-tick logging volume is acceptable; a BLOCKER is a false positive). The flag defaults to False through `get_feature_flag`, the log is debug level, and nothing else changes.
