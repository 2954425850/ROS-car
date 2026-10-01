# Monocular automatic height measurement implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Use the existing wrist RGB camera and arm feedback to measure a support plane and target height before grasping, without new hardware, markers, or manually supplied scene heights.

**Architecture:** Reuse `geom.pixel_ray` and measured joint poses for metric multiview triangulation. Match actual image features, fit the local support plane and a supported top surface, and refuse ambiguous/insufficient data. Capture at stationary observation poses and feed the measured 3D target into the existing grasp controller; keep the manual height mode compatible.

**Tech Stack:** Python 3, NumPy, OpenCV, existing ROS 2/K230 bridges, pytest.

## Global constraints

- No additional hardware or printed/environment markers.
- First establish automatic measurement; accuracy and elapsed time require physical validation.
- Raw calibrated 1280x720 images only; never use upright preview images or bounding-box centres as cross-view feature tracks.
- Existing intrinsics/hand-eye/arm lengths supply scale, not an assumed table/object height.
- Measurement failure prevents grasping; no default-height fallback.
- Offline commands and dry runs must not publish arm commands or change services.
- Implementation runs inline in this session; do not install missing superpowers skills or dispatch agents.
- No physical robot motion in this coding session.

### Task 1: Metric geometry and refusal criteria

**Files:** Create `pi/raspot_ws/src/arm_grasp/arm_grasp/height.py`; test `pi/raspot_ws/src/arm_grasp/test/test_height.py`.

**Interfaces:** `triangulate(centres, directions, config)` returns a point and geometric diagnostics. `measure_cloud(support, target, config)` returns an auditable measurement dict. `measurement_target(report)` validates a successful report and returns its measured target point.

- [x] Test known independent rays, pure rotation/low baseline, wrong correspondences, robust tilted plane recovery, missing top evidence and support outside the target neighbourhood.
- [x] Solve `sum(I-dd.T) P = sum((I-dd.T) C)`; reject insufficient ray angle, negative ranges and excessive ray residual.
- [x] Fit an upward support plane with seeded RANSAC and SVD; require spatial coverage around the target and a coherent elevated target surface. Store local support z, normal height, top z, target point and residuals. Label diagnostics as consistency, not calibrated accuracy.
- [x] Run `python -m pytest raspot_ws/src/arm_grasp/test/test_height.py -q` with the package source on `PYTHONPATH`.

### Task 2: Image measurement and replay

**Files:** Create `pi/raspot_ws/src/arm_grasp/arm_grasp/height_vision.py`, `pi/raspot_ws/src/arm_grasp/tools/measure_height.py`; test `pi/raspot_ws/src/arm_grasp/test/test_height_vision.py`.

**Interfaces:** `measure_session(manifest_path, config)` consumes raw images, measured joints and a reference target box; returns the same report as Task 1 plus track counts, timings and calibration provenance.

- [x] Add image-dimension/frame validation and mutual SIFT ratio matching of true feature tracks observed in at least three views. Separate reference foreground from its local support ring with GrabCut; exclude image borders and wrist/gripper area.
- [x] Triangulate tracks using measured poses and reject reprojection errors. Require plane/top coverage and split-view consistency.
- [x] Save reports and reference diagnostic images; CLI exits nonzero with explicit reasons when measurement fails. Tests use independently rendered textured geometry and untextured scenes, not real-accuracy claims.

### Task 3: Stationary acquisition and grasp integration

**Files:** Create `pi/raspot_ws/src/arm_grasp/arm_grasp/height_capture.py`; modify `pi/raspot_ws/src/arm_grasp/tools/grasp_once.py`, `pi/raspot_ws/src/arm_grasp/arm_grasp/grasp.py`; test `pi/raspot_ws/src/arm_grasp/test/test_height_capture.py` and existing grasp tests.

**Interfaces:** `plan_scan(joints, fields)` returns bounded observation poses. `capture_scan(io, host, box, out_dir)` persists a replayable session, checks fresh/stable feedback while imaging, and returns a session path. `grasp.run(..., initial_point=None)` can seed a measured static target. An auto-height link never replaces the measured top centre with a detector-box-centre ray intersection.

- [x] Preflight every scan waypoint and interpolated joint path, keep the wrist roll consistent with calibration and avoid lowering the open tip below its initial observation height. Preserve current jaw position. Return to the reference pose for target association.
- [x] Add `--auto-height` to capture/measure before grasp and `--height-session` for read-only replay planning; reject manual `--h` conflicts and auto-scan combined with `--dry-run`.
- [x] Reuse driver/service lifecycle with cleanup on interruption and exceptions. Automatically stop on failed measurements or unreachable measured targets; `--yes` cannot override these refusals.
- [x] Test scan limits, capture drift, measured-point seeding, no height fallback and dry-run no-write behaviour. Run existing package tests and CLI selfchecks.

### Task 4: Operational documentation and validation

**Files:** Create `pi/raspot_ws/src/arm_grasp/docs/2026-10-01-auto-height.md`; add optional vision dependencies in `pi/raspot_ws/src/arm_grasp/requirements-height.txt`.

- [x] Document acquisition/replay/grasp commands, report fields, failure meanings, supported visible flat tops, assumed static scene and known calibration/frame constraints.
- [x] Provide physical validation protocol: independently ruler-measured truth, several positions/heights, repeated sessions, error distribution, refusal rate and wall-clock measurement time. Do not report synthetic success as physical precision.
- [x] Summarize tests actually run, remaining physical validation and exact commands to start it.

## Implementation validation

- Implemented metric geometry, image reconstruction/replay, stationary acquisition and automatic-height grasp wiring.
- 60 height/grasp-related tests pass; existing grasp CLI selfcheck passes.
- Final package suite: 171 passed, 9 failed. Git HEAD exported into an independent temporary directory reproduces exactly those same 9 failures (143 passed, 9 failed).
- No physical robot connection or motion, deployment, physical accuracy verification, or Pi timing measurement performed.
- First version supports visible textured flat tops; low-texture scenes explicitly refuse. Fixed bounded acquisition attempts are implemented; failure-driven rescan planning is not implemented.
