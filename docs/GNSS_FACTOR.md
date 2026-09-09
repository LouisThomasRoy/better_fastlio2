# The GNSS factor: design, evidence, and what had to be fixed first

`addGPSFactor()` in [`src/laserMapping.cpp`](../src/laserMapping.cpp), called
from `saveKeyFramesAndFactor()` where upstream left
`// addGPSFactor();   // TODO: GPS`.

It is modelled on LIO-SAM's function of the same name. It is **not** a mechanical
port: writing the factor took an afternoon, making it *help* took the rest of the
day and required fixing three upstream defects that have nothing to do with GNSS,
plus resolving one design conflict that does.

---

## The design

### The frame problem, which LIO-SAM does not have

LIO-SAM expects GPS already expressed in the odometry frame — that is what
`robot_localization`'s `navsat_transform_node` hands it. Here the input is raw
local ENU (`/gps/odom_enu`, emitted with covariance by the converter) while
FAST-LIO's world frame is the first IMU body frame: arbitrary yaw, arbitrary
origin. Adding ENU factors directly would rotate the whole graph mid-run and take
the ikd-tree map with it.

So the ENU→map yaw and translation are fitted **online**, from the first
`gnss/alignDistance` (30 m) of travel, and every fix is mapped through the fit.
Correspondences collected while waiting are added retroactively, so nothing is
discarded.

**The yaw of that fit is a gauge choice, not an error.** Provided pose 0 is left
free, the graph settles onto the GNSS targets, and `keyposes_to_tum.py --enu`
removes the same rotation again on output — so it cancels exactly. That is why
`gnss/freeGauge` replaces upstream's rigid `1e-12` pin on pose 0 with LIO-SAM's
prior: roll and pitch tight, because gravity observes them; x, y, z and yaw free,
because GNSS supplies them. With the origin pinned instead, a few milliradians of
fit error becomes real distortion growing with distance from the fit — about 1 m
at 200 m.

### Lever arm

The antenna sits 1.09 m from the IMU, which is as large as the GPS error itself,
so it is not optional. `GPSFactor` constrains the pose *translation*, so the
offset comes off the measurement through each keyframe's own attitude
(`gnss/leverArm`).

The alignment fit avoids the chicken-and-egg by comparing *predicted antenna
position in map* against *antenna position in ENU*, which needs no prior
knowledge of the transform.

### Spacing and weighting

Factors are spaced in **seconds** (`gnss/factorInterval`, 1.0 s), not every-Nth
pose. That distinction matters because the node emits keyframes at up to 10 Hz
while the receiver's error is a smooth correlated bias: one factor per keyframe
would count the same information ten times over.

The measurement covariance is diagonal in ENU and is **rotated** into the map
frame rather than assumed isotropic (`lat_err` ≈ 0.5–0.7 m against `lon_err`
≈ 0.4 m). A Cauchy kernel at `gnss/cauchyWidth` (1.5 m) keeps a bad fix from
dominating.

LIO-SAM's covariance gate — only use GPS once the pose covariance is bad enough —
is deliberately **not** ported. It is an online-SLAM heuristic for a system that
trusts its odometry by default; this is an offline product that wants a global
bound everywhere.

### Coupling: the part that matters

better_fastlio2 is coupled backwards: `saveKeyFramesAndFactor()` pushes the
optimised pose into the ESKF (`kf.change_x`) and `correctPoses()` rebuilds the
ikd-tree from graph poses. That is sound when the only global factor is loop
closure — rare, and it re-anchors the map to geometry the scan matcher can
actually see. It is **not** sound for GNSS, which nudges a 2 cm scan matcher with
a 0.5 m global sensor several times a second.

`gnss/looseCoupling` (default true) makes the backend a pure consumer: the ESKF
runs plain FAST-LIO2, the odometry factor is a true front-end-to-front-end
relative motion, and the published trajectory is the graph's.

Measured on `featuresAndGps`:

| coupling | loop closure | ATE | RPE @ 10 s |
|---|---|---|---|
| tight | on | 0.913 | 0.503 |
| tight | off | 0.979 | 0.477 |
| loose | on | 0.680 | 0.229 |
| **loose** | **off** | **0.261** | **0.181** |
| *(offline two-stage baseline)* | | *0.352* | *0.347* |

Tightly coupled, adding GNSS is **worse than adding no GNSS at all**
(front-end alone: 0.676 / 0.189).

### GNSS and loop closure do not coexist

With both enabled the solution satisfied **neither**: GNSS residual p95 2.12 m,
start-to-end 2.81 m on a loop that physically closes. With better_fastlio2's ICP
loop factors disabled, GNSS alone gave start-to-end **0.63 m** and a GNSS
residual whose *maximum* is 0.30 m.

The aggregate rmse (1.35 in-pipeline vs 1.10 two-stage) says "somewhat worse".
Per decile of time it says something completely different:

```
two-stage      0.07 0.16 0.84 0.38 0.07 0.07 0.10 0.08 0.11 0.07
in-pipeline    0.03 0.06 0.18 0.13 0.11 0.11 0.08 0.16 0.42 2.14
```

Better than the two-stage for 80% of the run, then falling apart exactly where
the loop factors fired. The likely mechanism: under loose coupling the front-end
is never re-anchored, so by the time the vehicle revisits, ICP is registering
submaps several metres apart with `setMaxCorrespondenceDistance(200)` and a Scan
Context yaw guess — and a mis-registration enters the graph with a variance taken
straight from the ICP fitness score.

So they are set as **alternatives**: loop closure off wherever GNSS exists, on
for `insideGarage` where it does not. GNSS is the better global constraint and it
applies everywhere, not only where the vehicle happens to revisit.

---

## Four defects that had to be fixed first

Three are upstream bugs, not consequences of adding GNSS. Each is documented at
its site in the source with the measurement that exposed it.

1. **`addOdomFactor()` and `saveFrame()` mix frames.** Both build their increment
   from `cloudKeyPoses6D->back()` — a *graph* pose — and `transformTobeMapped` —
   an *ESKF* pose. That is only a relative motion because the feedback keeps the
   two frames identical. Decouple them and the difference is the whole
   accumulated correction: every odometry factor then carries it as a spurious
   term and places the next pose back on the raw FAST-LIO trajectory at
   σ = 1 cm / 1 mrad, undoing GNSS as fast as it is applied. In `saveFrame()` the
   same bug inflated keyframe counts — 894 at a 1.0 m threshold where the
   frame-consistent version gives 445.

2. **`state_point` is not "for visualisation"**, whatever the comment says. The
   main loop calls `map_incremental()` immediately after, and that uses it to
   decide where in the ikd-tree the current scan belongs. Overwriting it with the
   graph pose inserts points at a pose the ESKF is not at, so the map drifts away
   from the filter matching against it. This diverged `twigs` — fastest, sparsest
   — at t = 217 s, out to a **1975 m** trajectory against a true 162 m, while
   pure FAST-LIO2 at the same keyframe spacing runs it cleanly at 159.2 m.

3. **A real segfault.** `performLoopClosure()` copies `cloudKeyPoses3D/6D` under
   `mtx` and then indexes `surfCloudKeyFrames` by
   `copy_cloudKeyPoses6D->size()-1`. Upstream pushes the pose clouds ~20 lines
   before the keyframe cloud, so a copy taken in that window sees one more pose
   than there are clouds and the loop thread reads past the end. It killed a run
   at keyframe 393. The three containers are now appended under one lock.
   `correctPoses()` also rewrote them with no lock at all; it now takes one,
   which matters more here because GNSS makes that path run about once a second
   instead of once a loop.

4. **`recontructIKdTree()` must follow the front-end.** It rebuilds the ESKF's
   local map from *graph* poses. Under loose coupling that splices a corrected
   map under an uncorrected filter — but simply skipping it is worse, because the
   periodic rebuild is what bounds and refreshes the local map. It now rebuilds
   from a parallel set of front-end keyframe poses.

---

## Keyframe density

`keyframeAddingDistThreshold` 1.0 m → 0.25 m → **0.0**, i.e. every LiDAR scan is
a keyframe. This is an independent win, and the largest single one on sequences
with no GNSS to lean on. On `featuresAndGps`, front-end only, no GNSS, at 0.25 m:
ATE 0.942 → 0.676, RPE 0.371 → **0.189**.

**Zero is the right setting, not merely a dense one.** 10 Hz is the ceiling —
`saveKeyFramesAndFactor()` runs once per scan — and going there from 0.25 m
improved mean ATE 0.298 → 0.275 and RPE 0.234 → 0.220. But the real argument is
sampling: a distance threshold produces *no keyframes while the vehicle is
stationary*, which left 5–20 s holes at the end of every sequence and a 6.4 s gap
mid-file on `ditches`. At zero the largest gap across all seven is 170 ms and
coverage runs from the first scan to the last. For a released ground truth that
matters more than the third decimal of ATE.

Cost on `ditches` (387 s, worst case): 3867 keyframes, 2.33 GB peak RSS, and the
node still keeps real time against `rosbag play` — which is not optional, because
`/save_map` is called 30 s after playback ends and a node that has fallen behind
ships a silently truncated trajectory.

Three parameters are secretly counted in *keyframes* rather than metres and had
to be rescaled or they silently change meaning: `ikdtree/kd_step` (30 → 300),
`loop/historyKeyframeSearchNum` (3 → 30), and `loop/loopClosureFrequency`
(5 → 10 Hz). All are called out individually in `config/charlie8_gnss.yaml`.

`surfCloudKeyFrames` holds one un-downsampled cloud per keyframe, so RAM scales
directly with this — that is where the 2.33 GB goes.

---

## Measured results

All seven sequences, validated against an independent commercial INS solution.
`in-graph` is this fork; `two-stage` is the same front-end with GNSS anchoring
done afterwards in a separate offline GTSAM stage.

| Sequence | two-stage ATE | **in-graph ATE** | two-stage RPE | **in-graph RPE** |
|---|---|---|---|---|
| ditches | 0.290 | 0.288 | 0.215 | 0.216 |
| featuresAndGps | 0.352 | **0.232** | 0.347 | **0.187** |
| field | 0.404 | **0.329** | 0.420 | **0.331** |
| insideGarage | 1.582 | **0.416** | 0.270 | 0.227 |
| mixOfNicefeatures… | 0.339 | 0.247 | 0.233 | 0.229 |
| niceFeatures | 0.235 | **0.188** | 0.126 | 0.167 |
| twigs | 0.240 | 0.229 | 0.219 | **0.184** |
| **mean** | **0.492** | **0.275** | **0.261** | **0.220** |

### Do not quote −44% without this paragraph

That mean is **dominated by one sequence**:

| | all seven | without `insideGarage` |
|---|---|---|
| ATE | 0.492 → 0.275 (**−44.1%**) | 0.310 → 0.252 (**−18.8%**) |
| RPE @ 10 s | 0.261 → 0.220 (−15.7%) | 0.260 → 0.219 (−15.7%) |

The honest summary is three separate claims, not one:

1. **RPE improves ~10–16% across the board**, with or without `insideGarage`.
   This is the robust result.
2. **ATE improves ~19% on the six GNSS sequences.** Every one improved or held:
   `featuresAndGps` −34%, `mixOfNicefeatures` −27%, `niceFeatures` −20%, `field`
   −19%, `twigs` −5%, `ditches` −1%.
3. **`insideGarage` improves ~3.8×, and none of it is GNSS** — that sequence has
   no fix at all. It is entirely the keyframe density plus the rescaled
   loop-closure parameters, and the same change would have helped the two-stage
   pipeline just as much.

The architectural win is therefore *not* mainly accuracy. It is that the second
optimisation stage is gone, RPE is better everywhere, and the GNSS residual is
flat rather than lumpy — while accuracy is, at worst, unharmed.

Per-sequence differences of ±5% are **not signal**: the node is not
deterministic under OpenMP and real-time bag playback, and repeat runs of the
same sequence moved the ENU→map fit rms from 0.535 to 0.617 m on `field`.

---

## Still open

- **Odometry factor noise is still upstream's fixed `1e-6 / 1e-4` variance**, not
  scaled for keyframe spacing, so denser keyframes made the chain looser end to
  end as a side effect rather than a decision. Making it explicit is the obvious
  next knob.
- **Loop closure is off wherever GNSS exists.** That is the right call on the
  evidence, but it was diagnosed under loose coupling. Whether a
  better-conditioned ICP (tighter `setMaxCorrespondenceDistance`, a sanity check
  against the GNSS prior before the factor is admitted) could make the two
  coexist is untested, and would matter for a sequence with both a long GNSS
  outage and a real loop. None of these seven is that sequence.
- **`field` and `niceFeatures` sit within run-to-run noise** of the two-stage
  result. If this is pushed further, `field` is the one to look at — open,
  sparse, no loop.
- **`insideGarage` cannot be validated.** Its 0.416 m is against a reference that
  is itself unaided indoors. Quote it with that caveat.
