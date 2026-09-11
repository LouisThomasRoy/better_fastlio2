# How the GNSS factor works

`addGPSFactor()` in [`src/laserMapping.cpp`](../src/laserMapping.cpp), called
from `saveKeyFramesAndFactor()` where upstream left this:

```cpp
// addGPSFactor();   // TODO: GPS
```

It's based on LIO-SAM's function of the same name, but it wasn't a copy-paste
job. Writing the factor took an afternoon; getting it to actually *improve*
anything took the rest of the day, because three upstream bugs had to be fixed
first and because GNSS fights with a feature the repo already had.

---

## The design

### The frame problem

LIO-SAM assumes GPS arrives already in the odometry frame, because that's what
`navsat_transform_node` hands it. Our input is raw local ENU
(`/gps/odom_enu`), while FAST-LIO's world frame is whatever the first IMU body
frame happened to be: random yaw, random origin. Feed ENU straight in and you
rotate the whole map mid-run, ikd-tree and all.

So the ENU→map yaw and translation get fitted online over the first
`gnss/alignDistance` (30 m) of driving, and every fix is mapped through that fit.
Fixes that arrive before it solves get added retroactively.

The yaw of that fit is a **gauge choice, not an error**. As long as pose 0 can
move, the graph settles onto the GNSS targets and `keyposes_to_tum.py --enu`
takes the same rotation back out on the way to the output file, so it cancels.
That's why `gnss/freeGauge` replaces upstream's rigid `1e-12` pin on pose 0 with
LIO-SAM's prior: roll and pitch tight (gravity observes them), x/y/z/yaw free
(GNSS observes those). Pin the origin instead and a few milliradians of fit error
becomes real distortion, about a metre at 200 m out.

### Lever arm

The antenna sits 1.09 m from the IMU, which is as big as the GPS error itself.
`GPSFactor` constrains the pose translation, so the offset comes off the
measurement through each keyframe's own attitude (`gnss/leverArm`). The alignment
fit avoids needing the transform it's solving for by comparing *predicted antenna
position in map* against *antenna position in ENU*.

### Spacing and weighting

Factors are spaced in **seconds** (`gnss/factorInterval`, 1.0 s), not every N
poses. The node emits keyframes at up to 10 Hz and the receiver error is a slow
correlated drift, so one factor per keyframe would count the same information ten
times and call it ten independent measurements.

Covariance is diagonal in ENU and gets **rotated** into the map frame rather than
assumed isotropic (`lat_err` 0.5–0.7 m against `lon_err` ~0.4 m), with a Cauchy
kernel at `gnss/cauchyWidth` (1.5 m).

LIO-SAM's covariance gate, where GPS only gets used once pose covariance is bad
enough, is left out on purpose. That's an online heuristic for a robot that
trusts its own odometry; this is offline and wants a global bound everywhere.

### Coupling, which is the part that matters

better_fastlio2 is coupled backwards: `saveKeyFramesAndFactor()` pushes the
optimised pose into the ESKF via `kf.change_x`, and `correctPoses()` rebuilds the
ikd-tree from graph poses. Fine for loop closure, which is rare and re-anchors
the map onto geometry the scan matcher can see. Wrong for GNSS, which shoves a
0.5 m global sensor into a 2 cm scan matcher several times a second.

`gnss/looseCoupling` (default true) makes the backend a pure consumer: the ESKF
runs plain FAST-LIO2, the odometry factor becomes a real front-end-to-front-end
relative motion, and the published trajectory is the graph's.

`featuresAndGps`, all four combinations:

| coupling | loop closure | ATE | RPE @ 10 s |
|---|---|---|---|
| tight | on | 0.913 | 0.503 |
| tight | off | 0.979 | 0.477 |
| loose | on | 0.680 | 0.229 |
| **loose** | **off** | **0.261** | **0.181** |
| *(offline two-stage baseline)* | | *0.352* | *0.347* |

Front-end alone is 0.676 / 0.189. So tightly coupled, adding GNSS is **worse than
adding no GNSS at all**.

### GNSS and loop closure won't share a graph

Both on and the optimiser satisfies neither: GNSS residual p95 2.12 m, and 2.81 m
start-to-end on a loop that physically closes. Drop the ICP loop factors and GNSS
alone gives **0.63 m** start-to-end and a GNSS residual whose *worst* value is
0.30 m.

The aggregate hides it. Residual rmse was 1.35 in-pipeline against 1.10
two-stage, which reads as "slightly worse". By decile of time:

```
two-stage      0.07 0.16 0.84 0.38 0.07 0.07 0.10 0.08 0.11 0.07
in-pipeline    0.03 0.06 0.18 0.13 0.11 0.11 0.08 0.16 0.42 2.14
```

Better for 80% of the run, then off a cliff exactly where the loop factors fire.
Likely mechanism: under loose coupling the front-end never gets re-anchored, so
by the time the vehicle comes back round, ICP is registering submaps metres apart
with `setMaxCorrespondenceDistance(200)` and a Scan Context yaw guess, and a
mis-registration enters the graph with a variance taken from the ICP fitness
score.

They're set up as alternatives: loop closure off wherever GNSS exists, on for
`insideGarage` where it doesn't.

---

## Four things that had to be fixed first

Three are upstream bugs with nothing to do with GNSS. Each is commented at its
site with the measurement that caught it.

**1. `addOdomFactor()` and `saveFrame()` mix frames.** Both build their increment
from `cloudKeyPoses6D->back()` (a *graph* pose) and `transformTobeMapped` (an
*ESKF* pose). That's only a relative motion because the feedback keeps the two
identical. Decouple them and the difference becomes the entire accumulated
correction, so every odometry factor drags the next pose back onto the raw
FAST-LIO trajectory at σ = 1 cm / 1 mrad, undoing GNSS as fast as it's applied.
The same bug in `saveFrame()` inflated keyframe counts: 894 at a 1.0 m threshold
where the fixed version gives 445.

**2. `state_point` is not "for visualisation".** The main loop calls
`map_incremental()` right after, and that uses `state_point` to place the scan in
the ikd-tree. Overwrite it with the graph pose and you insert points at a pose
the filter isn't at, so the map walks away from the filter matching against it.
Blew up `twigs` at t = 217 s: 1975 m trajectory against a true 162 m, where plain
FAST-LIO2 at the same spacing runs it at 159.2 m.

**3. A segfault.** `performLoopClosure()` copies `cloudKeyPoses3D/6D` under `mtx`
then indexes `surfCloudKeyFrames` with `copy_cloudKeyPoses6D->size()-1`. Upstream
pushes the pose clouds ~20 lines before the keyframe cloud, so a copy taken in
that window sees one more pose than there are clouds. Killed a run at keyframe
393. All three containers are now appended under one lock. `correctPoses()` also
rewrote them with no lock at all, which matters more here because GNSS makes that
path run about once a second instead of once per loop closure.

**4. `recontructIKdTree()` has to follow the front-end.** It rebuilds the ESKF's
local map from *graph* poses, which under loose coupling splices a corrected map
under an uncorrected filter. Skipping it is worse, since that rebuild is what
keeps the local map bounded. It now rebuilds from front-end keyframe poses.

---

## Keyframe density

`keyframeAddingDistThreshold` went 1.0 m → 0.25 m → **0.0**, so every scan is a
keyframe. Nothing to do with GNSS, and it's the biggest single improvement where
there's no GNSS to lean on. `featuresAndGps`, front-end only, no GNSS, at 0.25 m:
ATE 0.942 → 0.676, RPE 0.371 → **0.189**.

Zero is right rather than merely aggressive. 10 Hz is the ceiling anyway
(`saveKeyFramesAndFactor()` runs once per scan), and going there from 0.25 m only
moved mean ATE 0.298 → 0.275. The real argument is sampling: a distance threshold
produces **no keyframes while the vehicle is parked**, which left 5–20 s holes at
the end of every sequence and a 6.4 s gap mid-`ditches`. At zero the worst gap
across all seven is 170 ms.

Cost on `ditches` (387 s, worst case): 3867 keyframes, 2.33 GB peak RSS, and the
node still keeps real time against `rosbag play`. That last part isn't optional,
since `/save_map` is called 30 s after playback ends and a node that's behind
hands you a truncated trajectory silently.

Three parameters are secretly counted in *keyframes*, not metres, and silently
change meaning if you don't rescale them: `ikdtree/kd_step` (30 → 300),
`loop/historyKeyframeSearchNum` (3 → 30), `loop/loopClosureFrequency`
(5 → 10 Hz).

RAM scales with keyframe count because `surfCloudKeyFrames` keeps one
un-downsampled cloud each. That's the 2.33 GB.

---

## The numbers

Seven sequences against an independent commercial INS solution. "in-graph" is
this fork; "two-stage" is the same front-end with GNSS anchoring done afterwards
in a separate offline GTSAM stage.

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

### Don't quote −44% on its own

That mean is carried by one sequence:

| | all seven | without `insideGarage` |
|---|---|---|
| ATE | 0.492 → 0.275 (**−44.1%**) | 0.310 → 0.252 (**−18.8%**) |
| RPE @ 10 s | 0.261 → 0.220 (−15.7%) | 0.260 → 0.219 (−15.7%) |

Three claims, not one:

1. **RPE improves 10–16% everywhere**, with or without `insideGarage`. The robust
   result.
2. **ATE improves ~19% on the six GNSS sequences**, all of which improved or
   held: `featuresAndGps` −34%, `mixOfNicefeatures` −27%, `niceFeatures` −20%,
   `field` −19%, `twigs` −5%, `ditches` −1%.
3. **`insideGarage` improves 3.8×, and none of it is GNSS** — it hasn't got any.
   That's the keyframe density and the rescaled loop-closure parameters, and the
   old two-stage pipeline would have gained the same.

So the win isn't mainly accuracy. It's that a second optimisation stage
disappeared, RPE improved across the board, and the GNSS residual went flat
instead of lumpy, with accuracy at worst level.

Anything within ±5% is noise. The node isn't deterministic under OpenMP with
real-time playback, and re-running `field` moved the ENU→map fit rms from 0.535
to 0.617 m.

---

## What I'd look at next

- **Odometry factor noise is still upstream's hard-coded `1e-6 / 1e-4`**, never
  rescaled for keyframe spacing, so denser keyframes loosened the chain as a side
  effect rather than a decision.
- **Loop closure is off wherever GNSS exists.** Right on the evidence, but only
  ever diagnosed under loose coupling. A better-conditioned ICP (tighter
  `setMaxCorrespondenceDistance`, or checking against the GNSS prior before
  admitting the factor) might let them coexist. It would matter for a sequence
  with both a long GNSS outage and a real loop; none of these seven is that.
- **`field` and `niceFeatures` sit inside run-to-run noise** of the two-stage
  result. `field` is where to start if anyone pushes this further: open, sparse,
  no loop.
- **`insideGarage` can't be validated.** Its 0.416 m is against a reference that's
  itself unaided indoors.
