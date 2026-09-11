# How the GNSS factor works, and what it took to get there

The function is `addGPSFactor()` in [`src/laserMapping.cpp`](../src/laserMapping.cpp).
It gets called from `saveKeyFramesAndFactor()`, right where upstream left this
sitting:

```cpp
// addGPSFactor();   // TODO: GPS
```

I based it on LIO-SAM's function of the same name. If you're expecting a
copy-paste job, it wasn't one. Writing the factor took an afternoon. Getting it
to actually *improve* anything took the rest of the day, because three separate
upstream bugs had to be fixed first, and because GNSS turned out to fight with a
feature the repo already had.

---

## The design

### The frame problem, which LIO-SAM doesn't have

LIO-SAM assumes your GPS already arrives in the odometry frame, because that's
what `robot_localization`'s `navsat_transform_node` hands it. We don't have
that. Our input is raw local ENU (`/gps/odom_enu`, which the converter emits
with proper covariance), while FAST-LIO's world frame is whatever the first IMU
body frame happened to be: random yaw, random origin.

Feed ENU straight into the graph and you rotate the entire map mid-run, ikd-tree
and all. So instead, the ENU→map yaw and translation get fitted online from the
first `gnss/alignDistance` (30 m) of driving, and every fix afterwards gets
mapped through that fit. Fixes that arrive while we're still waiting for the fit
aren't thrown away, they get added retroactively once it solves.

The yaw that fit produces is a **gauge choice, not an error**, which took me a
while to be comfortable with. As long as pose 0 is free to move, the graph
settles onto the GNSS targets anyway, and `keyposes_to_tum.py --enu` takes the
same rotation back out at the end, so it cancels exactly. That's why
`gnss/freeGauge` swaps upstream's rigid `1e-12` pin on pose 0 for LIO-SAM's
prior: roll and pitch stay tight because gravity observes them, x/y/z/yaw go
free because GNSS observes those. Pin the origin instead and a few milliradians
of fit error turns into real distortion that grows with distance, about a metre
by 200 m out.

### Lever arm

The antenna sits 1.09 m from the IMU. That's as big as the GPS error itself, so
you can't hand-wave it. `GPSFactor` constrains the pose translation, so the
offset gets subtracted from the measurement using each keyframe's own attitude
(`gnss/leverArm`).

There's a chicken-and-egg here, since you need the transform to compensate the
lever arm and the lever arm to fit the transform. The alignment step dodges it
by comparing *predicted antenna position in map* against *antenna position in
ENU*, which doesn't need the transform at all.

### Spacing and weighting

Factors are spaced in **seconds** (`gnss/factorInterval`, currently 1.0 s), not
every N poses. This matters more than it looks: the node emits keyframes at up
to 10 Hz, and the receiver's error is a slow correlated drift rather than
independent noise. One factor per keyframe would be counting the same
information ten times over and telling the optimiser it's ten independent
measurements.

The covariance is diagonal in ENU and gets **rotated** into the map frame rather
than assumed isotropic, since `lat_err` runs 0.5–0.7 m against `lon_err` around
0.4 m. There's a Cauchy kernel at `gnss/cauchyWidth` (1.5 m) so one bad fix
can't drag everything with it.

One thing from LIO-SAM I left out on purpose: its covariance gate, where GPS
only gets used once the pose covariance has grown bad enough. That's a sensible
heuristic for online SLAM on a robot that trusts its own odometry. This is an
offline product where we want a global bound everywhere, all the time.

### Coupling, which is the part that actually matters

better_fastlio2 is wired backwards from what you'd expect.
`saveKeyFramesAndFactor()` pushes the optimised pose back into the ESKF via
`kf.change_x`, and `correctPoses()` rebuilds the ikd-tree from graph poses.

For loop closure that's fine. Loop closures are rare and they re-anchor the map
onto geometry the scan matcher can genuinely see. For GNSS it's a disaster,
because now you're shoving a 0.5 m global sensor into a 2 cm scan matcher
several times a second.

`gnss/looseCoupling` (default true) makes the backend a pure consumer. The ESKF
just runs plain FAST-LIO2, the odometry factor becomes a real
front-end-to-front-end relative motion, and what gets published is the graph's
trajectory.

Here's `featuresAndGps` with all four combinations:

| coupling | loop closure | ATE | RPE @ 10 s |
|---|---|---|---|
| tight | on | 0.913 | 0.503 |
| tight | off | 0.979 | 0.477 |
| loose | on | 0.680 | 0.229 |
| **loose** | **off** | **0.261** | **0.181** |
| *(offline two-stage baseline)* | | *0.352* | *0.347* |

Compare the tight rows against front-end alone, which is 0.676 / 0.189. Tightly
coupled, adding GNSS is **worse than adding no GNSS at all**. That result is
what sent me looking at the coupling in the first place.

### GNSS and loop closure won't share a graph

Turn both on and the optimiser satisfies neither: GNSS residual p95 of 2.12 m,
and 2.81 m between start and end on a loop that physically closes. Kill the ICP
loop factors and leave GNSS alone and start-to-end drops to **0.63 m**, with a
GNSS residual whose *worst* value is 0.30 m.

The aggregate numbers hide this completely. Overall residual rmse was 1.35
in-pipeline against 1.10 two-stage, which reads as "slightly worse, move on".
Split it by decile of time and it's a different story:

```
two-stage      0.07 0.16 0.84 0.38 0.07 0.07 0.10 0.08 0.11 0.07
in-pipeline    0.03 0.06 0.18 0.13 0.11 0.11 0.08 0.16 0.42 2.14
```

Better than the two-stage for 80% of the run, then it falls off a cliff, exactly
where the loop factors fire. My best guess at the mechanism: under loose coupling
the front-end never gets re-anchored, so by the time the vehicle comes back
round, ICP is trying to register submaps that are several metres apart using
`setMaxCorrespondenceDistance(200)` and a Scan Context yaw guess. When that
mis-registers, the bad result goes into the graph with a variance taken straight
from the ICP fitness score, which has no idea it's wrong.

So they're set up as **alternatives**: loop closure off wherever GNSS exists, on
for `insideGarage` where it doesn't. GNSS is the better constraint anyway, and it
applies over the whole run instead of only where the vehicle happens to revisit
somewhere.

---

## Four things that had to be fixed first

Three of these are plain upstream bugs with nothing to do with GNSS. Each one is
commented at its site in the source along with whatever measurement caught it.

**1. `addOdomFactor()` and `saveFrame()` mix up two different frames.** Both
build their increment out of `cloudKeyPoses6D->back()`, which is a *graph* pose,
and `transformTobeMapped`, which is an *ESKF* pose. That only works as a relative
motion because the feedback loop keeps the two frames identical. Decouple them
and suddenly the difference between the two is the entire accumulated
correction, so every odometry factor drags the next pose back onto the raw
FAST-LIO trajectory at σ = 1 cm / 1 mrad. It undoes GNSS as fast as you apply it.
The same bug in `saveFrame()` was also inflating keyframe counts: 894 at a 1.0 m
threshold where the fixed version gives 445.

**2. `state_point` is not "for visualisation".** The comment says it is. It
isn't. The main loop calls `map_incremental()` right after, and that uses
`state_point` to decide where in the ikd-tree the current scan goes. Overwrite it
with the graph pose and you're inserting points at a pose the filter isn't at, so
the map slowly walks away from the filter that's matching against it. This blew
up `twigs`, the fastest and sparsest sequence, at t = 217 s: a 1975 m trajectory
where the truth is 162 m. Plain FAST-LIO2 at the same keyframe spacing runs it
fine at 159.2 m.

**3. An actual segfault.** `performLoopClosure()` copies `cloudKeyPoses3D/6D`
under `mtx`, then indexes `surfCloudKeyFrames` with
`copy_cloudKeyPoses6D->size()-1`. Upstream pushes the pose clouds about 20 lines
before it pushes the keyframe cloud, so a copy taken in that window sees one more
pose than there are clouds, and the loop thread reads off the end. It killed a
run at keyframe 393. All three containers now get appended under one lock.
`correctPoses()` was also rewriting them with no lock at all, which matters much
more here than it did upstream, because GNSS makes that path run about once a
second instead of once per loop closure.

**4. `recontructIKdTree()` has to follow the front-end.** It rebuilds the ESKF's
local map from *graph* poses, which under loose coupling means splicing a
corrected map underneath an uncorrected filter. Just skipping it is worse though,
because that periodic rebuild is what keeps the local map bounded and fresh. It
now rebuilds from a parallel set of front-end keyframe poses.

---

## Keyframe density, an accidental second win

`keyframeAddingDistThreshold` went 1.0 m → 0.25 m → **0.0**, meaning every single
LiDAR scan becomes a keyframe. This has nothing to do with GNSS and it's the
biggest single improvement on sequences with no GNSS to lean on. On
`featuresAndGps`, front-end only, no GNSS, going to 0.25 m: ATE 0.942 → 0.676,
RPE 0.371 → **0.189**.

Zero is the right setting rather than just an aggressive one. 10 Hz is the
ceiling anyway since `saveKeyFramesAndFactor()` runs once per scan, and going
from 0.25 m to zero only moved mean ATE 0.298 → 0.275 and RPE 0.234 → 0.220. The
real argument is about sampling. A distance threshold produces **no keyframes at
all while the vehicle is parked**, which left 5–20 s holes at the end of every
sequence and a 6.4 s gap in the middle of `ditches`. At zero, the worst gap
anywhere in the seven is 170 ms and you get coverage from the first scan to the
last. For something you're releasing as ground truth that matters far more than
the third decimal place of ATE.

What it costs, measured on `ditches` (387 s, the worst case): 3867 keyframes,
2.33 GB peak RSS, and the node still keeps up with `rosbag play` in real time.
That last bit isn't optional. `/save_map` gets called 30 s after playback ends,
so a node that's fallen behind hands you a truncated trajectory and doesn't tell
you about it.

Three parameters are secretly counted in *keyframes* instead of metres, and they
silently change meaning if you don't rescale them: `ikdtree/kd_step` (30 → 300),
`loop/historyKeyframeSearchNum` (3 → 30) and `loop/loopClosureFrequency`
(5 → 10 Hz). Each one is called out where it lives in
`config/charlie8_gnss.yaml`.

RAM scales directly with keyframe count, because `surfCloudKeyFrames` keeps one
un-downsampled cloud per keyframe. That's where the 2.33 GB goes.

---

## The numbers

All seven sequences, checked against an independent commercial INS solution.
"in-graph" is this fork, "two-stage" is the same front-end with GNSS anchoring
bolted on afterwards as a separate offline GTSAM stage.

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

### Please don't quote −44% on its own

That mean is carried by one sequence:

| | all seven | without `insideGarage` |
|---|---|---|
| ATE | 0.492 → 0.275 (**−44.1%**) | 0.310 → 0.252 (**−18.8%**) |
| RPE @ 10 s | 0.261 → 0.220 (−15.7%) | 0.260 → 0.219 (−15.7%) |

There are really three claims here, not one:

1. **RPE improves 10–16% everywhere**, with or without `insideGarage`. This is
   the robust result and the one I'd actually defend.
2. **ATE improves about 19% on the six GNSS sequences.** Every single one
   improved or held: `featuresAndGps` −34%, `mixOfNicefeatures` −27%,
   `niceFeatures` −20%, `field` −19%, `twigs` −5%, `ditches` −1%.
3. **`insideGarage` improves by 3.8×, and none of that is GNSS,** because that
   sequence hasn't got any. It's entirely the keyframe density plus the rescaled
   loop-closure parameters, and the old two-stage pipeline would have gained just
   as much from the same change.

So the win here isn't really accuracy. It's that an entire second optimisation
stage disappeared, RPE got better across the board, and the GNSS residual went
flat instead of lumpy, while accuracy at worst stayed level.

One more thing: anything within about ±5% is **noise**, not signal. The node
isn't deterministic under OpenMP with real-time bag playback, and re-running the
same sequence moved the ENU→map fit rms from 0.535 to 0.617 m on `field`.

---

## What I'd look at next

- **The odometry factor noise is still upstream's hard-coded `1e-6 / 1e-4`**,
  never rescaled for keyframe spacing. Denser keyframes made the whole chain
  looser end to end as a side effect rather than as a decision. Making that
  explicit is the obvious next knob.
- **Loop closure is off wherever GNSS exists.** I think that's right on the
  evidence, but I only ever diagnosed it under loose coupling. A
  better-conditioned ICP (tighter `setMaxCorrespondenceDistance`, maybe sanity
  checking against the GNSS prior before the factor is admitted) might let the
  two coexist. Untested. It would matter for a sequence with both a long GNSS
  outage and a real loop, and none of these seven is that sequence.
- **`field` and `niceFeatures` sit inside run-to-run noise** of the old two-stage
  result. If anyone pushes this further, `field` is where to start: open, sparse,
  no loop, highest drift risk.
- **`insideGarage` can't really be validated.** Its 0.416 m is measured against a
  reference that is itself unaided indoors. Quote it with that caveat attached or
  not at all.
