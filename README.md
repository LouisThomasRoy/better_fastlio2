# better_fastlio2 + GNSS

A fork of [Yixin-F/better_fastlio2](https://github.com/Yixin-F/better_fastlio2) that
adds **GNSS position factors to the iSAM2 backend**, so a single optimisation
produces a globally-bounded, publishable trajectory from one LiDAR and one IMU.

Upstream ships the GTSAM backend and Scan Context loop closure but no GNSS: in
`src/laserMapping.cpp` the call was

```cpp
// addGPSFactor();   // TODO: GPS
```

commented out, with the function never defined, no GPS subscriber, and no
`gps_topic` parameter in any config. This fork writes that function.

It is used to generate ground truth for the **Charlie8** off-road radar dataset
(seven sequences, Ouster OS1-32 + XSENS MTI-200 + NovAtel PwrPak7), and ships
the configuration and launch file for it.

## New here? Start with the guide

**[docs/STUDENT_GUIDE.md](docs/STUDENT_GUIDE.md)** takes you from a bare Ubuntu
machine with Docker on it to a finished ground truth trajectory. It assumes no
ROS experience.

Once you know the moves:

```bash
docker build -t bfl2-gnss:noetic \
    --build-arg UID=$(id -u) --build-arg GID=$(id -g) docker/charlie8
./docker/charlie8/run_container.sh

# inside the container
cd /catkin_ws && catkin build fast_lio_sam -j"$(nproc)"
source devel/setup.bash
roslaunch fast_lio_sam mapping_charlie8.launch seq:=field
```

## What this fork changes

| | |
|---|---|
| `addGPSFactor()` | GNSS position factors in the node's own iSAM2 graph, spaced in **seconds**, lever-arm compensated, covariance rotated from ENU into the map frame, Cauchy kernel |
| Online ENU→map fit | the input is raw local ENU while FAST-LIO's world frame is the first IMU body frame; the rigid transform between them is fitted from the first 30 m of travel and every fix mapped through it, so the graph is never rotated mid-run |
| `gnss/freeGauge` | replaces upstream's rigid `1e-12` pin on pose 0 with LIO-SAM's prior — roll/pitch tight, x/y/z/yaw free |
| `gnss/looseCoupling` | keeps backend corrections out of the ESKF. Tightly coupled, GNSS is *worse than no GNSS at all* |
| 3 upstream bug fixes | a frame mix in `addOdomFactor()`/`saveFrame()`, `state_point` being overwritten (it is not "visualisation only" — `map_incremental()` uses it), and an out-of-bounds read in `performLoopClosure()` that segfaults |
| Charlie8 support | `config/charlie8*.yaml`, `launch/mapping_charlie8.launch`, `tools/`, `docker/charlie8/` |

The design, the ablations, and the evidence behind each choice are in
**[docs/GNSS_FACTOR.md](docs/GNSS_FACTOR.md)**.

See the diff for yourself:

```bash
git remote add upstream https://github.com/Yixin-F/better_fastlio2.git
git fetch upstream
git diff upstream/main -- src/laserMapping.cpp
```

## Results

Seven Charlie8 sequences, against an independent commercial INS solution.
`two-stage` is the same front-end with GNSS anchoring done offline afterwards;
`this fork` puts the GNSS factors inside the graph.

| | two-stage | **this fork** |
|---|---|---|
| mean ATE | 0.492 m | **0.275 m** |
| mean RPE @ 10 s | 0.261 m | **0.220 m** |

Read the per-sequence table and the caveat about that mean in
[docs/GNSS_FACTOR.md](docs/GNSS_FACTOR.md#measured-results) before quoting it —
a large part of the ATE gain comes from one sequence, and that sequence has no
GNSS at all.

## Layout

Everything upstream ships is where upstream put it. What this fork adds:

| Path | What |
|---|---|
| `src/laserMapping.cpp` | modified — `addGPSFactor()` and the fixes it needed |
| `config/charlie8_gnss.yaml` | the config to use. Heavily commented: read it |
| `config/charlie8.yaml` | same front-end, no GNSS section — the baseline |
| `config/charlie8_gnss_1m.yaml`, `_tight.yaml`, `charlie8_dual.yaml`, `charlie8_lidar1.yaml` | ablation variants, so the comparisons can be re-run |
| `launch/mapping_charlie8.launch` | launch file, with `gnss:=` and `loop:=` switches |
| `docker/charlie8/` | the build/run environment (ROS Noetic + GTSAM 4.0.3) |
| `tools/anchor/keyposes_to_tum.py` | `transformations.pcd` → TUM, and the rigid map→ENU change of coordinates |
| `tools/eval/validate.py` | GNSS residual, loop closure error, comparison against a reference |
| `tools/preproc/` | `.db3` → ROS 1 bag conversion (only needed if you have raw data) |
| `docs/` | student guide, GNSS design notes |
| `README_upstream.md` | upstream's original README, unchanged |

## Licence and attribution

GPL-2.0, inherited from upstream — see [LICENSE](LICENSE). This is a modified
version of `better_fastlio2` by Yixin-F, which is itself built on FAST-LIO2
(HKU-MARS), LIO-SAM (TixiaoShan) and Scan Context (Giseop Kim). The GNSS factor
is modelled on LIO-SAM's. Modifications are listed above and are visible as
commits on top of upstream `c188ce1`.
