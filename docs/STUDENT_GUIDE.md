# Getting the pipeline running from scratch

From a bare Ubuntu machine to a finished ground truth trajectory. No ROS
experience needed. Everything except Docker itself lives in the container.
Should take around 2h to install for the first time.

---

## What you're building

```
  /ouster0      32-beam LiDAR, 10 Hz  ──┐
  /imu          100 Hz                  ├──►  FAST-LIO2 front-end
                                        │     (error-state Kalman filter +
                                        │      ikd-tree map)
                                        │            │  keyframe poses
                                        │            ▼
  /gps/odom_enu 20 Hz, sigma ~0.5 m  ───┴──►  iSAM2 pose graph
                                              + addGPSFactor()   ◄── the part
                                                     │               this fork
                                                     ▼               adds
                                            ground_truth.tum
                                            (a trajectory in local ENU)
```

The front-end is accurate locally and drifts over distance. GNSS is the other way
round, half a metre of noise, no drift. Together they're accurate enough to use
as *ground truth* for testing other algorithms.

---

## 1. Check Docker

```bash
docker run --rm hello-world
```

A greeting means you're set. `permission denied ... docker.sock` means you're not
in the `docker` group yet: `sudo usermod -aG docker $USER`, then log out and back
in.

If there's no `docker` command at all, install it first:
<https://docs.docker.com/engine/install/ubuntu/>

---

## 2. Make your folders

```bash
mkdir -p ~/slam/catkin_ws/src ~/slam/datasets ~/slam/output
```

| Folder | What goes in it | In the container |
|---|---|---|
| `~/slam/catkin_ws` | the code you clone, plus the `build/` and `devel/` folders catkin makes | `/catkin_ws` |
| `~/slam/datasets` | the `.bag` files you were given | `/datasets` (read-only) |
| `~/slam/output` | trajectories, maps and logs | `/output` |

A *catkin workspace* is the ROS 1 convention for where code gets built: anything
you want compiled goes in `src/`, and `catkin build` creates `build/` and
`devel/` beside it. All three folders live on your laptop, so your work survives
the container exiting and you can edit the code in your normal editor.

---

## 3. Copy the data in

```bash
cp /path/to/usb/*.bag ~/slam/datasets/
```

Recorded off a vehicle with a 32-beam Ouster LiDAR, an XSENS IMU and a NovAtel
GNSS receiver.

| Sequence | Duration | Path | GNSS | Notes |
|---|---|---|---|---|
| `field` | 154 s | 215 m | yes | **start here.** Shortest, and open enough to see the drift |
| `featuresAndGps` | 266 s | 433 m | yes | cleanest data, best rviz |
| `niceFeatures` | 263 s | 126 m | yes | returns to its start |
| `ditches` | 387 s | 458 m | yes | longest, driven along side slopes |
| `mixOfNicefeaturesAndOpenSpace` | 238 s | 215 m | yes | |
| `twigs` | 254 s | 162 m | yes | fastest driving |
| `insideGarage` | 227 s | 232 m | **none** | indoors, no GNSS fix. Runs differently, see step 8 |

---

## 4. Clone the pipeline

```bash
cd ~/slam/catkin_ws/src
git clone https://github.com/LouisThomasRoy/better_fastlio2.git
cd better_fastlio2
git log --oneline -8      # the fork's commits, on top of upstream
```

A fork of [better_fastlio2](https://github.com/Yixin-F/better_fastlio2) with the
GNSS work added.

⚠️ Clone it **inside `catkin_ws/src/`**. catkin only looks for packages under
`src/`, so anywhere else builds nothing and you get "package not found" in
step 8.

---

## 5. Build the container image

From inside the repo:

```bash
docker build -t bfl2-gnss:noetic \
    --build-arg UID=$(id -u) --build-arg GID=$(id -g) \
    docker/charlie8
```

It pulls the ROS Noetic image, apt-installs Eigen/PCL/Boost/GeographicLib/TBB,
**compiles GTSAM 4.0.3 from source** (15–30 min, this is the wait), builds two
ROS message packages the pipeline needs into `/opt/deps_ws`, and makes a user
with your UID so the files it writes belong to you.

Read [`docker/charlie8/Dockerfile`](../docker/charlie8/Dockerfile) while you
wait. It's the real answer to "how do I install this", written as code.

You do this once. Recompiling the SLAM code later doesn't mean rebuilding the
image.

---

## 6. Start the container

```bash
xhost +local:docker          # let the container draw rviz on your screen

docker run -it --rm \
    --name bfl2 \
    --net=host \
    --shm-size=2g \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/slam/catkin_ws:/catkin_ws \
    -v ~/slam/datasets:/datasets:ro \
    -v ~/slam/output:/output \
    --device /dev/dri:/dev/dri \
    bfl2-gnss:noetic
```

| Flag | What it's for |
|---|---|
| `-it` / `--rm` | interactive shell / delete the container on exit. Your files are on the host, so nothing is lost |
| `--name bfl2` | lets you open a second terminal into it |
| `--net=host` | ROS nodes talk over TCP; this saves a lot of hassle |
| `--shm-size=2g` | ROS moves point clouds through shared memory, 64 MB isn't enough |
| `-e DISPLAY` + `/tmp/.X11-unix` | the two halves of "rviz can open a window" |
| `-v ~/slam/...:/...` | the folders from step 2. **This is the passthrough:** `/datasets` in the container *is* `~/slam/datasets` outside |
| `:ro` | dataset mount read-only, so nothing inside can wreck your data. See step 8 |
| `--device /dev/dri` | gives rviz your GPU. Drop it if it errors and rviz falls back to software rendering |

Later on, `./docker/charlie8/run_container.sh` does all of that, and opens
another shell if the container is already up.

---

## 7. Compile the SLAM code

```bash
cd /catkin_ws
catkin build fast_lio_sam -j"$(nproc)"
source devel/setup.bash
rospack find fast_lio_sam        # /catkin_ws/src/better_fastlio2
```

The *package* is `fast_lio_sam`; `better_fastlio2` is just the folder. You'll hit
that again in step 8.

3 to 15 minutes depending on cores. **With 8 GB of RAM or less use `-j4`** — each
compiler process can take over a gigabyte, and an OOM kill shows up as a cryptic
"signal 9".

You need the `source` because this shell started before `devel/` existed. New
shells pick it up automatically.

⚠️ **Don't switch off the Release build.** `CMakeLists.txt` defaults to it. At
`-O0` the node runs four times slower than real time and doesn't warn you about
it — you just get a trajectory that stops a third of the way through and looks
fine until you check its length. I lost a run to this.

---

## 8. Run it

### Look at the data

```bash
rosbag info /datasets/field.bag
```

A bag records *topics*, named streams of timestamped messages. The pipeline uses
three of them: `/ouster0` (1540 messages, 10 Hz), `/imu` (15401, 100 Hz) and
`/gps/odom_enu` (3080, 20 Hz).

### Terminal 1: start the pipeline

```bash
roslaunch fast_lio_sam mapping_charlie8.launch seq:=field
```

That loads `config/charlie8_gnss.yaml` into the parameter server, starts the
mapping node and rviz, and starts `roscore` for you. rviz is empty until you play
the bag.

⚠️ **The node deletes `/output/field/` on startup**, every time. `fsmkdir()`
calls `fs::remove_all()` on its output folder before recreating it. Never point
`outdir:=` at anything you care about. It's also why `/datasets` is read-only.

### Terminal 2: play the bag

Second terminal on your laptop, into the container that's already running:

```bash
docker exec -it bfl2 bash
rosbag play /datasets/field.bag
```

rviz should fill in with a white point cloud and a coloured trajectory line
behind the sensor. Terminal 1 reports keyframes, and after ~30 m of driving, the
GNSS alignment being solved.

Watch the `[ Mapping Time ]` lines: `ave total` is seconds of processing per
0.1 s scan, so it has to stay **well under 0.1** (mine is ~0.015). Above 0.1 the
node can't keep up and your trajectory will come out short.

⚠️ **No `--clock`, no `use_sim_time`.** The main loop sleeps on wall time. Under
sim time those sleeps never return once the bag ends and the save below hangs
forever.

### Terminal 2: save it

`field` plays for 154 s. When it ends, **wait 30 seconds** — the pose graph and
loop-closure threads are still catching up, and saving early truncates your
trajectory.

```bash
rosservice call /save_map "{resolution: 0.0, destination: ''}"
ls -lh /output/field/
```

Nothing saves automatically. Ctrl-C the node without calling this and the run is
gone.

| File | What it is |
|---|---|
| `transformations.pcd` | **the one that matters.** Every keyframe pose with its timestamp |
| `map_from_enu.txt` | the ENU→map transform the GNSS code fitted, and how many factors it used |
| `GlobalMap.pcd` | the accumulated point cloud, ~300 MB |
| `LOG/`, `SCDs/`, `PCDs/` | diagnostics |

### insideGarage is different

No GNSS fix anywhere in it — `/gps` is there but every field is zero. Loop
closure becomes the only thing that can correct drift, so the two swap:

```bash
roslaunch fast_lio_sam mapping_charlie8.launch seq:=insideGarage \
    gnss:=false loop:=true
```

You can't run both. With GNSS and loop closure enabled together the optimiser
satisfies neither and the result is worse than either alone. The numbers are in
the comments in `config/charlie8_gnss.yaml`.

---

## 9. Make a trajectory file

`transformations.pcd` is a point cloud file holding a list of poses, which
nothing else reads. Convert it to **TUM format**, one line per pose,
`timestamp x y z qx qy qz qw`:

```bash
python3 /catkin_ws/src/better_fastlio2/tools/anchor/keyposes_to_tum.py \
    /output/field/transformations.pcd \
    /output/field/ground_truth.tum \
    --enu /output/field/map_from_enu.txt
```

`--enu` also moves it out of the SLAM frame (origin and heading are wherever the
vehicle started) into local ENU: x east, y north, z up. That's a rigid rotation
and shift using the transform the node already fitted, not another optimisation.

---

## 10. Check it

```bash
python3 /catkin_ws/src/better_fastlio2/tools/eval/validate.py \
    /output/field/ground_truth.tum
```

`field` should be about **1500 poses, 154 s, 215 m**. Check all three. A short
span means the node fell behind and the trajectory is truncated; a path length of
20 m or 2000 m means it diverged.

With a reference trajectory:

```bash
python3 /catkin_ws/src/better_fastlio2/tools/eval/validate.py \
    /output/field/ground_truth.tum --compare /datasets/reference/field.tum
```

A clean `field` run lands around 0.05 m RMSE, worst pose ~0.13 m. It won't be
zero — the node isn't deterministic, since thread scheduling changes which scans
get batched together. Under a metre is fine.

For the four sequences that return to their start (`ditches`, `featuresAndGps`,
`insideGarage`, `niceFeatures`), `--loop` reports how far the end landed from the
beginning. That's drift measured without any reference at all.

---

## 11. Read what you ran

- **`config/charlie8_gnss.yaml`** — every parameter has a comment saying why it's
  set that way. Start with `keyframeAddingDistThreshold`, `gnss/factorInterval`
  and `gnss/looseCoupling`.
- **`launch/mapping_charlie8.launch`** — how a launch file wires up parameters
  and nodes, and why `use_sim_time` is missing.
- **`src/laserMapping.cpp`** — search `addGPSFactor`. The ~340 lines under the
  `GNSS` banner are this fork's, the rest is upstream.
  [GNSS_FACTOR.md](GNSS_FACTOR.md) has the reasoning.

