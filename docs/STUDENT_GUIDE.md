# From a blank laptop to a running SLAM pipeline

This walks you from an Ubuntu machine with **nothing installed — not even
Docker** — to a finished ground truth trajectory produced by a LiDAR-inertial
SLAM pipeline with GNSS in the loop.

No prior ROS experience is assumed. You will type every command yourself;
nothing here is a script that hides the work, because the point of the exercise
is that next time you can do this to a pipeline nobody wrote a guide for.

**Time:** about 15 minutes of typing, and roughly an hour of waiting the first
time (compiling GTSAM and the SLAM code). After that, a run takes as long as the
sequence lasts — 3 to 7 minutes.

---

## What you are going to build

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

The front-end is accurate over short distances but drifts over long ones. GNSS
is the opposite: 0.5 m of noise, but no drift. The pose graph combines them, and
the output is good enough to serve as *ground truth* for evaluating other
algorithms — which is what it is used for.

---

## 0. What you need

| | |
|---|---|
| OS | Ubuntu 20.04 / 22.04 / 24.04, 64-bit (x86_64) |
| RAM | 16 GB recommended. 8 GB works — see the note in step 7 |
| Disk | ~40 GB free (image 4.7 GB, bags 0.7–2.5 GB each, ~350 MB per run) |
| Network | needed for steps 1, 4 and 5 |
| ROS | **do not install it.** It lives in the container |

Check your architecture if you are unsure — an Apple-silicon Mac or an ARM
laptop will not work here:

```bash
uname -m          # must print x86_64
```

---

## 1. Install Docker Engine

Docker is what lets you run Ubuntu 20.04 and ROS Noetic on a laptop that is not
Ubuntu 20.04. This pipeline needs ROS Noetic and GTSAM exactly 4.0.3; installing
those on a modern system directly is painful and breaks other things. In a
container it is reproducible and disposable.

Ubuntu ships a `docker.io` package that is usually out of date. Use Docker's own
repository:

```bash
# 1a. remove anything old that might conflict (fine if it prints nothing)
for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do
    sudo apt-get remove -y $pkg
done

# 1b. add Docker's signing key
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# 1c. add the repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 1d. install
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
                        docker-buildx-plugin docker-compose-plugin
```

> On Linux Mint or another Ubuntu derivative, `$VERSION_CODENAME` is the *Mint*
> codename, which Docker does not publish. Replace it with the Ubuntu one:
> `$(. /etc/os-release && echo "$UBUNTU_CODENAME")`.

**Then let your user run Docker without `sudo`.** Everything after this assumes
you did:

```bash
sudo groupadd -f docker
sudo usermod -aG docker $USER
newgrp docker            # or log out and back in
```

Check it:

```bash
docker run --rm hello-world
```

You should see "Hello from Docker!". If you get
`permission denied while trying to connect to the Docker daemon socket`, the
group change has not taken effect — log out and back in.

---

## 2. Create your folders

```bash
mkdir -p ~/slam/catkin_ws/src ~/slam/datasets ~/slam/output
```

Three folders, three jobs:

| Folder | Holds | Mounted in the container at |
|---|---|---|
| `~/slam/catkin_ws` | your **catkin workspace**: source code you clone, plus the `build/` and `devel/` trees the compiler produces | `/catkin_ws` |
| `~/slam/datasets` | the `.bag` files your instructor gave you | `/datasets` (read-only) |
| `~/slam/output` | trajectories, maps and logs the pipeline writes | `/output` |

A *catkin workspace* is the ROS 1 convention for building code: every package
you want to build goes in `src/`, and `catkin build` generates `build/` (object
files) and `devel/` (the runnable result) next to it. You never create those two
yourself.

The folders live on your host, not inside the container. That means your work
survives when the container exits — and you can edit the source with your normal
editor while the container compiles it.

---

## 3. Put the data in place

Copy the `.bag` files you were given into `~/slam/datasets`:

```bash
cp /path/to/usb/*.bag ~/slam/datasets/
ls -lh ~/slam/datasets/
```

The seven Charlie8 sequences, recorded from a vehicle with a 32-beam Ouster
LiDAR, an XSENS IMU and a NovAtel GNSS receiver:

| Sequence | Duration | Path | GNSS | Notes |
|---|---|---|---|---|
| `featuresAndGps` | 266 s | 433 m | yes | best conditioned — **start here** |
| `field` | 154 s | 215 m | yes | shortest; open and sparse, so drift is visible |
| `niceFeatures` | 263 s | 126 m | yes | returns to its start |
| `ditches` | 387 s | 458 m | yes | longest; driven along side slopes |
| `mixOfNicefeaturesAndOpenSpace` | 238 s | 215 m | yes | |
| `twigs` | 254 s | 162 m | yes | fastest motion |
| `insideGarage` | 227 s | 232 m | **none** | indoors: no GNSS fix at all. Runs differently — see step 8 |

You do not need all seven. `field` is the quickest way to a result;
`featuresAndGps` is the one that looks best.

---

## 4. Clone the pipeline

```bash
cd ~/slam/catkin_ws/src
git clone https://github.com/<GITHUB-USER>/better_fastlio2.git
cd better_fastlio2
```

This is a fork of [better_fastlio2](https://github.com/Yixin-F/better_fastlio2)
with GNSS support added. Look at what is different from the original — it is
three commits:

```bash
git log --oneline -4
```

Note where you cloned it: **inside `catkin_ws/src/`**. catkin only builds what it
finds under `src/`. Cloning it to your Desktop is the single most common way to
get "package not found" in step 8.

---

## 5. Build the container image

From inside the repository you just cloned:

```bash
docker build -t bfl2-gnss:noetic \
    --build-arg UID=$(id -u) --build-arg GID=$(id -g) \
    docker/charlie8
```

What this does, in order:

1. starts from the official ROS Noetic image (Ubuntu 20.04 + ROS + rviz);
2. installs the C++ libraries the pipeline needs — Eigen, PCL, Boost,
   GeographicLib, TBB;
3. **compiles GTSAM 4.0.3 from source.** This is the factor-graph library the
   pose graph is built on. It takes 15–30 minutes and is most of your wait;
4. builds two small ROS message packages the pipeline depends on
   (`livox_ros_driver`, `darknet_ros_msgs`) into `/opt/deps_ws`, so your own
   workspace stays clean;
5. creates a user inside the container with **your** user ID, so files it writes
   into your folders belong to you and not to root.

`-t bfl2-gnss:noetic` is the name you are giving the image. The last argument,
`docker/charlie8`, is the *build context* — the folder holding the `Dockerfile`.

While it compiles, read [`docker/charlie8/Dockerfile`](../docker/charlie8/Dockerfile).
Every non-obvious line has a comment explaining why it is there. That file is the
answer to "how do I install this pipeline", written as code.

When it finishes:

```bash
docker images | grep bfl2
```

You only ever do this once. Rebuilding the *source* later does not mean
rebuilding the image.

---

## 6. Start the container

First, let containers draw windows on your screen — rviz needs this:

```bash
xhost +local:docker
```

Then start the container:

```bash
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

Line by line:

| Flag | Why |
|---|---|
| `-it` | interactive terminal — you get a shell |
| `--rm` | delete the container when you exit. Your files are on the host, so nothing is lost |
| `--name bfl2` | so you can open a second terminal into it |
| `--net=host` | share the host's network. ROS nodes talk to each other over TCP, and this keeps that simple |
| `--shm-size=2g` | ROS moves point clouds through shared memory; the 64 MB default is not enough |
| `-e DISPLAY` + `/tmp/.X11-unix` | the two halves of "let rviz open a window" |
| `-v ~/slam/...:/...` | the three folders from step 2. **This is the passthrough**: `/datasets` inside the container *is* `~/slam/datasets` outside it |
| `:ro` | the dataset mount is read-only, so nothing running inside can damage your data |
| `--device /dev/dri` | hand the GPU to rviz. Drop this flag if it errors; rviz falls back to software rendering |

You are now at a prompt inside the container. Your host is untouched — nothing
you do in here installs anything on your laptop.

> Later, `./docker/charlie8/run_container.sh` runs exactly this command for you,
> and attaches a second shell if the container is already up. Use it once you
> understand what it is doing.

---

## 7. Build the SLAM source

Inside the container:

```bash
cd /catkin_ws
catkin build fast_lio_sam -j"$(nproc)"
```

`fast_lio_sam` is the *package* name — `better_fastlio2` is only the folder the
repository lives in. You will see this mismatch again in step 8.

This compiles for 5–15 minutes. `laserMapping.cpp` alone is a 2500-line
translation unit full of Eigen and PCL templates.

> **8 GB of RAM or less:** use `-j4` instead of `-j"$(nproc)"`. Each parallel
> compiler process can take over a gigabyte, and if the kernel's OOM killer
> stops one you get a confusing "signal 9" error rather than a clear message.

The build is a **Release** build — `CMakeLists.txt` defaults to it. That is not a
detail: at `-O0` this node runs about four times slower than real time, and the
way you find out is not an error but a trajectory that quietly stops a third of
the way through the sequence. If you ever build it elsewhere, pass
`--cmake-args -DCMAKE_BUILD_TYPE=Release` yourself.

When it succeeds, tell your current shell where the result is:

```bash
source devel/setup.bash
```

You have to do this because the shell was started *before* `devel/` existed. New
shells pick it up automatically. Check:

```bash
rospack find fast_lio_sam        # /catkin_ws/src/better_fastlio2
```

---

## 8. Run it

### First, look at the data

```bash
rosbag info /datasets/field.bag
```

A ROS bag is a recording of *topics* — named streams of timestamped messages.
You should see `/ouster0` (the LiDAR, ~1540 messages at 10 Hz), `/imu` (100 Hz)
and `/gps/odom_enu` (20 Hz). Those three are exactly what the pipeline
subscribes to.

### Terminal 1 — start the pipeline

```bash
roslaunch fast_lio_sam mapping_charlie8.launch seq:=field
```

`roslaunch` reads [`launch/mapping_charlie8.launch`](../launch/mapping_charlie8.launch),
which loads [`config/charlie8_gnss.yaml`](../config/charlie8_gnss.yaml) into the
parameter server, starts the mapping node, and starts rviz. It also starts
`roscore` — the name server every ROS node registers with — because none is
running yet.

rviz opens and shows nothing at first. That is correct: nothing is publishing
yet.

> ⚠️ **The node deletes `/output/field/` when it starts.** Not a typo, and not
> optional: `fsmkdir()` calls `fs::remove_all()` on its output directory before
> recreating it. Never point `outdir:=` at a folder holding anything you want to
> keep. This is also why `/datasets` is mounted read-only.

### Terminal 2 — play the bag

Open a second terminal **on your host**, and attach it to the running container:

```bash
docker exec -it bfl2 bash
```

Then, inside:

```bash
rosbag play /datasets/field.bag
```

Now watch terminal 1 and rviz. You should see:

- a white point cloud building up into a recognisable scene;
- a coloured trajectory line growing behind the sensor;
- lines in terminal 1 reporting keyframes and, once about 30 m have been driven,
  a message about the GNSS alignment being solved.

`field` plays for 154 seconds in real time. Let it finish.

Terminal 1 prints a `[ Mapping Time ]` line per scan. The `ave total` field is
seconds of processing per 0.1 s scan — it should sit **well under 0.1**. If it
is larger, the node is falling behind the bag and will end up with a trajectory
shorter than the sequence.

> **Do not add `--clock` to `rosbag play`**, and do not set `use_sim_time`. The
> node's main loop sleeps on wall time; under simulated time those sleeps never
> return once the bag ends, and the save step below will hang forever.

### Terminal 2 — save the result

When playback ends, **wait about 30 seconds**. The pose graph optimiser and the
loop-closure thread run behind the front-end and are still catching up; saving
early gives you a truncated trajectory. Then:

```bash
rosservice call /save_map "{resolution: 0.0, destination: ''}"
```

This is a *service call* — a request/response to the running node, as opposed to
the streaming topics. It writes the results out. Nothing is saved automatically:
if you kill the node without this call, the whole run is lost.

Check what landed:

```bash
ls -lh /output/field/
```

| File | What |
|---|---|
| `transformations.pcd` | **the one that matters** — every keyframe pose with its timestamp |
| `map_from_enu.txt` | the ENU→map transform the GNSS code fitted, and how many factors it added |
| `GlobalMap.pcd` | the accumulated point cloud (~300 MB) |
| `LOG/`, `SCDs/`, `PCDs/` | diagnostics |

Then stop the node in terminal 1 with `Ctrl-C`.

### The exception: insideGarage

That sequence was recorded indoors and has no GNSS fix at all — `/gps` is
present but every field is zero. With no global position sensor, loop closure
becomes the only thing that can correct drift, so the two switch places:

```bash
roslaunch fast_lio_sam mapping_charlie8.launch seq:=insideGarage \
    gnss:=false loop:=true
```

They are alternatives rather than complements: with both enabled the optimiser
satisfies neither constraint. The reasoning, with numbers, is in the comments of
`config/charlie8_gnss.yaml` — worth reading, because it is a good example of a
result that is not obvious in advance.

---

## 9. Turn the result into a trajectory file

`transformations.pcd` is a point cloud file being used as a pose list, which is
awkward to work with. Convert it to **TUM format** — one line per pose,
`timestamp x y z qx qy qz qw`, which every SLAM evaluation tool reads:

```bash
python3 /catkin_ws/src/better_fastlio2/tools/anchor/keyposes_to_tum.py \
    /output/field/transformations.pcd \
    /output/field/ground_truth.tum \
    --enu /output/field/map_from_enu.txt
```

`--enu` additionally rotates the trajectory out of the SLAM frame (whose origin
and heading are wherever the vehicle happened to start) into **local ENU** —
x east, y north, z up. That is a rigid change of coordinates using the transform
the node already fitted, not another optimisation.

It prints a summary and writes `ground_truth.tum` plus a `.json` alongside it
recording how many GNSS factors were used.

---

## 10. Check your answer

```bash
python3 /catkin_ws/src/better_fastlio2/tools/eval/validate.py \
    /output/field/ground_truth.tum
```

Sanity checks first: `field` should report about **1500 poses, 154 s, ~215 m**.
Check all three. A span much shorter than 154 s means the node fell behind and
the trajectory is truncated; a path length that is wildly wrong — 20 m, or
2000 m — means the run diverged.

If your instructor gave you reference trajectories, compare against one:

```bash
python3 /catkin_ws/src/better_fastlio2/tools/eval/validate.py \
    /output/field/ground_truth.tum --compare /datasets/reference/field.tum
```

Expect an RMSE of a few centimetres up to about 0.2 m. It will **not** be zero:
the node is not deterministic — thread scheduling changes which scans arrive
together — so two runs of the same sequence differ slightly. Agreement well
under a metre is a pass.

For the four sequences that return to their starting point (`ditches`,
`featuresAndGps`, `insideGarage`, `niceFeatures`), add `--loop` to see how far
the end of the trajectory lands from the beginning. That number is drift you can
measure without any reference at all.

---

## 11. What you actually ran

Now that it works, read the parts you skipped. In rough order of usefulness:

**`config/charlie8_gnss.yaml`** — every parameter, with a comment explaining why
it has the value it has. Start with `keyframeAddingDistThreshold`,
`gnss/factorInterval` and `gnss/looseCoupling`.

**`launch/mapping_charlie8.launch`** — how a launch file wires up parameters and
nodes, and why `use_sim_time` is deliberately absent.

**`src/laserMapping.cpp`** — search for `addGPSFactor`. Around 340 lines starting
at the `GNSS` banner comment are this fork's; the rest is upstream. See
[GNSS_FACTOR.md](GNSS_FACTOR.md) for the design.

### Things to try

1. **Turn GNSS off** and see how much it was doing:
   `roslaunch fast_lio_sam mapping_charlie8.launch seq:=field gnss:=false`
   then compare the two trajectories with `validate.py --compare`. On `field`
   the difference is metres.
2. **Break the de-skewing.** Set `time_unit: 2` in the config and re-run. Nothing
   errors; the trajectory just gets worse. Working out *why* from the config
   comments is the exercise.
3. **Run the same sequence twice** without changing anything and compare. The
   difference between the two runs is your repeatability, and it is the honest
   lower bound on any accuracy you claim.
4. **Watch the topics live** while a run is going, from a third terminal:
   `rostopic hz /ouster0`, `rostopic echo -n1 /gps/odom_enu`,
   `rosnode info /laserMapping`.

---

## Troubleshooting

**`permission denied ... /var/run/docker.sock`**
The group change from step 1 has not taken effect. Log out and back in.

**rviz does not open / `cannot open display`**
Run `xhost +local:docker` on the *host* before starting the container. If you
are on Wayland (Ubuntu 22.04+ default), this still works through XWayland.

**`libGL error: failed to load driver: iris` (or `i915`), or rviz is black**
The container's graphics driver does not match your GPU. Force software
rendering — slower, but always works. Add to `docker run`:
`-e LIBGL_ALWAYS_SOFTWARE=1`, and drop `--device /dev/dri`.

**`[rospack] Error: package 'fast_lio_sam' not found`**
Either you did not `source /catkin_ws/devel/setup.bash` in this shell, or you
cloned the repository somewhere other than `~/slam/catkin_ws/src/`.

**`catkin build` fails with `c++: fatal error: Killed signal terminated`**
Out of memory. Rebuild with `-j4`, or `-j2`.

**`RLException: Unable to contact my own server`**
ROS cannot resolve its own hostname. You started the container without
`--net=host`; use the command in step 6 as written.

**The bag finishes but `/save_map` hangs**
You played the bag with `--clock`, or something set `use_sim_time`. Kill it,
start again without.

**`transformations.pcd` missing after `/save_map`**
The node was killed before the call, or the call errored. Check terminal 1.
Remember the node wipes its output directory at startup, so a *previous* run's
files are gone by then too.

**The trajectory is much shorter than the sequence**
The node fell behind `rosbag play` and `/save_map` captured only what it had
finished. Check `ave total` in terminal 1 (see step 8) and confirm the build is
a Release build:
`grep CMAKE_BUILD_TYPE /catkin_ws/build/fast_lio_sam/CMakeCache.txt`.
Close other heavy applications and re-run — the node has to keep real time.

**The trajectory is ten times too long, or wanders off**
Usually the wrong sequence config, or a bag that is not one of the seven. Check
`rosbag info` shows `/ouster0` and `/imu` with the message counts you expect.

**Disk fills up**
`GlobalMap.pcd` is ~300 MB per run and `/output/<seq>/` is wiped on the next run
of the *same* sequence, but not of a different one. Delete what you do not need.

---

## Cheat sheet

```bash
# on the host
xhost +local:docker
./docker/charlie8/run_container.sh          # start, or attach another shell

# in the container, once
cd /catkin_ws && catkin build fast_lio_sam -j"$(nproc)" && source devel/setup.bash

# per sequence: terminal 1
roslaunch fast_lio_sam mapping_charlie8.launch seq:=SEQ
#           insideGarage only:  ... seq:=insideGarage gnss:=false loop:=true

# terminal 2
rosbag play /datasets/SEQ.bag
sleep 30
rosservice call /save_map "{resolution: 0.0, destination: ''}"

# terminal 2, after
python3 /catkin_ws/src/better_fastlio2/tools/anchor/keyposes_to_tum.py \
    /output/SEQ/transformations.pcd /output/SEQ/ground_truth.tum \
    --enu /output/SEQ/map_from_enu.txt
python3 /catkin_ws/src/better_fastlio2/tools/eval/validate.py \
    /output/SEQ/ground_truth.tum
```
