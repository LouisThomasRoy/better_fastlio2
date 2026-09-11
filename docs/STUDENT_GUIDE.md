# Getting the pipeline running from scratch

If you've never touched ROS before, start here. This takes you from a laptop
with nothing installed on it, not even Docker, to a finished ground truth
trajectory out of a LiDAR-inertial SLAM pipeline with GNSS wired into it.

You'll type every command yourself. I could have wrapped all of this in one
script, but then you'd learn nothing, and the next pipeline you have to install
won't come with a guide.

Budget about two hours for your first run. Only about fifteen minutes of that is
you doing anything, the rest is waiting for stuff to compile, so line up
something else to do. Once it's built, a run takes as long as the sequence
lasts, so 3 to 7 minutes.

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

The front-end is very good locally and drifts over long distances. GNSS is the
other way round: half a metre of noise on every fix, but it never drifts. Put
both in a pose graph and you get something accurate enough to use as *ground
truth* for testing other algorithms, which is the whole reason this exists.

---

## 0. What you need

| | |
|---|---|
| OS | Ubuntu 20.04 / 22.04 / 24.04, 64-bit (x86_64) |
| RAM | 16 GB is comfortable. 8 GB works, see the note in step 7 |
| Disk | ~40 GB free (image 4.7 GB, bags 0.7–2.5 GB each, ~350 MB per run) |
| Network | for steps 1, 4 and 5 |
| ROS | **don't install it.** It lives in the container |

If you're not sure what CPU you have, check. An M1/M2 Mac or an ARM laptop won't
work here:

```bash
uname -m          # has to say x86_64
```

---

## 1. Install Docker

Docker is how you run Ubuntu 20.04 and ROS Noetic on a laptop that isn't Ubuntu
20.04. This pipeline wants ROS Noetic and GTSAM 4.0.3 specifically. Installing
those straight onto a modern system is miserable and tends to break other
things you have. In a container you can throw it away and start again.

Don't use the `docker.io` package from Ubuntu, it's usually ancient. Use
Docker's own repo:

```bash
# 1a. get rid of anything old that would conflict (fine if it finds nothing)
for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do
    sudo apt-get remove -y $pkg
done

# 1b. Docker's signing key
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# 1c. the repo
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 1d. install
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
                        docker-buildx-plugin docker-compose-plugin
```

> On Mint or another Ubuntu spin, `$VERSION_CODENAME` gives you the Mint
> codename, which Docker doesn't publish packages for. Swap in the Ubuntu one:
> `$(. /etc/os-release && echo "$UBUNTU_CODENAME")`.

Now let yourself run Docker without `sudo`. Everything below assumes you've done
this:

```bash
sudo groupadd -f docker
sudo usermod -aG docker $USER
newgrp docker            # or just log out and back in
```

Test it:

```bash
docker run --rm hello-world
```

If that prints "Hello from Docker!" you're good. If you get
`permission denied while trying to connect to the Docker daemon socket`, the
group change hasn't kicked in yet, so log out and back in.

---

## 2. Make your folders

```bash
mkdir -p ~/slam/catkin_ws/src ~/slam/datasets ~/slam/output
```

Three folders, one job each:

| Folder | What goes in it | Shows up in the container as |
|---|---|---|
| `~/slam/catkin_ws` | your **catkin workspace**: the code you clone, plus the `build/` and `devel/` folders the compiler makes | `/catkin_ws` |
| `~/slam/datasets` | the `.bag` files you were given | `/datasets` (read-only) |
| `~/slam/output` | trajectories, maps and logs the pipeline writes | `/output` |

If you haven't used ROS 1 before: a *catkin workspace* is just the convention
for where code gets built. Anything you want compiled goes in `src/`, and
`catkin build` creates `build/` (object files) and `devel/` (the stuff you
actually run) beside it. Don't make those two yourself, catkin does it.

All three folders live on your laptop, not inside the container. So your work
survives when the container exits, and you can edit the code in VS Code or
whatever you normally use while the container compiles it.

---

## 3. Copy the data in

```bash
cp /path/to/usb/*.bag ~/slam/datasets/
ls -lh ~/slam/datasets/
```

These are the Charlie8 sequences, recorded off a vehicle carrying a 32-beam
Ouster LiDAR, an XSENS IMU and a NovAtel GNSS receiver:

| Sequence | Duration | Path | GNSS | Notes |
|---|---|---|---|---|
| `featuresAndGps` | 266 s | 433 m | yes | cleanest data, **start here** |
| `field` | 154 s | 215 m | yes | shortest run. Open and sparse, so you can see the drift |
| `niceFeatures` | 263 s | 126 m | yes | comes back to where it started |
| `ditches` | 387 s | 458 m | yes | longest, and driven along side slopes |
| `mixOfNicefeaturesAndOpenSpace` | 238 s | 215 m | yes | |
| `twigs` | 254 s | 162 m | yes | fastest driving |
| `insideGarage` | 227 s | 232 m | **none** | indoors, so no GNSS fix at all. Runs differently, see step 8 |

You don't need all of them. `field` gets you to a result fastest,
`featuresAndGps` is the one that looks impressive in rviz.

---

## 4. Clone the pipeline

```bash
cd ~/slam/catkin_ws/src
git clone https://github.com/LouisThomasRoy/better_fastlio2.git
cd better_fastlio2
```

It's a fork of [better_fastlio2](https://github.com/Yixin-F/better_fastlio2)
with the GNSS work added on top. If you want to see exactly what changed, it's
only a few commits:

```bash
git log --oneline -8
```

Pay attention to *where* you cloned it: **inside `catkin_ws/src/`**. catkin only
looks for packages under `src/`, so if you clone this to your Desktop or your
home folder it will compile nothing and you'll get "package not found" in
step 8 wondering why.

---

## 5. Build the Docker image

Run this from inside the repo you just cloned:

```bash
docker build -t bfl2-gnss:noetic \
    --build-arg UID=$(id -u) --build-arg GID=$(id -g) \
    docker/charlie8
```

Here's what it's doing while you wait:

1. pulls the official ROS Noetic image (Ubuntu 20.04 + ROS + rviz);
2. apt-installs the C++ libraries the pipeline needs: Eigen, PCL, Boost,
   GeographicLib, TBB;
3. **compiles GTSAM 4.0.3 from source.** That's the factor graph library the
   pose graph is built on, and it's 15 to 30 minutes of your life. This is the
   bulk of the wait;
4. builds two small ROS message packages the pipeline depends on
   (`livox_ros_driver` and `darknet_ros_msgs`) into `/opt/deps_ws`, so they stay
   out of your workspace;
5. makes a user inside the container with the same UID as you, so everything it
   writes into your folders belongs to you instead of root.

`-t bfl2-gnss:noetic` is just the name you're giving the image. The last
argument, `docker/charlie8`, is the *build context*, meaning the folder the
`Dockerfile` sits in.

Since you're stuck waiting anyway, open
[`docker/charlie8/Dockerfile`](../docker/charlie8/Dockerfile) and read it. I
commented every line that isn't obvious. That file is basically the answer to
"how do I install this pipeline", except written as code so it can't go stale.

When it's done:

```bash
docker images | grep bfl2
```

You do this once, ever. Recompiling the SLAM code later does not mean rebuilding
the image.

---

## 6. Start the container

rviz needs permission to draw on your screen, so first:

```bash
xhost +local:docker
```

Then:

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

That's a lot of flags, so:

| Flag | What it's for |
|---|---|
| `-it` | interactive terminal, i.e. you get a shell |
| `--rm` | throw the container away when you exit. All your files are on the host, so you lose nothing |
| `--name bfl2` | lets you open a second terminal into it later |
| `--net=host` | use the laptop's network. ROS nodes talk over TCP and this saves a lot of hassle |
| `--shm-size=2g` | ROS passes point clouds through shared memory and Docker's 64 MB default isn't enough |
| `-e DISPLAY` + `/tmp/.X11-unix` | the two halves of "rviz is allowed to open a window" |
| `-v ~/slam/...:/...` | the three folders from step 2. **This is the passthrough:** `/datasets` inside the container *is* `~/slam/datasets` outside it |
| `:ro` | the dataset mount is read-only so nothing in the container can wreck your data. There's a reason for this, see step 8 |
| `--device /dev/dri` | gives rviz your GPU. If it complains, drop this flag and rviz will fall back to software rendering |

You should now be at a prompt inside the container. Nothing you do in here
touches your actual laptop.

> Once you've done this a few times, `./docker/charlie8/run_container.sh` runs
> the same thing for you, and if the container is already up it just opens
> another shell in it. Learn the long version first though.

---

## 7. Compile the SLAM code

Inside the container:

```bash
cd /catkin_ws
catkin build fast_lio_sam -j"$(nproc)"
```

Note the name. The *package* is called `fast_lio_sam`, `better_fastlio2` is only
the folder the repo lives in. That trips people up, and you'll hit it again in
step 8.

This takes 3 to 15 minutes depending on your core count. `laserMapping.cpp` on
its own is 2500 lines of Eigen and PCL templates, so it's slow to compile.

> **If you have 8 GB of RAM or less,** use `-j4` instead of `-j"$(nproc)"`. Each
> compiler process can eat over a gigabyte, and when the OOM killer takes one
> out you get a cryptic "signal 9" instead of anything useful.

This is a **Release** build, which `CMakeLists.txt` now sets by default. Don't
undo that. At `-O0` the node runs roughly four times slower than real time, and
you don't get an error about it. What you get is a trajectory that quietly stops
a third of the way through the sequence and looks fine until you check its
length. I burned a run on exactly that. If you ever build this somewhere else,
pass `--cmake-args -DCMAKE_BUILD_TYPE=Release` yourself.

Once it's built, point your current shell at the result:

```bash
source devel/setup.bash
```

You need this because you opened that shell before `devel/` existed. Any new
shell picks it up on its own. Check it worked:

```bash
rospack find fast_lio_sam        # should print /catkin_ws/src/better_fastlio2
```

---

## 8. Run it

### Look at the data first

```bash
rosbag info /datasets/field.bag
```

A bag is a recording of *topics*, which are named streams of timestamped
messages. You should see `/ouster0` (the LiDAR, 1540 messages at 10 Hz), `/imu`
(15401 of them, 100 Hz) and `/gps/odom_enu` (3080, 20 Hz). Those three are what
the pipeline actually subscribes to. The others are along for the ride.

### Terminal 1: start the pipeline

```bash
roslaunch fast_lio_sam mapping_charlie8.launch seq:=field
```

`roslaunch` reads [`launch/mapping_charlie8.launch`](../launch/mapping_charlie8.launch),
dumps [`config/charlie8_gnss.yaml`](../config/charlie8_gnss.yaml) into the
parameter server, starts the mapping node and opens rviz. It also starts
`roscore` for you, which is the name server every ROS node has to register with.

rviz will be empty at first. That's fine, nothing is publishing yet.

> ⚠️ **Heads up: the node deletes `/output/field/` on startup.** That's not a
> bug and it's not avoidable. `fsmkdir()` calls `fs::remove_all()` on its output
> folder before recreating it, every single time the node starts. Never point
> `outdir:=` at anything you care about. It's also why `/datasets` is mounted
> read-only, because a typo there would take your bags with it.

### Terminal 2: play the bag

Open a second terminal **on your laptop** and jump into the container that's
already running:

```bash
docker exec -it bfl2 bash
```

Then:

```bash
rosbag play /datasets/field.bag
```

Now go watch terminal 1 and rviz. You should get:

- a white point cloud filling in something that looks like a real place;
- a coloured line trailing behind the sensor, which is the trajectory;
- keyframe messages scrolling past in terminal 1, and after about 30 m of
  driving, a line about the GNSS alignment being solved.

`field` plays for 154 seconds at real speed. Let it run.

While it does, look at the `[ Mapping Time ]` lines in terminal 1. The
`ave total` number is how many seconds of processing each 0.1 s scan costs, so
it needs to stay **well under 0.1**. Mine sits around 0.015. If yours is bigger
than 0.1, the node can't keep up with the bag and you're going to end up with a
trajectory shorter than the sequence.

> **Don't add `--clock` to `rosbag play`** and don't set `use_sim_time`. The
> node's main loop sleeps on wall time. Under simulated time those sleeps never
> wake up once the bag ends, so the save below will just hang there forever.

### Terminal 2: save it

When the bag finishes, **wait about 30 seconds** before doing anything. The pose
graph and the loop-closure thread run behind the front-end and they're still
catching up. Save too early and you cut the end off your own trajectory. Then:

```bash
rosservice call /save_map "{resolution: 0.0, destination: ''}"
```

That's a *service call*, which is a one-off request to a running node, as
opposed to the topics that stream continuously. Nothing gets saved on its own,
so if you Ctrl-C the node without calling this, the entire run is gone.

See what you got:

```bash
ls -lh /output/field/
```

| File | What it is |
|---|---|
| `transformations.pcd` | **the important one.** Every keyframe pose with its timestamp |
| `map_from_enu.txt` | the ENU→map transform the GNSS code fitted, plus how many factors it ended up using |
| `GlobalMap.pcd` | the whole accumulated point cloud, around 300 MB |
| `LOG/`, `SCDs/`, `PCDs/` | diagnostics, ignore them for now |

Ctrl-C the node in terminal 1 when you're done.

### insideGarage is different

That one was recorded indoors, so there's no GNSS fix anywhere in it. The `/gps`
topic is there but every field is zero. With no global sensor, loop closure is
the only thing left that can pull the drift back, so the two swap places:

```bash
roslaunch fast_lio_sam mapping_charlie8.launch seq:=insideGarage \
    gnss:=false loop:=true
```

You can't just turn both on, which is the interesting part. With GNSS and loop
closure both active the optimiser ends up satisfying neither of them and the
result is worse than either alone. The numbers behind that are in the comments
in `config/charlie8_gnss.yaml`. Go read them, it's a good example of something
you would never guess from first principles.

---

## 9. Make a trajectory file out of it

`transformations.pcd` is a point cloud file being abused as a list of poses,
which nothing else knows how to read. Convert it to **TUM format** instead, one
line per pose, `timestamp x y z qx qy qz qw`. Every SLAM evaluation tool on
earth reads that:

```bash
python3 /catkin_ws/src/better_fastlio2/tools/anchor/keyposes_to_tum.py \
    /output/field/transformations.pcd \
    /output/field/ground_truth.tum \
    --enu /output/field/map_from_enu.txt
```

`--enu` also spins the trajectory out of the SLAM frame (whose origin and
heading are just wherever the vehicle happened to be sitting when you hit play)
into **local ENU**, so x is east, y is north, z is up. No optimisation happens
here, it's a rigid rotation and shift using the transform the node already
worked out during the run.

You get a summary printed, plus `ground_truth.tum` and a `.json` beside it
saying how many GNSS factors went in.

---

## 10. Check you didn't get garbage

```bash
python3 /catkin_ws/src/better_fastlio2/tools/eval/validate.py \
    /output/field/ground_truth.tum
```

For `field` you want roughly **1500 poses, 154 s, ~215 m**. Look at all three
numbers, not just one. A span much shorter than 154 s means the node fell behind
and your trajectory got cut off. A path length that's obviously wrong, like 20 m
or 2000 m, means the run diverged and you should just do it again.

If you were given reference trajectories, compare against one:

```bash
python3 /catkin_ws/src/better_fastlio2/tools/eval/validate.py \
    /output/field/ground_truth.tum --compare /datasets/reference/field.tum
```

You're looking for an RMSE of a few centimetres. A clean `field` run comes in
around 0.05 m with the worst single pose about 0.13 m off. It won't be zero and
it shouldn't be: the node isn't deterministic, because thread scheduling changes
which scans get batched together, so no two runs match exactly. Anything well
under a metre is fine.

Four of the sequences come back to where they started (`ditches`,
`featuresAndGps`, `insideGarage`, `niceFeatures`). For those, add `--loop` and
it'll tell you how far the end of your trajectory landed from the beginning.
That's drift you can measure without needing a reference at all, which is handy.

---

## 11. Now go read what you just ran

It works, so this is the part where you find out what you did. Roughly in order
of how much you'll get out of it:

**`config/charlie8_gnss.yaml`.** Every parameter has a comment saying why it's
set the way it is. Start with `keyframeAddingDistThreshold`,
`gnss/factorInterval` and `gnss/looseCoupling`.

**`launch/mapping_charlie8.launch`.** How a launch file hooks up parameters and
nodes, and why `use_sim_time` is missing on purpose.

**`src/laserMapping.cpp`.** Search for `addGPSFactor`. The ~340 lines under the
`GNSS` banner comment are the ones this fork added, everything else is upstream.
[GNSS_FACTOR.md](GNSS_FACTOR.md) explains the design if you want the reasoning.

### Stuff worth trying

1. **Turn GNSS off** and see what it was actually buying you:
   `roslaunch fast_lio_sam mapping_charlie8.launch seq:=field gnss:=false`, then
   compare the two with `validate.py --compare`. On `field` they're metres
   apart.
2. **Break the de-skewing on purpose.** Set `time_unit: 2` in the config and run
   it again. Nothing will error, the trajectory just gets worse. Figuring out
   why from the config comments is the exercise.
3. **Run the same sequence twice** and change nothing. However much the two runs
   disagree is your repeatability, and you can't honestly claim any accuracy
   better than that.
4. **Poke at the topics while it's running,** from a third terminal:
   `rostopic hz /ouster0`, `rostopic echo -n1 /gps/odom_enu`,
   `rosnode info /laserMapping`.

---

## When it breaks

**`permission denied ... /var/run/docker.sock`**
The docker group change from step 1 hasn't taken effect. Log out, log back in.

**rviz won't open, or `cannot open display`**
You forgot `xhost +local:docker` on the host before starting the container. It
works fine on Wayland too (Ubuntu 22.04 and up), through XWayland.

**`libGL error: failed to load driver: iris` (or `i915`), or rviz is just black**
The container's graphics driver doesn't match your GPU. Force software
rendering, which is slower but always works: add `-e LIBGL_ALWAYS_SOFTWARE=1` to
your `docker run` and drop `--device /dev/dri`.

**`[rospack] Error: package 'fast_lio_sam' not found`**
Either you didn't `source /catkin_ws/devel/setup.bash` in this particular shell,
or you cloned the repo somewhere other than `~/slam/catkin_ws/src/`. It's
almost always the second one.

**`catkin build` dies with `c++: fatal error: Killed signal terminated`**
You ran out of RAM. Try `-j4`, or `-j2`.

**`RLException: Unable to contact my own server`**
ROS can't resolve its own hostname. You started the container without
`--net=host`. Use the command in step 6 exactly as written.

**The bag finishes and `/save_map` just hangs**
You played it with `--clock`, or something set `use_sim_time`. Kill it and run
it again without.

**No `transformations.pcd` after `/save_map`**
Either the node died before you called it, or the call errored. Look at
terminal 1. And remember the node wipes its output folder on startup, so if you
already restarted it, the previous run's files are gone too.

**The trajectory is way shorter than the sequence**
The node fell behind `rosbag play` and `/save_map` only got what it had
finished. Check `ave total` in terminal 1 (step 8), and make sure you're on a
Release build:
`grep CMAKE_BUILD_TYPE /catkin_ws/build/fast_lio_sam/CMakeCache.txt`.
Close whatever else is chewing your CPU and run it again.

**The trajectory is ten times too long, or wanders off into nowhere**
Usually the wrong config for that sequence, or a bag that isn't one of these
seven. Check `rosbag info` shows `/ouster0` and `/imu` with sensible message
counts.

**You're out of disk**
`GlobalMap.pcd` is ~300 MB per run. `/output/<seq>/` gets wiped when you re-run
*that* sequence, but not when you run a different one, so they pile up. Delete
what you don't need.

---

## Cheat sheet

```bash
# on the host
xhost +local:docker
./docker/charlie8/run_container.sh          # start it, or open another shell in it

# in the container, once
cd /catkin_ws && catkin build fast_lio_sam -j"$(nproc)" && source devel/setup.bash

# per sequence, terminal 1
roslaunch fast_lio_sam mapping_charlie8.launch seq:=SEQ
#           insideGarage only:  ... seq:=insideGarage gnss:=false loop:=true

# terminal 2
rosbag play /datasets/SEQ.bag
sleep 30
rosservice call /save_map "{resolution: 0.0, destination: ''}"

# terminal 2, afterwards
python3 /catkin_ws/src/better_fastlio2/tools/anchor/keyposes_to_tum.py \
    /output/SEQ/transformations.pcd /output/SEQ/ground_truth.tum \
    --enu /output/SEQ/map_from_enu.txt
python3 /catkin_ws/src/better_fastlio2/tools/eval/validate.py \
    /output/SEQ/ground_truth.tum
```
