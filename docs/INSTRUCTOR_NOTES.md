# Notes for whoever runs the lab

Everything you need to run [STUDENT_GUIDE.md](STUDENT_GUIDE.md) as a session,
plus the things that are obvious from my side of it and not from theirs.

## Before the first session

**Check the clone URL.** The guide tells students to clone from
`https://github.com/LouisThomasRoy/better_fastlio2.git`. If you fork this again
under a different account, that URL appears in two places, `docs/STUDENT_GUIDE.md`
and line 9 of this file:

```bash
sed -i 's|LouisThomasRoy/better_fastlio2|your-user/better_fastlio2|g' docs/*.md
```

**What each student needs a copy of:**

| File | Size | Why |
|---|---|---|
| `field.bag` | 708 MB | shortest sequence, so it's the one to start on |
| `featuresAndGps.bag` | 1.5 GB | cleanest data, looks good in rviz |
| one more of their choosing | 1.3–2.5 GB | `insideGarage.bag` if you want them to hit the no-GNSS case |
| `reference/<seq>.tum` | ~300 KB | optional, it's what step 10's `--compare` needs |
| `SHA256SUMS` | — | worth having them check it after copying off a stick |

All seven bags come to about 11 GB, which is more than most USB sticks hold and
more than anyone needs for a lab. Two sequences is plenty. Hand out the rest if
someone asks.

**Don't hand out** these, which sit in the same output folders and are easy to
grab by accident:

- the `ground_truth.json` next to any reference `.tum`. It has the ENU anchor
  latitude and longitude in it, which puts the recording site on a map to within
  a few metres. The `.tum` files themselves are in a local frame, so those are
  fine.
- anything that came from the sponsor's `/pose` topic, or `.db3` files that still
  have `/pose` and `/tf` in them.

The bags out of `tools/preproc/db3_to_ros1.py` only contain `/ouster0`, `/imu`
and the GNSS topics, so they're already clean.

## Regenerating the bags

The bags come from the raw `.db3` sequences via the second, Python-only image.
Students never touch this. It's here so the repo can rebuild its own inputs:

```bash
docker build -t bfl2-tools -f docker/charlie8/Dockerfile.tools docker/charlie8
docker run --rm -v "$PWD":/ws -v /path/to/sequences:/data:ro -w /ws bfl2-tools \
    python3 tools/preproc/db3_to_ros1.py \
        /data/field-Charlie8/data.db3 /ws/out/field.bag
```

The converter does four things that matter, all written up in its header. It
synthesises `ring` and per-point `time`, since the driver emits neither and
without them nothing can de-skew. It drops the 4–5% of points that come back
non-finite. It regenerates the IMU orientation with a Madgwick filter, because
the recorded quaternion is unusable (rotating measured gravity by it gives you
`[8.91, 3.36, 2.39]` instead of `[0, 0, 9.81]`). And it fills in the covariances,
which are all zero in the recording, from the MTI-200 datasheet.

`tools/preproc/check_bag.py` checks a converted bag for the specific things that
make better_fastlio2 fail quietly instead of loudly.

## How long it takes

| Step | Time | Can you do it ahead? |
|---|---|---|
| Install Docker | 5–10 min | yes, tell them to do it before they turn up |
| `docker build` (GTSAM is most of it) | 20–40 min | **yes, and you should.** See below |
| `catkin build` | 3–15 min | no, this is the point of the exercise |
| One run of `field` | ~4 min | no |
| Export + validate | 1 min | no |

Two hours works if the image is already built. Three if it isn't. On shared lab
machines, build the image once under a shared tag and let them skip step 5, but
still make them open the `Dockerfile` and read it, because that file is the
actual answer to "how do I install this thing".

To pre-seed machines without everyone hammering the network at once:

```bash
docker build -t bfl2-gnss:noetic --build-arg UID=$(id -u) --build-arg GID=$(id -g) docker/charlie8
docker save bfl2-gnss:noetic | gzip > bfl2-gnss.tar.gz     # ~2 GB compressed
# then on each machine:
docker load < bfl2-gnss.tar.gz
```

One catch with that: an image built with `UID=1000` will write root-owned files
for anyone whose UID isn't 1000. If everyone on your lab machines is 1000 you're
fine, otherwise let them build it themselves.

## What they'll actually get stuck on

Roughly in order of how often it happens. All of it is in the guide's
troubleshooting section, but these are the ones worth watching for:

1. **Cloning the repo somewhere other than `catkin_ws/src/`.** Then catkin builds
   nothing and roslaunch says the package doesn't exist.
2. **Forgetting to source `devel/setup.bash`** in the shell they had open while
   it was building.
3. **Playing the bag with `--clock`.** Everything looks completely normal until
   `/save_map` hangs forever and the run is gone. Worth saying out loud at the
   start, because the failure comes late and gives no clue.
4. **Calling `/save_map` the second playback stops,** which cuts the end off the
   trajectory. Make them wait the 30 s.
5. **rviz.** Either X11 permissions or a GPU driver mismatch in the container.
   `xhost +local:docker` first, then `LIBGL_ALWAYS_SOFTWARE=1` if it's still
   unhappy.
6. **Running out of RAM during `catkin build`** on 8 GB laptops with
   `-j$(nproc)`.
7. **A slow or loaded machine.** The node has to keep real time against
   `rosbag play`, and if it can't, `/save_map` writes a truncated trajectory
   without erroring. The package defaults to a Release build now precisely
   because of this, but a laptop with a browser full of tabs can still fall
   behind. The tell is `ave total` above 0.1 in the `[ Mapping Time ]` lines, and
   a span in step 10 that's shorter than the sequence.

And the one that isn't their fault but looks like it: the node **deletes its
output directory every time it starts** (`fsmkdir()` → `fs::remove_all()`).
Somebody will re-run a sequence to check one thing and lose the result they
already had. Tell them before it happens.

## Checking their work

A correct `field` run lands around **1500 poses, 154 s, 215 m**. Here's a run I
did straight through the guide as written:

```
ground_truth.tum: 1536 poses, 153.5 s, 215.9 m
  vs reference (1536 matched)   rmse  0.046  med  0.031  p95  0.084  max  0.126 m
  vs reference RPE @ 10s        rmse  0.037  med  0.014  p95  0.098  max  0.170 m
```

Don't expect that to reproduce digit for digit and don't grade it that way. The
node isn't deterministic under OpenMP with real-time playback, and running the
same sequence twice moves the ENU→map fit rms by about 0.08 m. Grade on pose
count, span, path length, and agreement comfortably under a metre.

For what it's worth on timing, a Release build reports `ave total` around 0.015 s
per 0.1 s scan, so roughly seven times faster than real time. There's plenty of
headroom on a normal laptop.

If you want a written question out of this, here's a good one, and the config
comments contain everything needed to answer it: *why is loop closure turned off
on every sequence except `insideGarage`?*

## What stayed behind

This repo is the pipeline itself. The ground-truth workspace that drives it over
all seven sequences didn't come over: `run.sh`, the variant matrix
(`lidar0`/`lidar1`/`dual`/`gnss`), the old offline two-stage pose graph
(`anchor/gnss_posegraph.py`), the plotting and cross-variant comparison scripts,
and the release builder. Students don't need any of it, and it's full of paths
and conventions that only make sense in that workspace.

What did come over: the node, its configs and launch file, the converter, the TUM
export and `validate.py`.
