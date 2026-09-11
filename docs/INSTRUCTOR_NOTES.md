# Notes for whoever runs the lab

Companion to [STUDENT_GUIDE.md](STUDENT_GUIDE.md).

## What to hand out

| File | Size | Why |
|---|---|---|
| `field.bag` | 708 MB | shortest sequence, the one to start on |
| `featuresAndGps.bag` | 1.5 GB | cleanest data, looks good in rviz |
| one more of their choosing | 1.3–2.5 GB | `insideGarage.bag` for the no-GNSS case |
| `reference/<seq>.tum` | ~300 KB | optional, for step 10's `--compare` |
| `SHA256SUMS` | — | worth checking after a copy off a USB stick |

All seven bags come to ~11 GB, which is more than anyone needs for a lab. Two
sequences is plenty.

**Don't hand out:**

- the `ground_truth.json` sitting next to any reference `.tum`. It contains the
  ENU anchor lat/lon, which locates the recording site to a few metres. The
  `.tum` files are in a local frame and are fine.
- anything derived from the sponsor's `/pose` topic, or `.db3` files that still
  contain `/pose` and `/tf`.

Bags from `tools/preproc/db3_to_ros1.py` only carry `/ouster0`, `/imu` and the
GNSS topics, so they're already clean.

If you re-fork this under a different account, the clone URL appears in
`docs/STUDENT_GUIDE.md` and below:

```bash
sed -i 's|LouisThomasRoy/better_fastlio2|your-user/better_fastlio2|g' docs/*.md
```

## Regenerating the bags

From the raw `.db3` sequences, using the Python-only image. Students never touch
this:

```bash
docker build -t bfl2-tools -f docker/charlie8/Dockerfile.tools docker/charlie8
docker run --rm -v "$PWD":/ws -v /path/to/sequences:/data:ro -w /ws bfl2-tools \
    python3 tools/preproc/db3_to_ros1.py \
        /data/field-Charlie8/data.db3 /ws/out/field.bag
```

The converter synthesises `ring` and per-point `time` (the driver emits neither,
so without them nothing can de-skew), drops the 4–5% non-finite points,
regenerates the IMU orientation with a Madgwick filter because the recorded
quaternion is unusable, and fills in the all-zero covariances from the MTI-200
datasheet. `tools/preproc/check_bag.py` checks the result.

## Timing

| Step | Time | Ahead of time? |
|---|---|---|
| Install Docker | 5–10 min | yes, have them do it before turning up |
| `docker build` (GTSAM is most of it) | 20–40 min | **yes, and you should** |
| `catkin build` | 3–15 min | no, that's the exercise |
| One run of `field` | ~4 min | no |
| Export + validate | 1 min | no |

Two hours works if the image is already built, three if it isn't. On shared
machines, build it once under a shared tag and let them skip step 5, but still
make them read the `Dockerfile`.

```bash
docker build -t bfl2-gnss:noetic --build-arg UID=$(id -u) --build-arg GID=$(id -g) docker/charlie8
docker save bfl2-gnss:noetic | gzip > bfl2-gnss.tar.gz     # ~2 GB compressed
docker load < bfl2-gnss.tar.gz                             # on each machine
```

An image built with `UID=1000` writes root-owned files for anyone whose UID
isn't 1000. Fine if your lab machines are uniform, otherwise let them build it.

## What they get stuck on

1. **Cloning outside `catkin_ws/src/`.** catkin builds nothing, roslaunch says
   the package doesn't exist.
2. **Forgetting to source `devel/setup.bash`** in the shell that was open during
   the build.
3. **Playing the bag with `--clock`.** Looks normal until `/save_map` hangs
   forever and the run is gone. Worth saying out loud up front, because the
   failure comes late and gives no clue.
4. **Calling `/save_map` the moment playback stops**, which cuts the end off.
   Make them wait 30 s.
5. **rviz** — X11 permissions or a GPU driver mismatch.
   `xhost +local:docker`, then `LIBGL_ALWAYS_SOFTWARE=1`.
6. **OOM during `catkin build`** on 8 GB laptops with `-j$(nproc)`.
7. **A loaded machine.** The node has to keep real time against `rosbag play`, and
   if it can't, `/save_map` writes a truncated trajectory without erroring. The
   package defaults to Release for this reason, but a laptop under load can still
   fall behind. Tell: `ave total` above 0.1 in the `[ Mapping Time ]` lines.

And the one that isn't their fault: the node **deletes its output directory every
time it starts** (`fsmkdir()` → `fs::remove_all()`). Someone will re-run a
sequence to check something and lose the result they had. Warn them first.

## Checking their work

A correct `field` run is about **1500 poses, 154 s, 215 m**. A run straight
through the guide as written:

```
ground_truth.tum: 1536 poses, 153.5 s, 215.9 m
  vs reference (1536 matched)   rmse  0.046  med  0.031  p95  0.084  max  0.126 m
  vs reference RPE @ 10s        rmse  0.037  med  0.014  p95  0.098  max  0.170 m
```

Don't grade on an exact match. The node isn't deterministic under OpenMP with
real-time playback, and the same sequence twice moves the ENU→map fit rms by
~0.08 m. Grade on pose count, span, path length, and agreement under a metre.

A Release build reports `ave total` around 0.015 s per 0.1 s scan, so roughly
seven times faster than real time — plenty of headroom.

Good written question, answerable from the config comments alone: *why is loop
closure turned off on every sequence except `insideGarage`?*

## What stayed behind

This repo is the pipeline. The ground-truth workspace that drives it over all
seven sequences didn't come over: `run.sh`, the variant matrix, the old offline
two-stage pose graph (`anchor/gnss_posegraph.py`), the plotting and comparison
scripts, and the release builder. Students don't need any of it.
