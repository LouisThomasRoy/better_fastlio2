# Instructor notes

Everything a TA needs to run [STUDENT_GUIDE.md](STUDENT_GUIDE.md) as a lab, and
the things in it that are easy to get wrong from the other side of the desk.

## Before the first session

**Replace the placeholder repository URL.** The guide clones from
`https://github.com/<GITHUB-USER>/better_fastlio2.git`. Once the fork exists:

```bash
grep -rl '<GITHUB-USER>' . | xargs sed -i 's|<GITHUB-USER>|your-github-username|g'
```

**Hand out, per student:**

| File | Size | Notes |
|---|---|---|
| `field.bag` | 708 MB | the one to start on — shortest sequence |
| `featuresAndGps.bag` | 1.5 GB | best-conditioned; the one that looks good |
| one more, their choice | 1.3–2.5 GB | `insideGarage.bag` if you want the no-GNSS case |
| `reference/<seq>.tum` | ~300 KB | optional, for step 10's `--compare` |
| `SHA256SUMS` | — | students should verify after copying off a USB stick |

All seven bags are ~11 GB total, which is more than most USB sticks and more
than most students need. Two sequences is enough for the lab; hand out the rest
on request.

**Do not hand out** — these are in the same output tree and are easy to sweep up
by accident:

- `ground_truth.json` next to any reference `.tum`. It carries the ENU anchor
  latitude and longitude, which geolocates the recording site to a few metres.
  The `.tum` files themselves are in a local frame and are safe.
- anything derived from the sponsor's `/pose` topic, or `.db3` files that still
  contain `/pose` and `/tf`.

The bags produced by `tools/preproc/db3_to_ros1.py` contain only `/ouster0`,
`/imu` and the GNSS topics, so they are already clean.

## Regenerating the bags

The bags are built from the raw `.db3` sequences by the second, Python-only
image. This is not part of the student path — it is here so the repository can
reproduce its own inputs:

```bash
docker build -t bfl2-tools -f docker/charlie8/Dockerfile.tools docker/charlie8
docker run --rm -v "$PWD":/ws -v /path/to/sequences:/data:ro -w /ws bfl2-tools \
    python3 tools/preproc/db3_to_ros1.py \
        /data/field-Charlie8/data.db3 /ws/out/field.bag
```

The converter does four things that matter, all documented in its header: it
synthesises `ring` and per-point `time` (the driver emits neither, so nothing
could de-skew without it), drops the 4–5% non-finite points, regenerates the IMU
orientation with a Madgwick filter because the recorded quaternion is unusable,
and fills in the all-zero covariances from the MTI-200 datasheet.

`tools/preproc/check_bag.py` verifies a converted bag against the things that
break better_fastlio2 quietly rather than loudly.

## Timing a session

| Step | Time | Can it be done in advance? |
|---|---|---|
| Install Docker | 5–10 min | yes — tell them to do it before the session |
| `docker build` (GTSAM dominates) | 20–40 min | **yes, and you should.** See below |
| `catkin build` | 5–15 min | no — this is the point of the exercise |
| One run of `field` | ~4 min | no |
| Export + validate | 1 min | no |

Two hours is comfortable if the image build happens beforehand; three if it does
not. If your lab machines are shared, build the image once as a shared tag and
have students skip step 5 — but make them read the `Dockerfile` anyway, since it
is the actual answer to "how do I install this".

To pre-seed machines without network pressure on the day:

```bash
docker build -t bfl2-gnss:noetic --build-arg UID=$(id -u) --build-arg GID=$(id -g) docker/charlie8
docker save bfl2-gnss:noetic | gzip > bfl2-gnss.tar.gz     # ~2 GB compressed
# on each machine:
docker load < bfl2-gnss.tar.gz
```

Note the UID caveat: an image built with `UID=1000` writes root-owned files for a
student whose UID is not 1000. On lab machines where everyone is UID 1000 this is
fine; otherwise let them build it themselves.

## What students actually get wrong

In rough order of frequency, all covered in the guide's troubleshooting section:

1. **Cloning the repo outside `catkin_ws/src/`.** Then `catkin build` finds
   nothing and `roslaunch` reports the package missing.
2. **Not sourcing `devel/setup.bash`** in the shell that was open during the
   build.
3. **Playing the bag with `--clock`.** Everything looks fine until `/save_map`
   hangs forever and they lose the run. This one is worth calling out at the
   front of the session — the failure is silent and late.
4. **Calling `/save_map` immediately** when playback ends, getting a trajectory
   that stops early. Make them wait 30 s.
5. **rviz** — X11 permissions, or GPU driver mismatch inside the container.
   `xhost +local:docker`, then `LIBGL_ALWAYS_SOFTWARE=1` if it is still unhappy.
6. **OOM during `catkin build`** on 8 GB laptops with `-j$(nproc)`.
7. **A slow machine.** The node has to keep real time against `rosbag play`; if
   it cannot, `/save_map` writes a truncated trajectory and nothing errors. The
   package now defaults to a Release build for this reason, but a laptop under
   load can still fall behind. The tell is `ave total` > 0.1 in the
   `[ Mapping Time ]` lines, and a span in step 10 shorter than the sequence.

The one that is not their fault and looks like it is: the node **wipes its output
directory at startup** (`fsmkdir()` → `fs::remove_all()`). A student who re-runs
a sequence to "check something" loses the previous result. Say it out loud.

## Grading / checking a result

A correct `field` run gives roughly:

```
ground_truth.tum: ~1500 poses, 154.0 s, ~215 m
```

Against a reference trajectory, ATE RMSE should be a few centimetres up to about
0.2 m. It is **not** reproducible to the digit — the node is not deterministic
under OpenMP and real-time playback, and repeat runs of the same sequence move
the ENU→map fit rms by ~0.08 m. Do not grade on an exact match; grade on
path length, pose count, and agreement well under a metre.

A good written question, answerable from the config comments alone: *why is loop
closure disabled on every sequence except `insideGarage`?*

## What did not come over from the research workspace

This repository is the pipeline. The ground-truth generation workspace that
drives it over all seven sequences — `run.sh`, the variant matrix
(`lidar0`/`lidar1`/`dual`/`gnss`), the offline two-stage pose graph
(`anchor/gnss_posegraph.py`), the plotting and cross-variant comparison scripts,
and the release builder — stays where it is. Students do not need it, and it
encodes paths and dataset conventions that only make sense there.

What did come over: the node, its configs and launch file, the converter, the
TUM export, and `validate.py`.
