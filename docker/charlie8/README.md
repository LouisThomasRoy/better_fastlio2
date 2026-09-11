# The container

Two images live here. You only need the first.

## `Dockerfile` — the SLAM environment

ROS Noetic, GTSAM 4.0.3, PCL, Eigen, GeographicLib, and the two ROS message
packages `fast_lio_sam` depends on. It contains **no pipeline source code**: you
clone that yourself and bind-mount it, so you can edit on the host and compile in
the container.

```bash
docker build -t bfl2-gnss:noetic \
    --build-arg UID=$(id -u) --build-arg GID=$(id -g) docker/charlie8
```

`UID`/`GID` make the container's user match yours, so the build tree and results
it writes into your bind mounts belong to you rather than to root.

Three mounts, and the container expects all three:

| Container path | Host path (suggested) | Mode |
|---|---|---|
| `/catkin_ws` | `~/slam/catkin_ws` | rw — your workspace, with this repo in `src/` |
| `/datasets` | `~/slam/datasets` | **ro** — the bags |
| `/output` | `~/slam/output` | rw — trajectories, maps, logs |

`/datasets` is read-only on purpose: the mapping node calls `fs::remove_all()` on
its output directory at startup, and a mistyped `outdir:=` should not be able to
delete your data.

`run_container.sh` runs the full `docker run` for you and handles X11 and GPU
passthrough. Run it a second time and it attaches another shell to the container
that is already up. [`../../docs/STUDENT_GUIDE.md`](../../docs/STUDENT_GUIDE.md)
walks through the command it replaces — learn that first.

## `Dockerfile.tools` — the converter

Python only, no ROS. Converts the raw `.db3` sequences into the ROS 1 bags the
pipeline consumes. You only need it if you were given raw `.db3` data rather than
bags; the converter's own header documents what it fixes and why.

```bash
docker build -t bfl2-tools -f docker/charlie8/Dockerfile.tools docker/charlie8
```

## Upstream's Dockerfile

`../Dockerfile` is the original from better_fastlio2 — an `nvidia/cuda` base that
bakes the source into the image. It is left untouched for reference. Nothing in
the mapping path needs CUDA, which is why this one starts from plain Ubuntu 20.04
and runs on any laptop.
