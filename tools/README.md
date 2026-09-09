# tools

Python helpers around the SLAM node. Everything here runs with numpy only,
except `preproc/`, which needs the `bfl2-tools` image (see
[`../docker/charlie8/README.md`](../docker/charlie8/README.md)).

| Path | What |
|---|---|
| `anchor/keyposes_to_tum.py` | `transformations.pcd` → TUM, and the rigid map→ENU change of coordinates using the `map_from_enu.txt` the node writes. **This is the step that turns a run into a trajectory file.** |
| `eval/validate.py` | pose count, path length, loop-closure error, GNSS residual (needs the `.db3`), and comparison against a reference trajectory |
| `eval/gnss_residual_profile.py` | GNSS residual per decile of time. Finds localised failures that an aggregate rmse hides — this is what exposed the loop-closure conflict |
| `preproc/db3_to_ros1.py` | `.db3` → ROS 1 bag. Synthesises `ring` and per-point `time`, drops non-finite points, regenerates the IMU orientation, fills in covariances |
| `preproc/check_bag.py` | verifies a converted bag against the things that break the node quietly rather than loudly |
| `preproc/extrinsics.py` | `sensors.yaml` → `T_imu_sensor` for every sensor |
| `preproc/madgwick.py` | IMU orientation filter used by the converter |
| `preproc/determine_stamp_convention.py` | LiDAR/IMU temporal calibration |
| `preproc/cdr.py` | minimal CDR reader for the `.db3` |
| `sensors.yaml` | the vehicle's surveyed sensor geometry |

Typical use, inside the SLAM container after a run:

```bash
python3 tools/anchor/keyposes_to_tum.py \
    /output/field/transformations.pcd /output/field/ground_truth.tum \
    --enu /output/field/map_from_enu.txt

python3 tools/eval/validate.py /output/field/ground_truth.tum
```
