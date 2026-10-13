# GMR `.pkl` data contract — Unitree G1 29-DOF

Verified against the GMR save path used by this project. The motion file is a pickled dictionary:

| Key | Type / shape | Meaning |
|---|---|---|
| `fps` | scalar integer | Motion frame rate; current LAFAN1 suite uses 30 FPS. |
| `root_pos` | `float64[T, 3]` | Floating-base world position in metres. |
| `root_rot` | `float64[T, 4]` | Floating-base quaternion in **xyzw** order. MuJoCo qpos uses wxyz, so loaders must reorder. |
| `root_quat_order` | `str` | Always `"xyzw"`. Written by every tool in this repo; loaders refuse a pkl where it is missing or different unless the order is given explicitly (`--quat-order`, `quat_order=`). See `mocap_g1/gmr_pkl.py`. |
| `dof_pos` | `float64[T, 29]` | Joint angles in radians, in the loaded MJCF's hinge-joint declaration order. |
| `joint_names` | `list[str]` (29) | The `dof_pos` column names, in column order. Written by `tools/convert_npz_to_gmr_pkl.py` (converter report schema v2, 2026-10-12). `eval`'s `run_fk_pass` refuses to score a pkl whose `joint_names` differ from the scoring MJCF's hinge order; a legacy pkl without the key is still scored, but its result carries `joint_order.status: "unverified_legacy_pkl"` and a WARNING note, and the qualification driver's stage 8 treats anything but `"verified"` as BLOCKED. |
| `local_body_pos` | `None` | Not populated by this pipeline. |
| `link_body_list` | `None` | Not populated by this pipeline. |

`T` varies by clip. The faithful replica of GMR's historical `bvh_to_robot.py` path drops the final
BVH frame; do not infer duration from a source filename or assume an alternate writer does the same.

## Joint order observed in the 29-DOF G1 MJCF

| Index | Joint | Index | Joint |
|---:|---|---:|---|
| 0 | `left_hip_pitch_joint` | 15 | `left_shoulder_pitch_joint` |
| 1 | `left_hip_roll_joint` | 16 | `left_shoulder_roll_joint` |
| 2 | `left_hip_yaw_joint` | 17 | `left_shoulder_yaw_joint` |
| 3 | `left_knee_joint` | 18 | `left_elbow_joint` |
| 4 | `left_ankle_pitch_joint` | 19 | `left_wrist_roll_joint` |
| 5 | `left_ankle_roll_joint` | 20 | `left_wrist_pitch_joint` |
| 6 | `right_hip_pitch_joint` | 21 | `left_wrist_yaw_joint` |
| 7 | `right_hip_roll_joint` | 22 | `right_shoulder_pitch_joint` |
| 8 | `right_hip_yaw_joint` | 23 | `right_shoulder_roll_joint` |
| 9 | `right_knee_joint` | 24 | `right_shoulder_yaw_joint` |
| 10 | `right_ankle_pitch_joint` | 25 | `right_elbow_joint` |
| 11 | `right_ankle_roll_joint` | 26 | `right_wrist_roll_joint` |
| 12 | `waist_yaw_joint` | 27 | `right_wrist_pitch_joint` |
| 13 | `waist_roll_joint` | 28 | `right_wrist_yaw_joint` |
| 14 | `waist_pitch_joint` |  |  |

## Safe consumption rule

The table documents the current artifact; it is not an API guarantee. Resolve `dof_pos` columns by
joint name from the MJCF loaded for that motion:

```python
import mujoco

model = mujoco.MjModel.from_xml_path(g1_mjcf_path)
joint_names = [
    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
    for jid in range(model.njnt)
    if model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE
]
column = {name: index for index, name in enumerate(joint_names)}
waist_yaw = dof_pos[:, column["waist_yaw_joint"]]
```

Pkls from `tools/convert_npz_to_gmr_pkl.py` carry this list as `joint_names`, and
`eval/retarget_metrics/robot.py`'s `verify_dof_joint_order` performs exactly this check
before any FK or limit lookup. Every exported dataset manifest should record the ordered
joint-name list and the MJCF hash. CSV
exports should use joint names as headers; never ship unlabeled numeric columns.
