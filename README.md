# G1 人体运动参考轨迹数据集（RL 训练用）

实拍人体运动 → Unitree G1（**29-DoF，不含手指**）重定向参考轨迹，供强化学习/控制器开发使用。

**流水线**：D435i 实拍 RGB-D → FrankMocap 姿态估计 → 深度融合定标（根 + 脚底，深度真实参与）→ GMR 差分 IK 重定向 → 接触后处理（PhysCap SIGGRAPH Asia 2020 + Contact & Human Dynamics ECCV 2020 文献方法：接触精修 + 根高约束最小二乘 + 三指标门禁）→ MuJoCo 运动学回放验证。
生成日期 2026-08-26/27，30 fps。

---

## ⚠️ 使用前必读的口径

1. **轨迹是运动学重定向产物，未经过动力学仿真验证。** 视频是 `kinematic playback`（逐帧设定 qpos 渲染），**不是**机器人实际能走出来的证明。机器人能否复现由你的 RL/控制器决定——这正是这份数据的用途。动力学 PD 基线（M5）尚未完成。
2. **29-DoF 契约**：不含手指。qpos 布局见下节，关节顺序与限位表必须逐项对上。
3. **已知轨迹特征**（不是 bug，是重定向的固有形态，见「已知问题」）：摆动脚滞后导致接触标签在摆动末期不可靠（gait 类 skate 30~42%）、深蹲高脚悬空 3.84%、march 单帧根 z 阶跃 5.7 cm（≈1.7 m/s 速度尖峰）。训练时建议对这几类帧做数据清洗或容错。

---

## 目录结构

每个 take 一个目录，含：

| 文件 | 说明 |
|---|---|
| `robot_motion.npz` / `.json` | ★ **RL 参考轨迹**（M3 后处理最终产物，用这个） |
| `gmr_raw.npz` / `.json` | GMR 重定向原始输出（未后处理，根 z 会穿地，别用） |
| `human_motion.npz` / `.json` | 源人体运动（SMPL 24 关节，M2 产物） |
| `mujoco_kinematic.mp4` | 运动学回放视频（kinematic playback，非物理仿真） |
| `playback.json` | M4 回放门禁结果 |

`models/unitree_g1_menagerie/`：仿真用机器人模型副本（MuJoCo Menagerie unitree_g1，含 STL 资产与 LICENSE，38M）。README 之外还有 `walk_diagnostic.html`、`penetration_dist_*.png` 两个 M3 诊断件（可忽略），以及两篇 PDF（PhysCap / Contact & Human Dynamics，接触后处理方法的参考文献，arXiv 编号见文件内引用）。

## robot_motion.npz schema（重点）

| 键 | 形状 | 说明 |
|---|---|---|
| `qpos` | (T, 36) | 前 7 = 根位姿 `[x, y, z, qw, qx, qy, qz]`（自由关节 pelvis）；后 29 = 关节角度（rad），`qpos[:, 7+i]` 对应下方关节表第 i 行 |
| `qvel` | (T, 35) | `[0:3]` 根线速度（世界系，m/s）、`[3:6]` 根角速度（世界系，so3 log，rad/s）、`[6:35]` 29 个关节角速度（中心差分） |
| `contact` | (T, 2) bool | 后处理精修接触标签，**索引 0 = 左脚，1 = 右脚** |
| `source_contact` | (T, 2) bool | 人体检测的原始接触标签（精修前） |
| `timestamps` | (T,) | 秒（float64），dt ≈ 0.0333（30 fps） |
| `valid` | (T,) bool | 全 True（无效帧已剔除） |
| `source_valid_idx` | (T,) | 本行对应的 human_motion.npz 源帧索引 |
| `task_pos_error` / `task_rot_error` | (T,) | GMR IK 任务误差（m / rad），轨迹质量参考 |

## 坐标系与单位

- 世界系：右手系，**x 前 / y 左 / z 上**，单位米。
- 四元数顺序：**wxyz**。
- 地面 = z 0（M1 几何事实）。脚底最低点全程 ≥ −1 cm（构造性保证，7 条 take 实测 min = +0.0 mm）。
- 关节角度单位 rad。

## 29-DoF 关节顺序与限位

`qpos[:, 7+i]` 的 i 按此表（限位取 menagerie `g1.xml`，rad）：

| i | 关节名 | min | max |
|---|---|---|---|
| 0 | left_hip_pitch_joint | -2.5307 | 2.8798 |
| 1 | left_hip_roll_joint | -0.5236 | 2.9671 |
| 2 | left_hip_yaw_joint | -2.7576 | 2.7576 |
| 3 | left_knee_joint | -0.0873 | 2.8798 |
| 4 | left_ankle_pitch_joint | -0.8727 | 0.5236 |
| 5 | left_ankle_roll_joint | -0.2618 | 0.2618 |
| 6 | right_hip_pitch_joint | -2.5307 | 2.8798 |
| 7 | right_hip_roll_joint | -2.9671 | 0.5236 |
| 8 | right_hip_yaw_joint | -2.7576 | 2.7576 |
| 9 | right_knee_joint | -0.0873 | 2.8798 |
| 10 | right_ankle_pitch_joint | -0.8727 | 0.5236 |
| 11 | right_ankle_roll_joint | -0.2618 | 0.2618 |
| 12 | waist_yaw_joint | -2.6180 | 2.6180 |
| 13 | waist_roll_joint | -0.5200 | 0.5200 |
| 14 | waist_pitch_joint | -0.5200 | 0.5200 |
| 15 | left_shoulder_pitch_joint | -3.0892 | 2.6704 |
| 16 | left_shoulder_roll_joint | -1.5882 | 2.2515 |
| 17 | left_shoulder_yaw_joint | -2.6180 | 2.6180 |
| 18 | left_elbow_joint | -1.0472 | 2.0944 |
| 19 | left_wrist_roll_joint | -1.9722 | 1.9722 |
| 20 | left_wrist_pitch_joint | -1.6144 | 1.6144 |
| 21 | left_wrist_yaw_joint | -1.6144 | 1.6144 |
| 22 | right_shoulder_pitch_joint | -3.0892 | 2.6704 |
| 23 | right_shoulder_roll_joint | -2.2515 | 1.5882 |
| 24 | right_shoulder_yaw_joint | -2.6180 | 2.6180 |
| 25 | right_elbow_joint | -1.0472 | 2.0944 |
| 26 | right_wrist_roll_joint | -1.9722 | 1.9722 |
| 27 | right_wrist_pitch_joint | -1.6144 | 1.6144 |
| 28 | right_wrist_yaw_joint | -1.6144 | 1.6144 |

重定向内部用的 GMR `g1_mocap_29dof.xml` 限位比上表更严；**两条轨迹的 qpos 与 menagerie 模型逐位兼容**（已验证同 qpos → 位姿差 0.0），用哪个模型加载都能直接吃。

## 机器人模型

- `models/unitree_g1_menagerie/g1.xml`：MuJoCo Menagerie unitree_g1（`nq=36, nv=35, nu=29`），29 个与关节同名的 position actuator（资产原值 kp=500）。mesh 相对路径按 cwd 解析，加载示例里已处理。
- 上游源：<https://github.com/google-deepmind/mujoco_menagerie>（commit `da76818e`）；重定向模型上游 <https://github.com/nvlabs/gmr>（commit `bb1bbe40`）。

## 7 条 take

| take | 目录 | 帧数/时长 | 内容 | 穿地(min) | Floating | Skate | 根 z 站立 std |
|---|---|---|---|---|---|---|---|
| stand | `stand_20260826_003347` | 423 / 14.1s | 站立 | 0.0mm | 0% | 1.30% | 6.0mm |
| arm_raise | `arm_raise_20260826_012045` | 573 / 19.1s | 双臂上举 | 0.0mm | 0% | 6.11% | 6.7mm |
| wave | `wave_20260826_010641` | 572 / 19.1s | 挥手 | 0.0mm | 0% | 7.19% | 4.7mm |
| squat | `squat_20260826_010732` | 573 / 19.1s | 深蹲 | 0.0mm | 3.84% | 29.53% | 豁免（下蹲动作本身） |
| walk | `walk_fwd_back_20260826_013009` | 572 / 19.1s | 前进-后退走 | 0.0mm | 0.76% | 41.77% | 豁免（步态） |
| march | `march_20260826_013216` | 573 / 19.1s | 原地踏步 | 0.0mm | 0.00% | 36.67% | 豁免（步态） |
| turn | `turn_20260826_013401` | 575 / 19.2s | 原地转身 | 0.0mm | 0.09% | 35.06% | 11.1mm |

- 穿地 min：MuJoCo FK 下 8 个脚球（4/脚，r=5mm）最低点距地面，全程 ≥ −1 cm 硬门禁。
- Floating：接触脚悬空 >3 cm 的帧占比（门禁 ≤5%）；Skate：接触脚滑移 >2 cm/帧的帧占比（只测不拦，成因见「已知问题」）。
- 左右互换检查（数值判据）：cos(左−右脚矢量, 身体左轴) 中位数 0.951~0.992，无系统性为负，未互换。

## 加载示例

```python
import re
from pathlib import Path
import numpy as np
import mujoco

repo = Path("models/unitree_g1_menagerie").resolve()   # 必须绝对路径:g1.xml 的
                                                       # meshdir="assets" 会给相对
                                                       # 路径再前置一层 assets/
xml = (repo / "g1.xml").read_text()
# mesh 相对路径按 cwd 解析，改写为绝对路径
xml = re.sub(r'file="([^/"]+\.STL)"', rf'file="{repo / "assets"}/\1"', xml)
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

d = np.load("stand_20260826_003347/robot_motion.npz")
qpos = d["qpos"]            # (T, 36): [root 7 | 29 joints]
for t in range(len(qpos)):
    data.qpos[:] = qpos[t]
    mujoco.mj_forward(model, data)
    # data.xpos / data.xquat 即各 body 世界位姿
```

## 已知问题（训练前建议处理）

1. **Skate 30~42%（gait 类）**：GMR 摆动脚滞后——摆动末期脚底穿地 → 被接触精修判为接触钉在地面 → 随后滑移。表现：`contact` 标签在摆动末段不可靠、脚底有滑移帧。修法需腿部 IK 级改动，超出当前范围；训练数据清洗时可按 `skate` 帧剔除或用运动学一致性过滤。
2. **squat 高脚悬空 3.84%**：深蹲双支撑不对称，较高脚悬空 3~5 cm（全部人标签接触、脚基本静止），GMR 膝/脚任务权重妥协（与膝盖残差同类）。
3. **march 单帧根 z 阶跃 5.7 cm**：接触硬等式 + 根-z-only 拟合的固有形态，表现为单帧根高抬升、根 z 速度尖峰 ≈1.7 m/s。PD/RL 跟踪时该帧是已知挑战。
4. **膝盖残差 ~16°**：人站立膝盖弯 33~40°（FrankMocap 估计），机器人复现 23~25°（GMR pelvis+feet 权重 100 vs knee 10 的妥协），诚实记录。
5. **GMR IK 误差**：pos_err mean 6.6~7.4 cm、rot_err mean 8.0~12.5°（squat rot p99 19.3°）。
6. 关节时序平滑已入管线（帧间速度 < 30°/帧、加速度 p99 < 10°/帧² 门禁保证）。

## 更新日志

- 2026-08-26/27：M1 实录 7 take → M2 人体运动 → M3 GMR 重定向 + 接触后处理（六门禁全过）→ M4 运动学回放（门禁全过）。M5 PD 动力学基线待做。
