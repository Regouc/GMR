# GVHMR+GMR 重定向输出说明(PPO 轨迹跟踪用)

面向 RL 队友(M5)的数据使用说明。**管线版本:v1(2026-08-31)**,脚-地接触修正版 v2 明天出,
**pkl 格式/帧数/关节顺序与 v1 完全一致**,按本说明写的 sim 代码可无缝切换。

## 1. 管线一句话

```
RGB mp4(30fps, 三脚架固定机位)
  → GVHMR 视频级人体姿态估计(SMPL-X)
  → 接地(ground_hmr4d.py,修 GVHMR 走路时根高度上漂 ~8.7cm)
  → GMR 官方转换器重定向到 Unitree G1(29-DoF)
  → robot_motion.pkl + 官方渲染视频
```

只对喂给转换器的**人体数据**做接地修正,GMR 官方代码零改动,重定向结果未做任何后处理。

## 2. 数据清单

数据在 `outputs_gvhmr/<take>/robot_motion.pkl`;机器人视频在同目录 `videos/unitree_g1_hmr4d_results.mp4`
(官方渲染器,与 pkl 同一份数据)。

| take | 内容 | 帧数 N | 时长(N/30fps) |
|---|---|---|---|
| stand | 站立 | 436 | 14.5s |
| arm_raise | 手臂上举 | 586 | 19.5s |
| wave | 挥手 | 586 | 19.5s |
| squat | 下蹲 | 586 | 19.5s |
| walk | 往返行走 | 585 | 19.5s |
| march | 原地踏步 | 586 | 19.5s |
| turn | 转身 | 588 | 19.6s |

## 3. pkl 格式

标准 Python pickle,键值:

| 键 | 形状 | 说明 |
|---|---|---|
| `fps` | int | 30 |
| `root_pos` | (N,3) | 根(骨盆)世界位置,m;z 已接地 |
| `root_rot` | (N,4) | 根朝向四元数,**xyzw 序** |
| `dof_pos` | (N,29) | 29 个关节角,弧度,顺序见 §5 |

- 无 NaN;四元数已归一化;关节角均在限位内(已校验)。
- **N = 源视频帧数 − 1**:官方脚本从第 1 帧起输出、丢弃第 0 帧(外部代码怪癖,未改)。
  不要假设 N 等于源视频帧数,也不要去"修"它——v2 同样如此。
- 29-DoF 顺序与上一版 FrankMocap 管线交付的契约**同序**,老代码不用改。

加载与拼接(Isaac Gym / MuJoCo 口径):

```python
import pickle
import numpy as np

p = pickle.load(open("robot_motion.pkl", "rb"))
fps, rp, rr, dof = p["fps"], p["root_pos"], p["root_rot"], p["dof_pos"]

# 根四元数 xyzw -> wxyz(MuJoCo/Isaac Gym 口径)
qwxyz = np.c_[rr[:, 3], rr[:, 0], rr[:, 1], rr[:, 2]]
# 整机 qpos:(N, 36) = 自由关节 7(位置3+朝向4)+ 29 关节
qpos = np.concatenate([rp, qwxyz, dof], axis=1)
```

## 4. 仿真资产

**必须用同一个 xml**:`external/gmr/assets/unitree_g1/g1_mocap_29dof.xml`(重定向与回放同一模型)。
Isaac Gym 建 asset 时从该 xml 转,保证关节顺序/限位/连杆几何/接触几何一致,否则接触与
传感器读数会对不上。GMR 官方重定向在 MuJoCo 上做,关节角与限位口径都是 xml 里的原始值。

## 5. 29-DoF 关节顺序与限位(qpos 第 8~36 维,单位度)

| idx | 关节 | 限位 | idx | 关节 | 限位 |
|---|---|---|---|---|---|
| 0 | left_hip_pitch | [-90, 90] | 15 | waist_pitch | [-29.8, 29.8] |
| 1 | left_hip_roll | [-30, 90] | 16 | left_shoulder_pitch | [-177, 65.8] |
| 2 | left_hip_yaw | [-90, 90] | 17 | left_shoulder_roll | [-34.4, 129] |
| 3 | left_knee | [-5, 165] | 18 | left_shoulder_yaw | [-80.2, 114.6] |
| 4 | left_ankle_pitch | [-50, 30] | 19 | left_elbow | [-60, 97.4] |
| 5 | left_ankle_roll | [-15, 15] | 20 | left_wrist_roll | [-113, 113] |
| 6 | right_hip_pitch | [-90, 90] | 21 | left_wrist_pitch | [-92.5, 92.5] |
| 7 | right_hip_roll | [-90, 30] | 22 | left_wrist_yaw | [-92.5, 92.5] |
| 8 | right_hip_yaw | [-90, 90] | 23 | right_shoulder_pitch | [-177, 65.8] |
| 9 | right_knee | [-5, 165] | 24 | right_shoulder_roll | [-129, 34.4] |
| 10 | right_ankle_pitch | [-50, 30] | 25 | right_shoulder_yaw | [-114.6, 80.2] |
| 11 | right_ankle_roll | [-15, 15] | 26 | right_elbow | [-60, 97.4] |
| 12 | waist_yaw | [-90, 90] | 27 | right_wrist_roll | [-113, 113] |
| 13 | waist_roll | [-29.8, 29.8] | 28 | right_wrist_pitch | [-92.5, 92.5] |
| 14 | waist_pitch | [-29.8, 29.8] | | | |

## 6. 接触判据口径

脚接触用 xml 里的**脚球**(每脚 4 个 r=0.005 球,ankle_roll 连杆上,局部位置
x∈{-0.05,0.12}, y=±0.025/±0.03, z=-0.03):脚底参考平面 = 球心 z − 0.005 = **局部 z − 0.035**。
每帧取 4 球最低点即可。

注意:该资产的**视觉鞋底比功能鞋底低约 7cm**(STL 网格特性),官方渲染视频里脚踝柱截面会贴地,
**别用视频画面判断接触**,以脚球 FK 为准。

## 7. 已知问题(诚实口径,v1)

官方 GMR IK 不含碰撞回避(原作者确认),v1 未做接触后处理,存在两类已知问题,
**明天出的 v2 会在 pkl 数据层修复(格式不变)**:

1. **穿地**(脚球底低于地面,均值/最大 cm):walk 2.1/3.9、squat 3.5/7.4、turn 3.5/5.5、
   march 3.0/5.0、arm_raise 1.5/2.0、wave 1.6/2.7、stand 0.0。动态动作越剧烈越明显。
2. **单脚高度偏差**(GVHMR 单目深度对两脚的系统性偏差被重定向放大):stand 右脚恒定高
   约 1.1cm(最大 5.4cm)、squat 约 1.3cm、arm_raise 左脚约 0.7cm。

**对奖励设计的建议**:接触奖励用容差带(如脚底 |z| ≤ 2~3cm 即算接触),不要奖励精确贴 0;
v2 出来后若接触标签仍重要,先跑 v2 数据再调接触项。如果急着搭 sim 管线,v1 先跑通,
v2 无缝切换。

## 8. 注意事项

1. **插值按 sim 时间**:参考帧率 30fps,`ref_idx = int(t * 30)` 用连续时间索引并 clamp,
   不要按 sim step 整数索引(帧率不一致时参考会"抽帧"或重复,造成跳步)。
2. **四元数**:root_rot 是 xyzw,进 sim 前转 wxyz;不要对四元数做分量平均/分量滤波。
3. **速度**:需要 qvel 时从(滤波后的)qpos 用数值差分重算,不要对官方 pkl 里的角速度
   做额外滤波后混用。
4. **接触判据别用水平速度门槛**:踏步蹬地(贴地但脚水平速度快)会被误判腾空,老管线
   因此 march 成片入土 7~8cm。用脚球高度容差带(§6)。
5. **跳步排查口径**:MuJoCo/Isaac Gym 自由关节 + 执行器不可能单帧瞬移一个身宽
   (≈13m/s 级根速度),出现跳步先查 reset/qpos 直设/录屏伪影,不是参考数据问题。
6. **根自由度**:root_pos x,y 是人在世界系的移动路径;如果 sim 里想固定朝向/原地训练,
   可以对 root 做 heading 对齐后置零 x,y,但 z 与朝向务必保留。

## 9. 复现与改动

- 重跑:在 `~/g1_mocap_pipeline` 下
  `bash scripts/gvhmr_pipeline/run_gvhmr_gmr.sh data/gvhmr_input/<take>_rgb.mp4 <take> 26`
  (需 conda env `gvhmr`/`gmr`,权重在 `~/GVHMR/inputs/checkpoints/`)。
- 数据有问题先找管线负责同学,不要自己改 pkl;明天 v2 修接触,只改数据不改格式。
