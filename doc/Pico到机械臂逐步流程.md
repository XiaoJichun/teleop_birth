# Pico 数据接收 → 解析 → 机械臂运动（逐步，不省略）

> 对照当前代码，不是概念草图。默认一键双 UR：`./start_pico_sim.sh`（150 Hz，世界系平移 + 腕部姿态）。  
> 物流 Arm-6A：`./start_logistics_teleop.sh`。两条路径在 **进入 Backend 之前完全共用** 同一套 Source / Mapper / Loop。

---

## 0. 总链

```
Pico App Send
  → Wi-Fi → 本机 RoboticsServiceProcess（PC Service）
  → xrobotoolkit_sdk.get_*()
  → XRoboSource.read() → XRObservation
  → TeleopLoop
  → backend.get_feedback()（当前 TCP，7D）
  → DualArmMapper.map()（clutch / 轴对齐 / 限步 / 工作空间 / 低通 / 夹爪 γ）
  → SparseTarget
  → 掉线则冻结左右 EE 的 active
  → Backend.apply()
      双 UR：home → DLS IK 写 qpos+ctrl → ParallelGripper → mj_step
      物流：home+释放粘性 → 未 clutch 用 hold 位姿 → 本地 DLS → 夹爪 qpos → mj_step → sticky → 防穿模 → 可选 TACTO
  → BackendState 下一拍当反馈
  → 可选 JSONL SessionRecorder、LatencyProbe、sleep 对齐 hz
```

位姿总线始终是 **7 维 `xyz + quat(xyzw)`**。关节角只存在于 Backend / IK 内部。

---

## 1. Pico 端与本机服务（循环开始之前）

### 1.1 头显侧

1. 头显与 PC 同一网段（脚本会打印 `hostname -I`；热点优先 `10.42.0.1`）。
2. 打开 App **XRoboToolkit**。
3. 设备列表选本机 IP。状态变成 **WORKING**。
4. 打开 **Send**。此后 PC Service 才持续有新的时间戳和位姿。
5. **长按该侧 Grip** 才开始跟手（双 UR 一键 `hold_s=0.2`，物流 YAML 也是 `0.2`；Mapper 类默认 `0.35`，以实际 YAML 为准）。
6. **Trigger** = 夹爪闭合量。左右互不影响。
7. 左 **Y 或 Menu** / 右 **B 或 Menu** 上升沿：该侧回零。
8. 摇杆轴、摇杆按下、A/X 键会被 SDK 读进来，但 **DualArmMapper 不用它们**。

### 1.2 本机启动（`start_pico_sim.sh` / `start_logistics_teleop.sh` 相同前半）

1. `uv sync`（物流再加 `--extra tacto --extra collect`）。
2. `bash scripts/bootstrap_third_party.sh`（menagerie；物流还要 tacto）。
3. 若 `import xrobotoolkit_sdk` 失败，跑 `scripts/xrobo/install_xrobo.sh`（解压运行时，无 sudo）。
4. 从 `configs/setup/xrobo.yaml` 读路径，设置：
   - `LD_LIBRARY_PATH` = `.venv` 里的 xrobo 库 + PC Service 目录 + `lib` + `SDK/x64`
   - `LC_ALL=C.UTF-8`
5. `scripts/xrobo/ensure_xrobo_service.sh`：若没有 `RoboticsServiceProcess`（`/proc` 名可能截成 `RoboticsService`），就拉起进程。这是 Pico Wi-Fi 流的接收端。
6. 双 UR：`exec uv run python run_teleop.py` + `one_click_pico_sim.overrides`。  
   物流：`exec uv run python scripts/robots/artixon_teleop.py`（不走 Hydra `config.yaml`，读 `configs/robots/artixon_teleop.yaml`）。

### 1.3 Python 再进一次库路径

`run_teleop.py` 若 `source.type=xrobo`，会再调 `apply_xrobo_library_path()` 并 `check_xrobo_sdk_available()`。失败直接退出，提示 `./start_pico_sim.sh`。

`XRoboSource.start()` 同样：设库路径 → import SDK → 调 YAML 里的 `api.init`（默认 `"init"`）。`stop()` 调 `api.close`。

---

## 2. 每一帧：SDK 原始量如何变成 `XRObservation`

实现：`teleop_birth/teleop/xr/sources.py` · `XRoboSource.read()`。  
函数名全部来自 `configs/setup/xrobo.yaml` 的 `api:`，缺函数则该字段用零姿态 / 0，不抛异常。

| YAML 键 | 默认 SDK 函数 | 写入 |
|---------|---------------|------|
| `timestamp_ns` | `get_time_stamp_ns` | `XRObservation.timestamp_ns`；没有函数则用本机 `time.time_ns()` |
| `head_pose` | `get_headset_pose` | `head` |
| `left_controller_pose` | `get_left_controller_pose` | `left_controller` |
| `right_controller_pose` | `get_right_controller_pose` | `right_controller` |
| `left_grip` / `right_grip` | `get_left_grip` / `get_right_grip` | `buttons.left_grip` / `right_grip`，float |
| `left_trigger` / `right_trigger` | `get_left_trigger` / `get_right_trigger` | Trigger，float，约定 0..1 |
| `button_a/b/x/y` | `get_A_button` … `get_Y_button` | bool |
| `left_menu` / `right_menu` | `get_left_menu_button` / `get_right_menu_button` | bool |
| `left_axis` / `right_axis` | `get_left_axis` / `get_right_axis` | `(x, y)`，不够 2 维则 `(0, 0)` |
| `left_axis_click` / `right_axis_click` | 同名 | bool |
| `body_available` + `body_poses` | `is_body_data_available` / `get_body_joints_pose` | **仅** `include_body=true` 时读；一维数组按 7 列 reshape。默认 `include_body=false`，一键 Pico **不读全身** |

位姿 `np.asarray(..., dtype=float64)`，本仓库按 **`[x,y,z, qx,qy,qz,qw]`** 使用。坐标系是 OpenXR：**Y 朝上、−Z 朝前**。MuJoCo 是 **Z 朝上**，对齐发生在 Mapper，不在 SDK 这一步。

`left_hand` / `right_hand` / `trackers` 在 XRobo 这一路上 **不填**。

### 2.1 掉线判定（不用位姿）

`_update_link_watch`：

- SDK 时间戳相对上一帧 **变化** → `stream_ok=True`。手不动但时间戳在走，不算掉线。
- 时间戳不变且持续 `stale_s`（默认 5 s）→ `stream_ok=False`，打印一次。
- `reconnect.enabled` 默认 **false**。若打开：冷却 `cooldown_s`（默认 8 s）后 `close` + 睡 0.4 s + 再 `init`，成功则本帧 `reconnected=True`（一次性，下一帧清掉）。

`valid`：循环未 start 或 SDK 空时为 False，其余读到的帧为 True。`valid=False` 时主循环 **不 map、不 apply**，只睡一个 `dt`。

---

## 3. 主循环（每拍，双 UR 默认 150 Hz）

`TeleopLoop.run`（`teleop/pipeline/loop.py`）：

1. `source.start()`、`backend.connect()`（建模型、回零、开 viewer）、`mapper.reset()`（清空 clutch / 低通 / 上一帧目标）。
2. 循环：
   1. 超时或步数到则停。
   2. `obs = source.read()`。`not obs.valid` → sleep `dt`，continue。若 `runtime.plot.enabled`（物流是 `plot.enabled`，默认都关）：记下本帧左右手柄 xyz，画「Pico 原始」曲线。
   3. `obs.reconnected` → `mapper.reset()`，打印「请重新长按 Grip」。
   4. `feedback = backend.get_feedback()`：左右 TCP 当前世界系 7D。
   5. `target = mapper.map(obs, feedback)`。
   6. 若 `stream_ok=False`：把 `left_ee` / `right_ee` 的 `active` 设为 False。目标位姿仍在，但 Backend **不推进**这两臂（双 UR 因此本拍不调 IK；物流改用 hold 位姿锁住）。
   7. `state = backend.apply(target)`。进入 `ik.solve` 之前，若开了 `plot.pre_ik`，记下即将求解的位姿（物流未 clutch 时是 hold 位姿，图上为点线）。绘图在子进程，不占控制环。
   8. `LatencyProbe.tick(obs.timestamp_ns)`：
      - `loop_ms`：本拍相对上一拍的墙钟间隔。
      - `xr_to_cmd_ms`：本机 `time.time_ns()` 减 SDK 时间戳，下限 0。窗口内记 mean / p50 / p95。
   9. 若开了 `runtime.log_enabled`：`SessionRecorder` 写 JSONL（观测、目标、反馈、延迟）。
   10. `on_step` 回调（打印等）。
   11. 若本拍耗时 `< 1/hz`，sleep 补齐。
3. `finally`：`backend.disconnect()`、`source.stop()`、关 recorder。

---

## 4. 解析与映射：`DualArmMapper.map`

实现：`teleop/mapping/mappers.py`。左右臂独立。一键数值见 `configs/setup/xrobo.yaml` → `one_click_pico_sim.overrides`。

### 4.1 回零键（先于 clutch）

- 左：`y` 或 `left_menu`；右：`b` 或 `right_menu`。
- **上升沿**（本帧按下且上帧未按）→ `home_left` / `home_right`。
- 该侧：`clutch.reset` + `lpf.reset`；本帧 `active=False`，extremity 保持 `_last`，**不跟手柄**。`meta.home_*` 交给 Backend 去写关节。
- 按住不放不会每帧重复回零。

### 4.2 每侧 clutch

输入：该侧手柄 7D、Grip、Trigger、机器人反馈 TCP（没有反馈就用 `_last`）。

第一帧若 `_last` 平移近似 0，先用反馈 TCP 填 `_last`，避免限步从原点拉一段。

`RelativeClutch.update`（`teleop/control/filters.py`）：

**是否想跟随 `_want`**

| `clutch_mode` | 条件 |
|---------------|------|
| `always` / `on` / `continuous` | 永远跟随 |
| `grip_or_trigger` / `any` | `max(grip, trigger) ≥ grip_threshold` |
| `long_press_grip`（一键默认） | Grip ≥ 阈值（默认 **0.45**）且连续按住 ≥ `hold_s`（一键 **0.2 s**）。松开把计时清零 |

松开（`want=False`）：`engaged=False`，参考点清空，返回 `(False, None)`。Mapper 于是 `active=False`，输出上一帧 `_last`（冻结）。

刚接合（`want` 且尚未 engaged）：锁定

- `xr_ref` = 当前手柄位姿
- `robot_ref` = 当前机器人 TCP
- 接合只锁 `xr_ref` 和 `robot_ref`。按下那一帧目标等于当前 TCP，不会瞬移。

已接合则算相对目标，见下一节。`scale` **只乘平移**，不缩放旋转。

### 4.3 世界系平移

\[
\Delta\mathbf{p}_{\mathrm{MJ}} = s\, R_a\,(\mathbf{p}_{\mathrm{XR}} - \mathbf{p}_{\mathrm{XR}}^{(0)})
,\quad
\mathbf{p}_t = \mathbf{p}_{r0} + \Delta\mathbf{p}_{\mathrm{MJ}}
\]

\(R_a\) = `openxr_yup_to_zup`：

| OpenXR | MuJoCo |
|--------|--------|
| +X 右 | +X 右 |
| +Y 上 | +Z 上 |
| −Z 前 | +Y 前（朝桌面） |

也可在 YAML 里给完整 `xr_align_matrix`；`identity` 则不改轴。

### 4.4 姿态（一键 `position_only=false`）

先可选乘上手柄本地修正 \(R_{\mathrm{ctrl}}\)（`controller_quat_xyzw`，默认单位阵）。竖直转和拧腕分开：

\[
R_{\Delta\mathrm{MJ}} = R_a\,(R_x R_{x0}^{\top})\, R_a^{\top}
,\quad
\psi = \operatorname{atan2}(R_{\Delta\mathrm{MJ},10},\, R_{\Delta\mathrm{MJ},00})
\]

\[
\mathbf{w} = \mathrm{axis\text{-}angle}(R_{x0}^{\top} R_x),\quad w_y\leftarrow 0
,\quad
R_t = \mathrm{Rot}_z(\psi)\, R_{r0}\, \mathrm{Rot}(M\mathbf{w})
\]

\(M\) 把握把右/上/后送到夹爪右、前、接近轴的反向，所以绕瞄准方向拧 = 绕朝下的接近轴拧。房间左右转只走 \(\psi\)。

`position_only=true` 时 \(R_t = R_{\mathrm{r}0}\)，只跟位置。公式细节见 [遥操作全链路详解.md](./遥操作全链路详解.md)：**§5.2** \(R_a\)、**§5.3** \(R_t\)、**§5.4** \(M\)。

### 4.5 接合后还要过三道稳妥（顺序固定）

1. **限步 `_limit_step`**  
   - 平移：相对上一帧 `_last`，位移长度 > `max_step_m`（一键 **0.20 m**）则截到该长度。  
   - 姿态：\(R_{\mathrm{err}}=R_{\mathrm{des}} R_{\mathrm{prev}}^{\top}\)，角 \(\arccos((\mathrm{tr}-1)/2)\)。大于 `max_step_rad`（一键 **0.35 rad**）则沿轴角只走 `max_step_rad`（Rodrigues）。
2. **工作空间球 `_clamp_workspace`**  
   中心 `workspace_center.left_ee/right_ee`，半径一键 **1.30 m**。超出则拉回球面。半径 ≤0 则不限制。物流 YAML 把中心放在工位 TCP 附近。
3. **低通 `PoseLowPass`**  
   \(\alpha=\) `filter_alpha`（一键 **0.92**，越大越跟手）。  
   平移：\((1-\alpha)\mathrm{prev}+\alpha\,\mathrm{new}\)。  
   四元数：同样线性混合；若与上一帧点积 <0 先翻符号再混，然后归一化（不是严格 slerp）。

之后写入 `extremities[left_ee|right_ee]`，`active=True`，并更新 `_last`。未接合则 `active=False` 且位姿 = `_last`。

### 4.6 头

`map_head=false`（物流 teleop YAML 显式关闭；双 UR 一键 overrides 未改 mapper 该项，以 `configs/algorithm/mapper/dual_arm.yaml` 为准）。为 true 时头只做低通，不做 clutch，并 `active[head]=True`。双 UR Backend **不消费 head**。

### 4.7 夹爪命令（与 clutch 无关）

每帧都算，松开 Grip 也会更新 Trigger：

\[
\mathrm{openness} = \mathrm{clip}(\mathrm{trigger}, 0, 1)^{\gamma}
\]

一键 \(\gamma=0.55\)。\(\gamma<1\) 时中段更灵敏，仿真里闭合看起来更快。0=开，1=合。

### 4.8 输出 `SparseTarget`

- `timestamp_ns` 沿用 SDK 时间戳  
- `extremities` / `active`  
- `left_gripper` / `right_gripper`  
- `meta`：`scale`、`clutch_mode`、`position_only`、`home_left`、`home_right`、`xr_grip_l/r`

`HumanoidMapper` 会在此之上填骨盆/脚踝；一键 Pico **不用**。

---

## 5. 双 UR：目标位姿怎么变成关节运动

`MujocoDualURBackend`（`robot/sim/mujoco_dual_ur.py`）。场景由 `robot/arm/ur5e/builder.py` 拼 menagerie 的 UR5e + Robotiq 2F-85。

### 5.1 `connect`（循环前一次）

1. 编译模型，建 `MjData`。
2. 解析左右臂 actuator、夹爪 actuator、TCP site、法兰 site、基座 body。
3. 放大夹爪力矩/增益：`force_scale=16`，`kp_scale=1.8`，`kv_scale=0.45`（让 Trigger 在仿真里跟得上）。
4. 关节 `qpos` 和 `ctrl` 写到 `left_home` / `right_home`，夹爪写 `ctrl_open`。
5. `settle_steps` 次 `mj_step`。
6. 记录法兰到 TCP 的固定变换 `T_flange_tcp`（解析 IK 用；DLS 直接对 TCP site）。
7. `build_arm_ik_solver`：一键 `ik=dls`。
8. `ParallelGripper.reset()` = 张开。
9. `runtime.render=true` 时 `launch_passive` viewer。

### 5.2 `get_feedback`

`mj_forward`，读左右 TCP site 的 `site_xpos` + `site_xmat`，转成 7D。这是下一拍 clutch 的 `robot_ref` 来源。

### 5.3 `apply` 顺序

1. **回零**（若 `meta.home_*`）：该侧 6 个臂关节 `qpos` 与 `ctrl` 写成 home，夹爪 `ctrl_open`，`mj_forward` 后再 `mj_step` 30 次。本帧该侧 `active=False`，所以下面不会再对它做 IK。
2. **仅 `is_active` 的臂** 调 `ik.solve(handles, pose_des, position_only=ik.position_only)`。未 clutch 的臂本拍不改臂 `ctrl`（执行器会保持上一拍 ctrl；不等于每拍重写 hold）。
3. 左右夹爪 **无论是否 clutch** 都 `ParallelGripper.apply`：  
   `ctrl = ctrl_open + openness * (ctrl_closed - ctrl_open)`，写入夹爪 actuator。
4. `mj_step` 次数 = `backend.sim_substeps`（一键 **4**）+ `end_effector.extra_sim_steps`（一键 **6**）= **10** 次物理步。
5. viewer 每 `viewer_sync_every`（一键 1）拍 `sync()`。
6. 再 `get_feedback()` 返回 `BackendState`。

### 5.4 DLS IK（一拍内，默认求解器）

`teleop/ik/dls.py`，对 **TCP site** 做阻尼最小二乘，不是法兰。一键：`iters=20`，`rot_weight=0.85`，`max_dq=0.28`，`position_only=false`。YAML 里还有 `damping`、`pos_weight`、`err_tol`（代码默认 damping 0.05、err_tol 8e-4，若 YAML 未覆盖就用默认）。

每一迭代：

1. `mj_forward`。
2. 误差 6 维：平移 `p_des - p`，旋转 \(0.5\,(R_{\mathrm{des}} R^{\top})\) 的反对称向量（`mat_to_6err`）。
3. 平移乘 `pos_weight`，旋转乘 `rot_weight`（`position_only` 时旋转权 0，任务降到 3 维）。
4. \(\lVert e\rVert < \mathrm{err\_tol}\) 则停。
5. `mj_jacSite` 取 TCP 位置/旋转雅可比，只留该臂 6 个自由度。
6. \(\Delta q = J^{\top}(JJ^{\top} + \lambda I)^{-1} e\)，再 clip 到 \(\pm\mathrm{max\_dq}\)。
7. `qpos` 与对应 actuator `ctrl` 写成新角，并夹在 `actuator_ctrlrange` 内。

DLS 把期望位姿直接写进仿真状态，随后 `mj_step` 让位置执行器继续跟踪这些 `ctrl`。

`ik=ssik`：解析 UR 解，把 TCP 世界系目标换成基座系法兰目标（`tcp_des_to_flange_in_base`），失败可 fallback DLS。`ik=curobo` 走 GPU，同样可 fallback。一键不走这两条。

---

## 6. 物流 Arm-6A：同一 `SparseTarget`，Backend 不同

`LogisticsTeleopBackend.apply`（`robot/arm/artixon/teleop_backend.py`）。Source / Mapper / Loop 与双 UR 相同；装配在 `scripts/robots/artixon_teleop.py`。

### 6.1 与双 UR 的差异

| 点 | 双 UR | 物流 |
|----|-------|------|
| IK | 共享 `teleop/ik` | `robot/arm/artixon/ik_dls.py` |
| 未 clutch | 本拍不调该臂 IK | 把该侧目标换成 `_hold_pose`（上次成功的目标或回零后的 TCP），每拍都 `step_to_poses`，避免仿真漂走 |
| 回零 | home + 张爪 + 30 step | 写 `home_q`、张爪、`mj_forward`，并 **强制释放** 该侧粘住的包裹 |
| 夹爪 | 2F-85 actuator ctrl | 闭合量 0..1 线性映射左指 `0.04875→0.006` m，右指相反；**ctrl 和 qpos 都写** |
| 物理步 | substeps+extra（一键 10） | `sim_substeps`（teleop YAML 默认 4）；每一步后再锁一次夹爪，防止桌面把指撑开 |
| 物体 | 双 UR 遥操主路径不粘物体 | sticky + 防穿模 + 可选 TACTO |

### 6.2 本地 DLS 之后

1. `step_to_poses` 对左右 TCP 各解一次 IK（目标可能是 mapper 输出或 hold）。
2. `_update_sticky`：  
   - 该侧已经抓着某盒 → 继续 `update`。  
   - 否则仅当 `gripper ≥ sticky_gripper_min`（teleop YAML **0.25**）才考虑吸附。  
   - 在未占用包裹里选指垫中心最近者；距离 ≤ `sticky_attach_radius_m`（YAML **0.09**）才调用该盒的 `StickyGrasp.update`。  
   - `StickyGrasp` 还要求 3D 距离和 XY 距离都 ≤ 自己的 `attach_radius_m`（构造默认 0.08，若 teleop 后端传入配置则以传入为准），且 gripper ≥ 其 `attach_gripper_min`。满足则记下抓取瞬间四元数，之后每帧把物体钉在左右指连杆中点，并 `anti.on_grasp`（关碰撞等）。Trigger 松开则 `on_release`。
3. `AntiPenetrationManager.step()`：支撑面 / 最小间隙，防止盒子穿桌穿爪。
4. TACTO：`tacto.enabled` 时四指垫 DIGIT 渲染；失败只打印，不中断遥操。接触数放进 `BackendState.extra["tacto_hits"]`。
5. 返回左右 TCP 反馈，供下一拍 clutch。

---

## 7. 一拍时序（双 UR 一键，数字是当前 YAML）

```
t = 1/150 s 预算（控制环；Pico 流通常 ~100 Hz，见下）
  Pico Send 已在 PC Service 缓冲（头显侧约 90–100 Hz 更新）
  get_time_stamp_ns + 头/左右手柄位姿 + Grip/Trigger/按键
  （本拍可能读到与上一拍相同的时间戳 = 零阶保持，不是插值上采样）
  时间戳变了？否则累计；≥5s 不变 → stream_ok=false（不自动重连）
  读左右 TCP（mj_forward）
  Grip≥0.45 且已按满 0.2s？
    否：该侧 active=false，目标=上一帧，IK 跳过
    是：锁参考（刚接合）或
        Δp = 2.0 * R_a * (p_xr - p_xr0)
        R = Rot_z(ψ) @ R_robot0 @ Rot(M w)   # ψ 是房间 yaw；w 是去掉绕「上」后的拧腕
        平移限 0.20m、转角限 0.35rad、工作空间半径 1.30m
        低通 α=0.92
  left/right_gripper = trigger^0.55
  stream_ok=false → 强制 active=false
  home 上升沿 → 该侧关节回 home、张爪、30 步
  active 臂：DLS 最多 20 次，写 6 关节 qpos+ctrl
  夹爪 ctrl = open + g*(closed-open)
  mj_step × (4+6)
  viewer.sync
  记延迟；若拍内还有剩余时间则 sleep
```

**Pico ~100 Hz vs 控制环 150 Hz**：两套时钟。控制环每拍轮询 SDK 缓冲（`XRoboSource.read`），不阻塞等新帧；缓冲未更新则重复同一 pose（零阶保持）。详见 `doc/遥操作全链路详解.md` §3「Pico ≈100 Hz 发、控制环 150 Hz」。

物流把最后「active 才 IK」换成「不 active 也用 hold 位姿 IK」，并在 `mj_step` 后做粘性抓取和防穿模。
