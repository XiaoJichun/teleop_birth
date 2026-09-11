# Progressive Extremity Teleop（PET）架构与创新点

> 项目代号：`birth_teleop`  
> 硬件锚点：Pico 4 Ultra Enterprise + XRoboToolkit  
> 目标：**双臂先出效果 → 同一底层扩到人形全身 → 可写论文且效果优先**

---

## 1. 一句话定位

**把双臂与人形都抽象成「稀疏末端 SE(3) 目标 + 可选关键点」**，XR 传感层与机器人后端解耦；换 embodiment 只换 Mapper / Backend，不改 Pico 通信与主循环。

双臂 = 左右手（+头）两个 extremity。  
人形 = 同一协议再加骨盆/双脚（+全身关键点策略可选）。

---

## 2. 为什么这样能「不大改底层」

| 层 | 职责 | 双臂阶段 | 人形阶段（不改协议） |
|----|------|----------|----------------------|
| **XR Source** | Pico → `XRObservation` | XRoboToolkit | 同左；打开 body / ankle tracker |
| **Mapper** | `XRObservation` → `SparseTarget` | `DualArmMapper`（手柄→EE） | `HumanoidMapper`（稀疏关键点 / Extremity SE3） |
| **Controller** | clutch、滤波、速度前馈、安全 | 共用 | 共用 |
| **Backend** | `SparseTarget` → 真机/仿真 | Mock / UR / ARX… | G1 / MOSAIC / HuMiT policy… |
| **Recorder** | 同步日志 / LeRobot | 共用 schema | 字段自然变长 |

**硬约束**：上层永远不直接读「关节角 retarget 结果」作为唯一接口；主接口永远是 `SparseTarget`。

---

## 3. 效果优先的工程原则

1. **Clutch（握 grip 才跟随）**：避免绝对姿态跳变飞车。  
2. **相对位姿增量**：按下瞬间锁定参考系，松手保持机器人位姿。  
3. **低通 + 速度估计**：为后续人形速度前馈留同一接口。  
4. **视觉与动作分轨测延迟**：先把「看得见 + 跟得上」做稳，再谈论文数字。  
5. **先 Mock / MuJoCo，再真机**：协议稳定后再接驱动。

---

## 4. 论文创新点（建议叙事）

### 主创新：Progressive Extremity Teleoperation

现有工作多在「双臂 IK 栈」与「全身 retarget/策略栈」之间断裂：换人形往往重写 XR→控制链路。  
PET 主张：

1. **统一稀疏目标（Sparse Extremity Target）**  
   - 受 HuMiT「最小参考」与 ExtremControl「末端 SE(3)」启发，但不绑死某一策略网络。  
   - 双臂阶段即可用同一目标空间采数与评测，人形阶段 **零协议迁移**。

2. **渐进 embodiment 扩展**  
   - Stage-A：2 EE + head（桌面双臂）  
   - Stage-B：+ pelvis / feet（脚踝 tracker）  
   - Stage-C：可选接入 HuMiT / MOSAIC / ExtremControl 作为 Backend，XR 层不动  

3. **延迟可测、可对比的统一基准**  
   - 内置 `LatencyProbe`（动作环 / 可选光学对齐接口）  
   - 同一任务协议下对比：QP-IK 双臂 vs 稀疏全身策略，避免「论文各测各的」。

4. **（可选深化）接口残差适配**  
   - 对齐 MOSAIC：通用跟踪 + 接口特定残差；双臂采的数据可迁移到人形上肢子空间。

### 与竞品差异（写 related work 用）

| 系统 | 强项 | PET 相对差异 |
|------|------|----------------|
| XRoboToolkit | Pico 底座、双臂样例 | 我们站在其上，补「跨 embodiment 统一目标层」 |
| TWIST2 | 便携全身采数 | 延迟偏高；我们主路径强调亚百毫秒友好接口 |
| HuMiT / ExtremControl | 低延迟人形 | 我们把其思想前移到双臂阶段，形成渐进栈 |
| BEAVR | 多 embodiment + LeRobot | Quest 栈；我们锁 Pico Enterprise + 稀疏 SE3 协议 |
| Unitree xr_teleoperate | 厂商开箱 | 产品路径；我们做可研究、可替换 Backend |

**务实提醒**：论文故事建立在 **效果扎实** 上——先双臂演示与延迟数字好看，再发人形章节；创新点是「统一协议 + 渐进扩展」，不是另造一个 Pico SDK。

---

## 5. 仓库模块地图

```
birth_teleop/
  schema/          # XRObservation, SparseTarget, ButtonState
  xr/              # MockSource, XRoboSource
  mapping/         # DualArmMapper, HumanoidMapper(stub)
  control/         # Clutch, PoseFilter, LatencyProbe
  backend/         # MockDualArm, RobotBackend protocol
  pipeline/        # TeleopLoop
  record/          # SessionRecorder
scripts/           # run_dual_arm_mock.py, ...
configs/           # yaml
```

---

## 6. 实施路线（与代码同步）

| Phase | 内容 | 成功标准 |
|-------|------|----------|
| **P0** | 协议 + Mock 双臂闭环 | 键盘/假 XR 驱动双 EE，clutch 正常 |
| **P1** | 接 XRoboToolkit Source | Pico Enterprise 真机驱动 Mock/仿真 |
| **P2** | 接真实双臂 Backend | 桌面任务可稳定遥操 |
| **P3** | HumanoidMapper + 脚踝 tracker | 同一 loop 出全身 SparseTarget |
| **P4** | 插 HuMiT/MOSAIC Backend | 人形反应任务；论文实验 |

---

## 7. 硬件决策（已确认）

| 项 | 选择 | 代码落点 |
|----|------|----------|
| 双臂 | UR 双臂 **MuJoCo 仿真** | `MujocoDualURBackend` + menagerie UR5e |
| 末端 | **夹爪优先**，灵巧手兼容 | `ParallelGripper` / `DexHandStub`（`ee/`） |
| 人形 | **Unitree G1** | `G1BackendStub` → 后续接 HuMiT/MOSAIC/xr_teleoperate |

未再阻塞协议层；后续只扩 Backend / EE 插件。

---

## 8. 你需要补充的硬件信息（影响 Backend，不影响协议）

已确认：UR 双臂仿真、夹爪优先、G1。若后续有真机 UR IP / 灵巧手型号 / G1 算力单元配置，再补进对应 Backend。
