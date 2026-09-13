# Progressive Extremity Teleop（PET）架构与创新点

> 项目代号：`teleop_birth`  
> 硬件锚点：Pico 4 Ultra Enterprise + XRoboToolkit  
> 目标：**双臂先出效果 → 同一底层扩到人形全身 → 可写论文且效果优先**  
> **算法/链路逐步说明**：[遥操作全链路详解.md](./遥操作全链路详解.md)  
> **学习向深入解析（几何直觉、失败模式、例题）**：[遥操作算法深入解析.md](./遥操作算法深入解析.md)

---

## 1. 一句话定位

**把双臂与人形都抽象成「稀疏末端 SE(3) 目标 + 可选关键点」**，XR 传感层与机器人后端解耦；换 embodiment 只换 Mapper / Backend，不改 Pico 通信与主循环。

双臂 = 左右手（+头）两个 extremity。  
人形 = 同一协议再加骨盆/双脚（+全身关键点策略可选）。

---

## 2. 为什么这样能「不大改底层」

| 层 | 职责 | 双臂阶段（已实现） | 人形阶段（不改协议） |
|----|------|--------------------|----------------------|
| **XR Source** | Pico → `XRObservation` | `XRoboSource` | 同左；打开 body / ankle tracker |
| **Mapper** | `XRObservation` → `SparseTarget` | `DualArmMapper`：世界系相对 + \(R_a\) 轴对齐 | `HumanoidMapper`（稀疏关键点 / Extremity SE3） |
| **Controller** | clutch、滤波、限步、工作空间 | `RelativeClutch` / `PoseLowPass` / `LatencyProbe` | 共用 |
| **Backend** | `SparseTarget` → 真机/仿真 | `MujocoDualURBackend`（DLS IK） | G1 / MOSAIC / HuMiT policy… |
| **Recorder** | 同步日志 | JSONL `SessionRecorder` | 字段自然变长 |

**硬约束**：上层永远不直接读「关节角 retarget 结果」作为唯一接口；主接口永远是 `SparseTarget`。

当前默认控制语义（一键 Pico）：

\[
\Delta\mathbf{p}_{\mathrm{MJ}} = s\, R_a\,(\mathbf{p}_{\mathrm{XR}} - \mathbf{p}_{\mathrm{XR}}^{(0)})
,\quad
R_t = R_{\mathrm{offset}}\, R_a\, R_{\mathrm{XR}}\, R_{\mathrm{ctrl}}\, R_a^{\top}
\quad(\texttt{ori\_mode=track})
\]

详见全链路文档第 5 节与 [算法深入解析 §3.4](./遥操作算法深入解析.md)。

---

## 3. 效果优先的工程原则

1. **长按 Grip 才跟随（clutch）**：避免绝对姿态跳变飞车。  
2. **相对位姿增量 + 世界系轴对齐**：按下锁定参考；OpenXR Y‑up 映射到 MuJoCo Z‑up。  
3. **姿态 `ori_mode=world_delta`**：房间系相对旋转（默认）；可切 `track` / `position_only` 消融。  
4. **低通 + 限步 + 工作空间球**：稳、可调、可复现。  
5. **夹爪仿真加速**：Trigger\(^\gamma\) + 2F‑85 力矩/步数调参。  
6. **延迟可测**：`LatencyProbe`（loop / xr→cmd）。  
7. **先 Mock / MuJoCo，再真机**：协议稳定后再接驱动。  
8. **全 YAML / Hydra**：数值不进硬编码；一键 `./start_pico_sim.sh`。

---

## 4. 论文创新点（建议叙事）

### 主创新：Progressive Extremity Teleoperation

现有工作多在「双臂 IK 栈」与「全身 retarget/策略栈」之间断裂：换人形往往重写 XR→控制链路。  
PET 主张：

1. **统一稀疏目标（Sparse Extremity Target）**  
   - 受 HuMiT「最小参考」与 ExtremControl「末端 SE(3)」启发，但不绑死某一策略网络。  
   - 双臂阶段即可用同一目标空间采数与评测，人形阶段 **零协议迁移**。

2. **渐进 embodiment 扩展**  
   - Stage-A：2 EE + head（桌面双臂）← **当前已跑通仿真**  
   - Stage-B：+ pelvis / feet（脚踝 tracker）  
   - Stage-C：可选接入 HuMiT / MOSAIC / ExtremControl 作为 Backend，XR 层不动  

3. **延迟可测、可对比的统一基准**  
   - 内置 `LatencyProbe`  
   - 同一任务协议下对比：DLS‑IK 双臂 vs 稀疏全身策略  

4. **跟手相对映射的工程完备性**（支撑演示与消融）  
   - 世界系平移 + \(R_a\)；姿态 `world_delta` vs `track`；`position_only`；clutch / 单臂 home；夹爪 \(\gamma\)/力矩加速  

5. **（可选深化）接口残差适配**  
   - 对齐 MOSAIC：通用跟踪 + 接口特定残差  

### 与竞品差异（写 related work 用）

| 系统 | 强项 | PET 相对差异 |
|------|------|----------------|
| XRoboToolkit | Pico 底座、双臂样例 | 我们站在其上，补「跨 embodiment 统一目标层」与自研相对映射 |
| TWIST2 | 便携全身采数 | 延迟偏高；我们主路径强调亚百毫秒友好接口 |
| HuMiT / ExtremControl | 低延迟人形 | 我们把其思想前移到双臂阶段，形成渐进栈 |
| BEAVR | 多 embodiment + LeRobot | Quest 栈；我们锁 Pico Enterprise + 稀疏 SE3 协议 |
| Unitree xr_teleoperate | 厂商开箱 | 产品路径；我们做可研究、可替换 Backend |

**务实提醒**：论文故事建立在 **效果扎实** 上——先双臂演示与延迟数字好看，再发人形章节；创新点是「统一协议 + 渐进扩展 + 可复现跟手映射」，不是另造一个 Pico SDK。

### 当前效果（工程事实，非正式实验表）

| 已具备 | 尚未具备 |
|--------|----------|
| Pico → 双 UR MuJoCo 相对遥操闭环 | 正式端到端延迟/任务成功率实验数字 |
| 世界系平移 + track 姿态 + 长按 clutch | 真机 UR / G1 实机 Backend |
| 夹爪 Trigger 仿真加速跟手 | HuMiT/ExtremControl 策略接入 |
| ~150 Hz、Hydra 一键复现 | |

参考来源清单与「参考了什么 / 没照搬什么」见 [遥操作全链路详解.md §8–9](./遥操作全链路详解.md)。

---

## 5. 仓库模块地图

```
src/teleop_birth/
  schema/          # XRObservation, SparseTarget, ButtonState, SE(3) 工具
  xr/              # MockSource, XRoboSource
  mapping/         # DualArmMapper, HumanoidMapper
  control/         # RelativeClutch, PoseLowPass, LatencyProbe, R_a
  backend/         # MockDualArm, MujocoDualUR, G1Stub
  sim/             # dual_ur5e_builder（menagerie）
  ee/              # ParallelGripper, DexHandStub
  pipeline/        # TeleopLoop
  record/          # SessionRecorder
  config/          # Hydra factory
  cli/             # 交互终端
configs/           # 全部可调参数
doc/               # 本文件 + 全链路详解 + 选型调研 + Pico 指南
```

---

## 6. 实施路线（与代码同步）

| Phase | 内容 | 状态 |
|-------|------|------|
| **P0** | 协议 + Mock 双臂闭环 | ✅ |
| **P1** | XRobo Source + MuJoCo 双 UR | ✅（一键 Pico） |
| **P2** | 真实双臂 Backend | ⏳ |
| **P3** | HumanoidMapper + 脚踝 tracker | 骨架已有 |
| **P4** | 插 HuMiT/MOSAIC Backend | ⏳ |

---

## 7. 硬件决策（已确认）

| 项 | 选择 | 代码落点 |
|----|------|----------|
| 双臂 | UR 双臂 **MuJoCo 仿真** | `MujocoDualURBackend` + menagerie UR5e |
| 末端 | **夹爪优先**，灵巧手兼容 | `ParallelGripper` / `DexHandStub` |
| 人形 | **Unitree G1** | `G1BackendStub` → 后续接 HuMiT/MOSAIC/xr_teleoperate |

---

## 8. 文档导航

| 文档 | 内容 |
|------|------|
| [IK求解器切换.md](./IK求解器切换.md) | **DLS / SSIK / cuRobo** 配置切换与安装 |
| [遥操作算法深入解析.md](./遥操作算法深入解析.md) | **学习向**：SE(3)、相对/世界系、\(R_a\)、clutch、滤波、DLS IK、例题自测 |
| [遥操作全链路详解.md](./遥操作全链路详解.md) | 逐步流程、伪代码、参考仓库、创新效果表 |
| [Pico连接指南.md](./Pico连接指南.md) | 装 APK、热点、WORKING/Send、休眠、调参 FAQ |
| [pico_teleop_方案调研与选型.md](./pico_teleop_方案调研与选型.md) | 竞品对比与选型理由 |
