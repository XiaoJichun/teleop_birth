# Pico 遥操双臂 → 人形：方案调研、对比与可行选型

> 调研日期：2026-09-11（续更：锁定硬件 **Pico 4 Ultra Enterprise**）  
> 目标：Pico 头显双臂遥操、低延迟、操作顺畅，并预留扩展到人形全身遥操。  
> **当前手头硬件：Pico 4 Ultra Enterprise**（企业版能力开箱可用，选型优势明显）。

---

## 0. 针对你的硬件：Pico 4 Ultra Enterprise

你已经踩在「当前 Pico 遥操事实标准机型」上，多数开源栈都按这款测过。

| 企业版能力 | 对遥操的意义 | 谁在用 |
|------------|--------------|--------|
| **全身 Body Tracking**（WebXR / OpenXR，开箱） | 可直接出 24 关节人体骨架，无需再找企业开通 | XRoboToolkit、TWIST2、NVIDIA Isaac Teleop / GR00T-WBC |
| **VST 透视相机权限** | 头显侧相机/透视流、立体视觉闭环更易开 | XRoboToolkit 官方要求 Enterprise + VST 权限 |
| **SN / 企业 API** | 设备管理、安全区、相机开关等可走 Enterprise API | XRobo Unity Client 面板「Network - SN」仅企业版有效 |
| **官方兼容声明** | 少踩消费者版坑 | Unitree `xr_teleoperate` 明确列出 **PICO 4 Ultra Enterprise** |

**建议配件（按阶段买）：**

1. **现在双臂**：头显 + 双手柄即可（手柄 6DoF → EE）。  
2. **人形全身采数/行走**：加购 **Pico Motion Tracker ×2（脚踝）**（TWIST2 / XRobo full-body 标配；至少 2 个，多更好）。  
3. **网络**：头显与机器人 PC **同千兆有线/同 Wi‑Fi 5G 专用 AP**，避免跨网段与 WSL2 端口转发坑（能用原生 Linux 更好）。

**你的设备上「本周就能跑」的两条线：**

- **主线（推荐）**：装 `XRoboToolkit-PICO-*.apk` → PC Service → MuJoCo/真机双臂 Sample。  
- **若已有/将买 Unitree G1**：同头显可直开 `xr_teleoperate`（浏览器 + Vuer），但双臂通用与低延迟人形策略仍建议保留 XRobo 底座。

---

## 1. 结论（先看这个）

**推荐主路径：以 XRoboToolkit 作为 Pico 传感/通信底座，双臂期用其 QP-IK 遥操栈，人形期在同一输入接口上切换全身跟踪策略（优先 HuMiT / ExtremControl，备选 MOSAIC / TWIST2 / Unitree xr_teleoperate）。**

| 阶段 | 选什么 | 为什么 |
|------|--------|--------|
| **现在（双臂）** | **XRoboToolkit**（Unity Client + PC Service + Teleop-Sample-Python） | **专为 Pico 4 Ultra 打造**；立体视觉 ~82 ms；双臂样例成熟；Enterprise 全身/VST 能力可直接启用 |
| **数据采集（可选）** | **LeRobot Pico4 Teleoperator** 插件 | 直接进 LeRobot 数据集格式，利于后续 ACT/DP/VLA |
| **扩展人形（低延迟优先）** | **HuMiT** 或 **ExtremControl** | 端到端约 **50–80 ms**，明显低于 TWIST2 等 ~200 ms+ 全身 retarget 路线 |
| **扩展人形（稳妥/多接口）** | **MOSAIC（智源）** + RobotBridge | 有现成 `pico_teleop.onnx`，强调 sim2real 与长时遥操稳健 |
| **买 Unitree 直接用** | **xr_teleoperate** | **官方已测 Pico 4 Ultra Enterprise**；G1/H1/灵巧手开箱；延迟偏产品可用 |

一句话：**头显已就位 → 先把 XRobo 双臂闭环跑通；接口留适配层；人形再插 HuMiT/MOSAIC/xr_teleoperate，不要换底座。**

> **工程落地**：本仓库已启动 `teleop_birth`（Progressive Extremity Teleop）——统一 `SparseTarget` 协议，双臂与人形共用底层。见 [遥操作全链路详解.md](./遥操作全链路详解.md)（逐步算法与参考来源）、[架构与创新点_ProgressiveExtremityTeleop.md](./架构与创新点_ProgressiveExtremityTeleop.md) 与根目录 `README.md`。

---

## 2. 方案全景对比

### 2.1 总表

| 方案 | 机构/作者 | 设备 | 主要形态 | 延迟（公开） | 开源成熟度 | 适合你的阶段 |
|------|-----------|------|----------|--------------|------------|--------------|
| **XRoboToolkit** | ByteDance PICO | Pico 4 Ultra / Quest 3 | 双臂、移动臂、灵巧手；全身数据已提供但未完整验证到人形 | 立体视觉 **~82 ms**（ZED→Pico）；Pico→Pico **~100 ms** | ★★★★★ 生态最完整 | **底座必选** |
| **MirrorLimb / RealMirror** | ZTE Terminators | Pico | 上肢/双手（手柄+26-DoF 手部）+ IsaacSim 采数 | 强调实时稳定，无统一端到端表 | ★★★★ Isaac/VLA 友好 | 仿真采数、手部精细操作 |
| **LeRobot Pico4 Teleop** | 社区（HF 第三方） | Pico 4 | 双臂 TCP + 头；另有 hand 变体 | 依赖底层 SDK，社区级 | ★★★ LeRobot 插件 | 双臂数据进 LeRobot |
| **Unitree xr_teleoperate** | Unitree | Pico / Quest / AVP | G1/H1/H2/R1 + 多种灵巧手 | 未强调亚百毫秒 | ★★★★★ 官方 | 已有 Unitree 人形时最快落地 |
| **BEAVR** | MIT ARCLab | **Meta Quest**（非 Pico） | 双臂 → 人形（RX-1 等） | 零拷贝流 **≤35 ms**（宣称） | ★★★★ | 架构可借鉴；硬件若锁 Pico 则不直接用 |
| **TWIST2** | Amazon FAR 等 | **Pico 4U + 脚踝 tracker** | 全身人形数据采集 + 视运动策略 | 外部估计约 **~234 ms** | ★★★★ 2025-12 开源 | 便携全身采数；延迟非最优 |
| **ExtremControl** | UMass / Genesis | MoCap / VR（Quest+VIVE） | 全身、极端响应任务 | MoCap **~54 ms**；VR **~64 ms** | ★★★★ 在 Genesis-Humanoid | **人形低延迟首选思路** |
| **HuMiT** | humit-ral | Pico 4 Ultra 稀疏关键点 | 全身（接球等反应任务） | 通信→推理 **~20 ms**；端到端中位 **~80 ms** | ★★★ 较新 | **Pico 人形低延迟首选** |
| **MOSAIC** | 智源 BAAI | Pico / Noitom 等 | 全身跟踪+多接口残差适配 | 强调稳健长时遥操 | ★★★★ 有 ONNX | 多接口/实机稳健优先 |
| **GR00T-WBC / SONIC** | NVIDIA GEAR | Pico（经 XRoboToolkit） | G1 全身 + 规划器模式 | 有 low-latency checkpoint | ★★★★ 文档完整 | 跟 NVIDIA 栈时选用 |

> 延迟口径不统一：有的测视频回传，有的测动作端到端光学流对齐。对比时看数量级即可。

### 2.2 分层视角（选型真正该按层看）

```
┌─────────────────────────────────────────────────────────┐
│  A. XR 传感层：Pico 头/手柄/手/全身/脚踝 tracker          │
│     → 事实标准：XRoboToolkit（APK + PC Service + SDK）   │
├─────────────────────────────────────────────────────────┤
│  B. 映射/控制层（可替换）                                 │
│     双臂：QP-IK / Cartesian TCP / dex-retargeting         │
│     人形：稀疏关键点策略(HuMiT) / 末端 SE(3)(ExtremControl) │
│           / 全身 retarget(TWIST2,GMR) / 残差适配(MOSAIC) │
├─────────────────────────────────────────────────────────┤
│  C. 机器人后端：双臂驱动 / Unitree DDS / RobotBridge      │
├─────────────────────────────────────────────────────────┤
│  D. 数据与学习：LeRobot / Isaac / 自有 logger             │
└─────────────────────────────────────────────────────────┘
```

**关键决策**：A 层锁定 XRoboToolkit；B/C 按阶段换；D 按是否做模仿学习决定是否接 LeRobot。

---

## 3. 各方案要点

### 3.1 XRoboToolkit（推荐底座）

- **Paper**: [arXiv:2508.00097](https://arxiv.org/abs/2508.00097)（IEEE SII 2026）  
- **Org**: [github.com/XR-Robotics](https://github.com/XR-Robotics)  
- **核心仓库**:
  - [XRoboToolkit-Unity-Client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client)（Pico APK）
  - [XRoboToolkit-PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)（gRPC/C++）
  - [XRoboToolkit-Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python)（双 UR5 / ARX R5 / MuJoCo / Shadow Hand）
- **能力**: 头/手柄/手/全身/物体 tracker；QP-IK；立体视觉；Python/C++/ROS2 样例  
- **延迟**: 视频流 ZED→Pico **82 ms** vs Open-TeleVision **121.5 ms**  
- **生态事实**: TWIST2、NVIDIA GR00T-WBC、MOSAIC Pico 路径均依赖或兼容该栈  
- **局限**: 论文明确全身到人形的 retarget **尚未完整验证**；耦合灵巧手 retarget 有限

### 3.2 MirrorLimb（RealMirror）

- **Paper**: [arXiv:2511.08865](https://arxiv.org/abs/2511.08865)  
- **Code**: [terminators2025/RealMirror](https://github.com/terminators2025/RealMirror)  
- **特点**: WebXR 手柄 + OpenXR 26-DoF 手部；IsaacSim 轨迹录制；抖动抑制；面向 VLA 数据集  
- **定位**: 上肢精细操作与仿真采数，不是全身人形方案

### 3.3 LeRobot Pico4 Teleoperator

- **Code**: [xensedyl/lerobot-teleoperator-pico4](https://github.com/xensedyl/lerobot-teleoperator-pico4)  
- **Hand 变体**: [lerobot-teleoperator-pico4-hand](https://github.com/xensedyl/lerobot-teleoperator-pico4-hand)（dex-retargeting）  
- **特点**: `pico4` / `bi_pico4` / `bi_pico4_head`；依赖 `xensevr_pc_service_sdk`（XRobo 系）  
- **定位**: 双臂采数进 LeRobot；不替代底层 XR 服务

### 3.4 Unitree xr_teleoperate

- **Code**: [unitreerobotics/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)  
- **特点**: televuer（WebRTC）+ teleimager；支持 Pico/Quest/AVP；G1/H1/H2/R1 + Dex/Inspire/BrainCo  
- **定位**: Unitree 实机最快路径；全身/上肢产品级，延迟非论文级优化重点

### 3.5 BEAVR（MIT）

- **Paper**: [arXiv:2508.09606](https://arxiv.org/abs/2508.09606)  
- **Code**: [ARCLab-MIT/beavr-bot](https://github.com/ARCLab-MIT/beavr-bot)、[BeaVR-app](https://github.com/ARCLab-MIT/BeaVR-app)  
- **特点**: Quest；宣称 ≤35 ms；多 embodiment；直接写 LeRobot schema；think–act 异步环  
- **对本项目**: **硬件不匹配 Pico**；可借鉴其零拷贝流与录制架构

### 3.6 TWIST2

- **Paper**: [arXiv:2511.02832](https://arxiv.org/abs/2511.02832)  
- **Code**: [amazon-far/TWIST2](https://github.com/amazon-far/TWIST2)  
- **特点**: Pico 4U + 脚踝 Motion Tracker；自制 ~$250 二自由度颈；GMR 在线 retarget；15 分钟采 100 demo  
- **延迟**: ExtremControl 论文外部估计约 **~234 ms**  
- **定位**: **便携全身采数体验好**；若你更在意“操作顺畅/反应快”，不是最优

### 3.7 ExtremControl

- **Paper**: [arXiv:2602.11321](https://arxiv.org/abs/2602.11321)  
- **Site**: [extremcontrol.github.io](https://extremcontrol.github.io/)  
- **Code**: 集成于 [Genesis-Humanoid](https://github.com/UMass-Embodied-AGI/Genesis-Humanoid)  
- **核心思想**: 不走全身关节 retarget，直接控四肢/末端 SE(3) + 底层速度前馈 PD  
- **延迟**: MoCap ~54 ms；VR ~64 ms；任务含乒乓球回击/颠球等  
- **注意**: 主实验是 OptiTrack / Quest+VIVE；接口无关，可接 Pico（需自己接 XRobo 流）

### 3.8 HuMiT

- **Paper/Site**: [humit-ral.github.io](https://humit-ral.github.io/)  
- **Code**: [humit-ral/HuMiT](https://github.com/humit-ral/HuMiT)  
- **核心思想**: 策略只吃稀疏关键点（根高度、根速度、当前帧关键点），去掉昂贵 IK/未来帧缓冲  
- **延迟**: ~20 ms（通信→推理）；端到端中位 ~80 ms  
- **定位**: **Pico → 人形低延迟**最对口的一篇；适合接球等反应任务

### 3.9 MOSAIC（智源）

- **Paper**: [arXiv:2602.08594](https://arxiv.org/pdf/2602.08594)  
- **Code**: [BAAI-Humanoid/MOSAIC](https://github.com/BAAI-Humanoid/MOSAIC) + [RobotBridge](https://github.com/BAAI-Humanoid/RobotBridge)  
- **Models**: HF `BAAI-Humanoid/MOSAIC_Model`（含 **`pico_teleop.onnx`**）  
- **特点**: 通用运动跟踪 + 接口残差快速适配；G1/H1_2/Adam  
- **定位**: 实机长时遥操稳健、多接口统一；延迟宣传不如 HuMiT/ExtremControl 激进

### 3.10 NVIDIA GR00T-WholeBodyControl（附加）

- **Code**: [NVlabs/GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl)  
- **特点**: Pico 安装脚本直接装 XRoboToolkit；SMPL 全身位姿 / 3 点 VR / 规划器模式；CloudXR 路径（G1+Thor）  
- **定位**: 已投入 NVIDIA 人形栈时使用；同样印证 **XRobo 是 Pico 标准入口**

---

## 4. 延迟与“手感”怎么理解

| 链路 | 大致量级 | 影响 |
|------|----------|------|
| Pico 位姿 → PC（XRobo） | 通常 <20–30 ms 量级 | 输入新鲜度 |
| 视频回传（操作者看见） | XRobo ~82–100 ms | 眩晕与精度操作 |
| 双臂 IK + 关节伺服 | 实现相关 | 轨迹平滑度 |
| 人形全身 retarget + 策略 | TWIST2 类常 **>200 ms** | “拖泥带水”感 |
| 稀疏目标 / 末端直接控 + 速度前馈 | **50–80 ms** | 反应任务可行 |

**操作顺畅 ≠ 只有动作延迟**：立体视觉质量、死区/滤波、按键 clutch、奇异点处理同样关键。双臂阶段优先把 **视觉回传 + clutch + QP-IK 正则** 做好，比盲目换框架更有效。

---

## 5. 可行落地方案（建议按此执行）

### 5.1 总体架构

```
Pico 4 Ultra (+ 可选脚踝 Tracker)
        │  XRoboToolkit APK
        ▼
XRoboToolkit-PC-Service  (局域网 gRPC)
        │  Pybind / ZMQ / ROS2 任选一种对外发布
        ▼
┌───────────────────┬────────────────────────────┐
│ Phase-1 双臂适配器 │ Phase-2 人形适配器           │
│ QP-IK / TCP 增量   │ HuMiT 策略 或 ExtremControl │
│ (+ LeRobot 录制)   │ 或 MOSAIC pico_teleop.onnx  │
└─────────┬─────────┴──────────────┬─────────────┘
          ▼                        ▼
     你的双臂机器人            Unitree G1 / 其他人形
```

### 5.2 Phase 0：基建（1–2 天）——针对 Pico 4 Ultra Enterprise

1. **头显准备**
   - 开启开发者模式；`adb` 可用  
   - 系统建议 **OS > 5.12**（XRobo 官方测试口径）  
   - 确认 VST / 企业权限可用（Enterprise 一般开箱即有）  
2. **安装链路**
   - 下载 [XRoboToolkit-PICO APK](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases)（如 v1.1.1）并 `adb install`  
   - PC：Ubuntu 22.04/24.04 优先安装 [PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)  
   - Python：克隆 [Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python)，`setup_conda.sh`  
3. **联调验收**
   - 头显与 PC 同网段 → 打开 APK → 选 PC IP → 面板显示 **WORKING**  
   - 勾选 Head + Controller，**Send ON**，握 grip 才驱动（防飞车）  
   - 先跑 `teleop_dual_ur5e_mujoco.py` 验证手感，再接真机  
4. **内部消息约定**：`head / left_ee / right_ee / gripper / (optional body_24joints)` + 统一时间戳  
5. **可选冒烟**：Enterprise 面板可见 SN；若已有脚踝 Tracker，再开 Full-body 流验证 `is_body_data_available()`

### 5.3 Phase 1：双臂遥操（2–4 周）

| 项 | 建议 |
|----|------|
| 控制 | 基于 XRobo Sample 的 **相对位姿 clutch**（grip 按下才跟随），避免绝对坐标跳变 |
| IK | 沿用其 **QP-IK + manipulability 正则**；奇异区优先保姿态平滑 |
| 视觉 | 优先 **外置相机/ZED→Pico** 低延迟路径（~82 ms）；或机器人 egocentric 流 |
| 采数 | 需要模仿学习则挂 **LeRobot Pico4 插件**；否则自研 HDF5/MP4 同步即可 |
| 手部 | 夹爪用扳机；灵巧手再上 MirrorLimb / pico4-hand + dex-retargeting |

**成功标准**：桌面双臂整理/折叠类任务可稳定演示；操作者主观延迟可接受；数据可回放。

### 5.4 Phase 2：扩展人形（按优先级）

| 优先级 | 路线 | 何时选 |
|--------|------|--------|
| **P0** | **同一 XRobo 输入 → HuMiT** | 要 Pico 原生 + 低延迟反应（接球、快速挥臂） |
| **P0′** | **ExtremControl 思路**（末端 SE(3)+速度前馈） | 已有较强控制/Genesis 能力；可接受先用更好动捕再迁 Pico |
| **P1** | **MOSAIC + pico_teleop.onnx** | 要多接口、长时稳健、智源栈 |
| **P1′** | **TWIST2** | 要脚踝 tracker 全身采数体验、移动操作 demo |
| **P2** | **Unitree xr_teleoperate** | 采购 G1 后要最快出活，可与上面策略并存（先官方后自研） |

**迁移原则**：Phase-1 不要把业务逻辑写死在某个机器人驱动里；XR 消息与机器人命令之间留一层 `TeleopAdapter`。

### 5.5 不建议的做法

- 双臂阶段直接深度定制 BEAVR（Quest 栈），除非你们愿意换头显  
- 一上来用 TWIST2 当唯一方案却又强要求亚百毫秒手感  
- 为每个机器人重写一套 Pico APK（应始终站在 XRobo 之上）  
- 忽视视觉回传，只优化关节环——操作者仍会觉得“不跟手”

---

## 6. 推荐技术栈清单（可执行）

```text
头显:       Pico 4 Ultra Enterprise（已有）
（人形）:    Pico Motion Tracker ×2 脚踝（后续采购）
XR 底座:    XRoboToolkit Unity Client + PC Service + Pybind
双臂:       XRoboToolkit-Teleop-Sample-Python（改你的 URDF/驱动）
采数:       LeRobot + lerobot-teleoperator-pico4（可选）
人形低延迟:  HuMiT 或 ExtremControl（Genesis-Humanoid）
人形稳健:    MOSAIC + RobotBridge（pico_teleop.onnx）
人形便携采数: TWIST2
Unitree 快速: xr_teleoperate（官方已列 Pico 4 Ultra Enterprise）
参考架构:    BEAVR（流与录制）、GR00T-WBC（NVIDIA 部署）
```

### 6.1 本周可执行清单（有 Enterprise 头显时）

| Day | 动作 | 产出 |
|-----|------|------|
| D1 | 开发者模式 + 装 XRobo APK + PC Service | 头显 WORKING |
| D2 | MuJoCo 双 UR5e / ARX 仿真遥操 | 确认 clutch / 扳机夹爪手感 |
| D3 | 接你的双臂驱动（改 Sample 硬件层） | 真机或仿真 TCP 跟随 |
| D4 | 视觉回传（机载相机→头显或外置 ZED） | 主观延迟可接受 |
| D5 | （可选）挂 LeRobot 插件录 10 条 demo | 数据管线打通 |
| 之后 | 买脚踝 Tracker；评估 HuMiT / MOSAIC / xr_teleoperate | 人形 Phase-2 选型定稿 |

---

## 7. 关键链接速查

| 名称 | Paper | Code / Site |
|------|-------|-------------|
| XRoboToolkit | https://arxiv.org/abs/2508.00097 | https://github.com/XR-Robotics |
| MirrorLimb | https://arxiv.org/abs/2511.08865 | https://github.com/terminators2025/RealMirror |
| LeRobot Pico4 | — | https://github.com/xensedyl/lerobot-teleoperator-pico4 |
| Unitree XR | — | https://github.com/unitreerobotics/xr_teleoperate |
| BEAVR | https://arxiv.org/abs/2508.09606 | https://github.com/ARCLab-MIT/beavr-bot |
| TWIST2 | https://arxiv.org/abs/2511.02832 | https://github.com/amazon-far/TWIST2 |
| ExtremControl | https://arxiv.org/abs/2602.11321 | https://extremcontrol.github.io / Genesis-Humanoid |
| HuMiT | https://humit-ral.github.io | https://github.com/humit-ral/HuMiT |
| MOSAIC | https://arxiv.org/pdf/2602.08594 | https://github.com/BAAI-Humanoid/MOSAIC |
| GR00T-WBC | — | https://github.com/NVlabs/GR00T-WholeBodyControl |

---

## 8. 最终建议（决策一句话）

**你已有 Pico 4 Ultra Enterprise：直接上 XRoboToolkit 做双臂底座（这是该机型的一等公民路径）。先跑通视觉闭环 + clutch + QP-IK；人形阶段优先 HuMiT/ExtremControl 要低延迟，MOSAIC/TWIST2 要稳健采数，Unitree 则用 xr_teleoperate 开箱——全部挂在同一头显数据流上，不要换底座重来。**

### 8.1 若只选一条「现在就动手」的路径

```
Pico 4 Ultra Enterprise
  → XRoboToolkit APK + PC Service
  → Teleop-Sample-Python（双臂 QP-IK）
  → 自研 TeleopAdapter（消息层）
  → 后期：HuMiT / MOSAIC / xr_teleoperate 任选插入
```

**不要选**：为双臂单独绑 BEAVR（Quest）、或一上来只跟 TWIST2 却又要亚百毫秒反应手感。
