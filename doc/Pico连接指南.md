# Pico 仿真遥操（一键）

> 算法与数据流： [遥操作全链路详解.md](./遥操作全链路详解.md)  
> **深入理解算法（推荐学习）**： [遥操作算法深入解析.md](./遥操作算法深入解析.md)

## 你只需要

**PC 端：**

```bash
cd /root/workspace/teleop_birth
./start_pico_sim.sh
```

或 `./start.sh` → 选 **「一键：Pico 仿真遥操（推荐）」**

**Pico 端（App 已装好时）：**

1. 打开 **XRoboToolkit**，按下方「进 App 后界面怎么点」操作  
2. **Status** 变为 **WORKING**，且 **Send** 已打开  
3. **长按该侧 Grip ≥ hold_s**（一键默认约 **0.2 s**）开始遥操（松开关闭；左右独立）  
4. **回零**：左手 **Y** 或 **Menu**；右手 **B** 或 **Menu**（该侧手臂回 home）  
5. **Trigger** = 夹爪（已仿真加速；扣扳机应较快开合）  
6. **拧腕** = 末端姿态跟随（`ori_mode=world_delta`；松 Grip 保持）

首次 PC 侧会自动：解压 PC Service → 装 adb → 编译 SDK（uv）→ 启动服务 → 开仿真。

### 进 App 后界面怎么点

先保证 PC 已跑 `./start_pico_sim.sh`（或 PC Service 已在跑），Pico 已连电脑热点/同一 Wi‑Fi。

| 顺序 | 界面区域 | 你要做的 |
|------|----------|----------|
| ① | 弹出的服务器列表，或 **Network → PC Service** | 点选电脑 IP（热点优先 **`10.42.0.1`**）。没有列表则点 **Enter** 手动输入该 IP，再连接 |
| ② | **Network → Status** | 应变为 **WORKING**。若还是断开，点 **Reconnect** |
| ③ | **Tracking** | 打开 **Head**、**Controller**（双臂遥操必需）。手势/全身先不用开 |
| ④ | **Data & Control → Send** | **必须打开**（最容易漏）。关掉则已连接但电脑收不到手柄数据 |
| ⑤ | 手柄 | **长按 Grip** 遥操该臂（松开停）；**Y/Menu（左）或 B/Menu（右）** 回零；Trigger 夹爪 |

可选：打开 **Switch w/ A Button** 后，可用右手柄 **A 键** 快速开关 Send。

双臂仿真阶段**不用选**：Motion Tracker / Full body、Hand tracking、ZED/相机 Listen 等。

核对清单：**WORKING + Head/Controller 开 + Send 开 + 长按 Grip 后手臂才动**。  
终端里 `xrL` 坐标应随手柄变化；`active` 仅在长按 Grip 后为 True。

---

## 首次：安装 XRoboToolkit APK（必做一次）

**XRoboToolkit 不是 Pico 自带应用**，要先装仓库下载的那个 APK，装好后才会出现在应用列表里。

### 还没装过（第一次）

1. Pico 打开 **开发者模式**，用 **USB** 连电脑  
2. 电脑执行：

```bash
cd /root/workspace/teleop_birth

# 若还没有 adb / APK，先跑安装脚本（会装 platform-tools 并下载 APK）
bash tools/scripts/install_xrobo.sh

tools/bin/adb devices
# 应能看到设备，例如：
#   PA98xxxxxxxxxx    device

tools/bin/adb install -r third_party/xrobo_downloads/XRoboToolkit-PICO-1.1.1.apk
# 成功应显示 Success
```

说明：

- 系统若提示 `adb: command not found`，**不要**依赖系统 apt；用项目内 `tools/bin/adb`（由 `install_xrobo.sh` 安装）。
- 若提示没有 APK 文件，先跑一次 `bash tools/scripts/install_xrobo.sh`，再执行上面的 `adb install`。
- 头显 USB 已连接时，`install_xrobo.sh` 也会尝试自动 `adb install`。

### 装好后在头显里找

1. 戴上 Pico，进 **应用库 / 未知来源应用**（企业版有时在「商店以外」或「已安装应用」）  
2. 找 **XRoboToolkit**（包名 `com.xrobotoolkit.client`）并打开  
3. 选电脑 IP → WORKING → Send → 移动手柄  

找不到图标时：确认允许显示未知来源应用；再执行一次 `tools/bin/adb install -r ...`；可用 `tools/bin/adb shell pm list packages | grep -i xrobo` 确认是否已安装。

---

## Pico 与电脑如何连接

遥操数据走 **局域网（PC Service ↔ XRoboToolkit）**；装 APK / 调试可用 **USB adb** 或 **无线 adb**。公司网常有设备隔离，**优先用电脑开热点**。

### 方式 A：电脑热点（推荐，尤其公司网）

**1. Ubuntu 开热点**

1. 右上角菜单 → **Wi‑Fi**  
2. 点右上角三个点 **`…`** → **Turn On Wi‑Fi Hotspot**  
3. 设置热点名称（SSID）、密码（≥8 位），点开启  

开启后，Ubuntu 热点默认网段一般是 **`10.42.0.0/24`**，电脑热点 IP：

**`10.42.0.1`**

**2. Pico 连上这个热点**

- 头显 Wi‑Fi 设置里只连该热点（不要再连公司访客网）  
- DHCP 会自动给 Pico 分配 IP（常见如 `10.42.0.x`）

**3. 遥操连接（XRoboToolkit）**

- PC 跑 `./start_pico_sim.sh`  
- Pico 打开 XRoboToolkit → 选 / 输入 **`10.42.0.1`** → WORKING → Send  

**4.（可选）无线 adb**

若已在头显开启无线调试 / 曾用 USB 开过 `adb tcpip 5555`，Pico 连上热点后可用：

```bash
# 把 <PICO_IP> 换成头显在热点下的地址（不是 10.42.0.1）
tools/bin/adb connect <PICO_IP>:5555
tools/bin/adb devices
```

装 APK 示例：

```bash
tools/bin/adb install -r third_party/xrobo_downloads/XRoboToolkit-PICO-1.1.1.apk
```

说明：

| 地址 | 用途 |
|------|------|
| **`10.42.0.1`** | 电脑热点网关 → XRoboToolkit 里选这个连 PC Service |
| **`10.42.0.x`（Pico 自己的 IP）** | 仅用于 `adb connect pico的ip:5555` |

查 Pico 热点 IP：头显网络详情，或电脑上 `arp -n | grep 10.42.0` / 路由器客户端列表。

### 方式 B：同一路由器 Wi‑Fi

1. 电脑与 Pico 连**同一 SSID**（勿用访客网）  
2. PC 跑一键脚本，看终端打印的本机 IP  
3. XRoboToolkit 选该 IP  

若一直连不上 WORKING，多半是隔离，改回 **方式 A**。

### 方式 C：USB（首次装 APK 最稳）

见上文「首次：安装 XRoboToolkit APK」：开发者模式 + USB + `tools/bin/adb install`。

---

## 关闭 Pico 休眠（强烈建议）

**要关。** 企业版里，**「自动休眠 / 息屏」不在普通电源、显示菜单**，所以你会找不到。

### 正确入口：企业设置（Pico 4 Ultra Enterprise）

按这个路径进（官方企业设置路径）：

1. **设置 → 通用 → 关于本机**  
2. 连续点击 **「软件版本」约 10 次** → 打开 **开发者选项**  
3. 进入 **开发者选项 → 企业设置 / Business Settings / 行业设置 → 特殊设置**  

然后改下面几项（名称可能是中/英）：

| 位置 | 选项 | 设成 |
|------|------|------|
| **基础设置 → 特殊设置** | **自动休眠模式 Auto Sleep Mode** | **关闭** |
| **基础设置 → 网络设置** | **灭屏后保持网络连接** | **打开**（很重要） |
| **系统设置 → 电源策略 Power Policy** | **系统休眠** | **永不 Never**（先改这个） |
| **系统设置 → 电源策略 Power Policy** | **灭屏 / Screen Off** | **永不 Never**（后改这个） |
| **基础设置**（若有） | **充电时亮屏 Screen On when Charging** | **打开** | 

注意：电源策略里要先把「系统休眠」设成永不，再设「灭屏」，否则可能改不动。

### 若仍找不到企业设置

- 确认头显是 **Enterprise / 企业版**（不是消费版 Ultra；消费版往往**关不掉**自动休眠）  
- 或用电脑 **PICO 企业套件 / MDM** 下发同样策略  
- USB 连上后跑保活脚本（见下）

### 一键 adb 保活（配合 USB）

```bash
cd /root/workspace/teleop_birth
bash tools/scripts/pico_keep_awake.sh
```

遥操时尽量 **插着 USB 充电**，XRoboToolkit 前台 + **Send 开**，少长时间摘下头显。

---

## 依赖怎么管

| 依赖 | 管理方式 |
|------|----------|
| Python / `xrobotoolkit_sdk` | **uv**（`.venv`） |
| PC Service、adb、APK | **`tools/scripts/install_xrobo.sh`**（路径写在 `configs/setup/xrobo.yaml`） |

adb **不能**进 uv，由安装脚本下载 Google `platform-tools` 到 `third_party/platform-tools/`，并在 `tools/bin/adb` 提供入口。

---

## 跟随 / 流畅度参数

| 项 | 默认（一键 Pico） | 说明 |
|----|-------------------|------|
| `mapper.clutch_mode` | `long_press_grip` | 长按 Grip 才遥操；松开停止 |
| `mapper.hold_s` | `0.2` | 长按判定时间（秒） |
| `mapper.reset_buttons_*` | Y/Menu、B/Menu | 左右手柄各自回零 |
| `mapper.delta_frame` | `world` | 相对增量叠在世界系（更跟手） |
| `mapper.xr_align` | `openxr_yup_to_zup` | OpenXR→MuJoCo 轴对齐 |
| `mapper.position_only` | `false` | 跟位置+姿态（`ori_mode=world_delta`） |
| `mapper.ori_mode` | `world_delta` | 房间系相对旋转；TCP 朝下时 yaw 不反号 |
| `mapper.gripper_gamma` | `0.55` | Trigger 曲线，越小中段越灵敏 |
| `mapper.max_step_m` | `0.08` | 每周期最大位移 |
| `runtime.scale` | `1.0` | 位移倍率（1:1） |
| `runtime.hz` | `150` | 控制频率 |
| `end_effector.force_scale` | `16` | 夹爪仿真力矩放大（加速闭合） |

配置入口：`configs/setup/xrobo.yaml`、`configs/algorithm/mapper/dual_arm.yaml`。

---

## 常见问题

| 现象 | 处理 |
|------|------|
| `sudo: command not found` | 已本地解压 deb，无需 sudo；重跑 `./start_pico_sim.sh` |
| `adb: command not found` | 跑 `bash tools/scripts/install_xrobo.sh`，使用 `tools/bin/adb` |
| 应用列表没有 XRoboToolkit | 按上文「首次：安装 APK」用 USB + `adb install` |
| 头显看不到 IP / 连不上 | 用「Pico 与电脑如何连接」方式 A 开热点；XRobo 选 `10.42.0.1` |
| 无线 adb 连不上 | Pico 已连热点后用 `adb connect <PICO_IP>:5555`（不是 `10.42.0.1`）；或改用 USB |
| WORKING 但臂不动 / `xrL` 不变 | **打开 Send**；确认 Head/Controller Tracking |
| `active=False` 臂不动 | **长按 Grip ≥ hold_s**（一键默认约 0.2 s）并保持 |
| 需要回零 | 左手 **Y/Menu**，右手 **B/Menu** |
| Pico 容易掉线 | 关休眠；保持 Send；**手静止不会再误重连**。若仍断：检查热点信号，头显里 Reconnect |
| 终端狂刷「重连 SDK」 | 已默认关闭误判重连；请更新代码后重跑 `./start_pico_sim.sh` |
| 臂动了但方向不像手 | 确认已重启加载 `delta_frame=world`；先只测平移。前→桌深(+Y)、上→+Z、右→+X。某轴反了告诉我 |
| 腕部乱拧 / 不像在跟手 | 确认 `ori_mode=world_delta`；可暂设 `position_only=true` 只测平移；仍怪可试 `ori_mode=track` 或调 `controller_quat_xyzw` |
| 夹爪开合慢 | 已放大 2F-85 `force_scale` + `extra_sim_steps`；重跑一键；还可把 `gripper_gamma` 降到 `0.45` |
| 跟手不够远 | 把 `runtime.scale` 调到 `1.3~1.6` |
| 跟手慢/够不着 | 提高 `max_step_m`、`workspace_radius_m`；确认未顶工作空间球 |
| 只想测仿真不要 Pico | `./start.sh` → 自定义 → `source=mock` |
