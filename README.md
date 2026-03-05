<p align="center">
  <img src="https://img.shields.io/badge/Platform-BearPi--Pico%20H3863-blue?style=for-the-badge" alt="Platform"/>
  <img src="https://img.shields.io/badge/Protocol-SparkLink%20SLE-green?style=for-the-badge" alt="Protocol"/>
  <img src="https://img.shields.io/badge/Routing-AODV%20Mesh-orange?style=for-the-badge" alt="Routing"/>
  <img src="https://img.shields.io/badge/License-Apache%202.0-lightgrey?style=for-the-badge" alt="License"/>
</p>

<h1 align="center">🌐 SLE Mesh — 星闪多跳自组网图传系统</h1>

<p align="center">
  <b>基于 HiSilicon H3863 的 SparkLink (NearLink) SLE Mesh 自组网络</b><br>
  手机 ←BLE→ 网关 ←SLE Mesh→ 中继 ←SLE→ 电子墨水屏
</p>

<p align="center">
  <a href="#-快速开始">快速开始</a> •
  <a href="#-系统架构">架构</a> •
  <a href="#-核心特性">特性</a> •
  <a href="#-配置指南">配置</a> •
  <a href="#-技术文档">文档</a> •
  <a href="#-常见问题">FAQ</a>
</p>

---

## 📖 项目简介

**SLE Mesh** 是一个运行在 [BearPi-Pico H3863](https://gitee.com/bearpi/bearpi-pico_h3863) 开发板上的**星闪 (SparkLink/NearLink) 多跳 Mesh 自组网系统**，可在多块开发板之间自动组建 Mesh 网络，实现手机通过 BLE 远程控制电子墨水屏显示图片。

> **🎯 一句话概括：** 手机发图 → BLE 到网关 → SLE Mesh 多跳中继 → 电子墨水屏刷新显示

### 为什么做这个项目？

- **星闪 (NearLink/SparkLink)** 是华为主导的新一代近距离无线通信技术，但当前几乎没有开源的 Mesh 组网实现
- 官方 SDK 仅提供点对点 SLE UART 示例，不支持多跳、路由、拓扑自发现
- 本项目从零实现了完整的 Mesh 协议栈，可作为**星闪 Mesh 开发的参考框架**

### 适合谁？

- 🔬 对星闪 / NearLink 技术感兴趣的嵌入式开发者
- 🛜 想学习 Mesh 自组网（AODV 路由、洪泛控制、拓扑发现）的开发者
- 📟 对 BLE + SLE 双射频共存应用感兴趣的工程师
- 🎓 物联网 / 无线传感器网络课程的学生（完整的多跳网络教学示例）

---

## ✨ 核心特性

<table>
<tr>
<td width="50%">

### 🛜 Mesh 自组网
- **全自动拓扑发现**：节点上电即自动扫描、建连、组网
- **多跳中继**：数据可跨多个节点逐跳转发到达目的地
- **AODV 路由协议**：按需路由发现 (RREQ/RREP/RERR)
- **HELLO 心跳**：周期性广播邻居表实现 2-hop 被动路由学习
- **洪泛速率控制**：滑动窗口限速 + TTL 递减防止广播风暴
- **CRC16-CCITT 校验**：保障帧数据完整性

</td>
<td width="50%">

### 📱 BLE + SLE 双射频
- **BLE Gateway**：手机通过 BLE GATT 与网关通信
- **SLE Mesh 回程**：网关与 Mesh 网络之间通过 SLE 通信
- **BLE/SLE 共存**：同一芯片同时运行 BLE 和 SLE 协议栈
- **拓扑实时查询**：手机 APP 可请求并展示全网拓扑
- **ACK 代理**：网关缓存数据后立即回复手机，异步 Mesh 注入

</td>
</tr>
<tr>
<td>

### 🖼️ 图片可靠传输
- **AIMD 拥塞控制**：类 TCP 的加性增/乘性减窗口调整
- **CHECKPOINT 机制**：分段检查传输进度，精准定位丢包
- **位图补包**：30 字节 bitmap 一次覆盖 240 个 seq 的丢包状态
- **跳数自适应参数**：1-hop/2-hop/3-hop 各用不同流控参数
- **Turbo 模式 (O2)**：传输期间动态降低 SLE 连接间隔至 15ms
- **大包优化 (O4)**：单包载荷 480B，23 包完成一张 240×360 图片

</td>
<td>

### 🔧 健壮性设计
- **僵尸连接检测 (P6/P7)**：RX 超时 + 连续发送失败 + 单向检测
- **conn_id 冲突防护 (P8)**：退避 + 黑名单应对协议栈 bug
- **P1 地址锁定**：防止 HELLO 地址串台
- **P3 Fallback**：低邻居密度时放宽连接规则加速组网
- **P10 Notify 延迟**：Server 建连后等 MTU 协商完成再发数据
- **P11 Exchange Info 重试**：优雅处理瞬态 0xEE 错误

</td>
</tr>
</table>

---

## 🏗️ 系统架构

```
┌──────────┐     BLE GATT      ┌──────────────┐    SLE Mesh     ┌──────────┐    SLE Mesh     ┌──────────────┐
│          │  ◄────────────►   │   Gateway    │  ◄──────────►  │  Relay   │  ◄──────────►  │   Receiver   │
│  Phone   │    Notify/Write   │  (BLE+SLE)   │   多跳中继       │  (SLE)   │   多跳中继       │  (SLE+ePaper)│
│  (APP)   │                   │  0xEBD8      │                │  0xB3F1  │                │  0x7A2C      │
└──────────┘                   └──────────────┘                └──────────┘                └──────────────┘

                               ┌─────────────────────────────────────────────────────────────┐
                               │                    Mesh Protocol Stack                       │
                               │                                                             │
                               │  ┌─────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │
                               │  │ BLE GW   │  │ Forward  │  │  Route   │  │  Transport  │  │
                               │  │ FC/AIMD  │  │ CRC/Dedup│  │  AODV    │  │ ConnPool    │  │
                               │  │ ACK Proxy│  │ Flood    │  │ RREQ/    │  │ Server/     │  │
                               │  │ Topo     │  │ HELLO 2h │  │ RREP/RERR│  │ Client      │  │
                               │  └─────────┘  └──────────┘  └──────────┘  └─────────────┘  │
                               │                                                             │
                               │  ┌──────────────────────────────────────────────────────┐   │
                               │  │           SLE Protocol Stack (HiSilicon)             │   │
                               │  │    SSAP Server (Notify)  ·  SSAP Client (WriteReq)   │   │
                               │  └──────────────────────────────────────────────────────┘   │
                               └─────────────────────────────────────────────────────────────┘
```

### 帧格式

```
+-------+-------+------+-----+--------+--------+--------+--------+--------+--------+
| magic | ver   | type | ttl | src(2) | dst(2) | seq(2) | len(2) | hop(2) | crc(2) |
| 0xAE  | 0x01  |      |     |        |        |        |        |        |        |
+-------+-------+------+-----+--------+--------+--------+--------+--------+--------+
|                         payload (0 ~ 504 bytes)                                   |
+-----------------------------------------------------------------------------------+
```

---

## 📁 项目结构

```
sle_mesh_networking/
├── mesh/                          # 🛜 Mesh 协议栈核心（本项目主要代码）
│   ├── mesh_config.h              #   全局配置参数（50+ 可调宏，详细注释）
│   ├── mesh_types.h               #   核心类型定义与帧格式
│   ├── mesh_api.h                 #   对外 API 接口声明
│   ├── mesh_main.c                #   主任务入口：初始化 + 定时事件循环 + 消息分发
│   ├── mesh_transport.{h,c}       #   传输层：连接池管理 + 双角色收发 + 僵尸清理
│   ├── mesh_forward.{h,c}         #   转发层：CRC校验 + 去重 + 限速 + HELLO + 单播/广播
│   ├── mesh_route.{h,c}           #   路由层：AODV 协议（RREQ/RREP/RERR + 路由表）
│   ├── ble_gateway.{h,c}          #   BLE 网关：手机桥接 + FC 流控引擎 + 拓扑收集
│   └── image_receiver.{h,c}       #   接收端：图片组装 + CRC16 校验 + 墨水屏驱动
├── sle_uart_server/               # SLE Server 角色（被连接方 + Notify 发送）
│   ├── sle_uart_server.{h,c}      #   SSAP 服务注册 + 连接管理
│   └── sle_uart_server_adv.{h,c}  #   SLE 广播配置（动态地址 + 名称生成）
├── sle_uart_client/               # SLE Client 角色（主动扫描连接 + WriteReq 发送）
│   └── sle_uart_client.{h,c}      #   多连接管理 + 退避重连 + 服务发现
├── sle_uart.c                     # SLE UART 桥接层入口
├── CMakeLists.txt                 # 构建配置（三模式：Server / Client / Mesh）
├── Kconfig                        # 编译选项定义
└── board.json                     # 硬件引脚配置
```

---

## 🚀 快速开始

### 硬件准备

| 数量 | 硬件 | 角色 | 说明 |
|:---:|------|------|------|
| 1块 | BearPi-Pico H3863 | **Gateway** | 连接手机 (BLE) + 连接 Mesh (SLE) |
| 0~N 块 | BearPi-Pico H3863 | **Relay** | 纯中继转发，扩展网络覆盖范围 |
| 1块 | BearPi-Pico H3863 + E-Paper | **Receiver** | 接收图片 + 墨水屏显示 |

> **最小配置**：2 块板（1 Gateway + 1 Receiver）即可运行  
> **推荐配置**：3~4 块板体验多跳 Mesh 的魅力

### 环境搭建

1. **下载 SDK**
   ```bash
   git clone https://gitee.com/bearpi/bearpi-pico_h3863.git
   ```

2. **复制本项目到 SDK**
   ```bash
   cp -r sle_mesh_networking/ bearpi-pico_h3863/application/samples/products/
   ```

3. **配置节点地址**（二选一）

   - **自动模式**（推荐）：无需修改，默认从 SLE MAC 地址推导
   - **手动模式**：编辑 `mesh/mesh_config.h` 中的 `MESH_NODE_ADDR`
     ```c
     #define MESH_NODE_ADDR  0x0001  // 每块板设不同值
     ```

4. **编译**
   ```bash
   python build.py ws63-liteos-app
   ```
   > 在 SDK 根目录的 `build/config/target_config/` 中选择 `SAMPLE_SUPPORT_SLE_MESH` 编译选项

5. **烧录**
   
   使用 HiBurn 工具烧录 `output/ws63/` 下的固件到每块开发板

6. **上电组网**
   
   所有板子上电后将**自动扫描、建连、组网**。通过串口可看到：
   ```
   [sle mesh] transport init ok
   [sle mesh] my addr: 0xEBD8 (auto)
   [sle mesh] server connected: conn_id=0, total=1
   [sle mesh] neighbor addr update: conn_id=0, 0x0000 -> 0xB3F1
   [sle mesh] HELLO sent, neighbors=1
   ```

### 手机 APP 连接

网关节点同时运行 BLE 广播，手机使用支持 BLE GATT 的 APP（如 nRF Connect / LightBlue）连接名为 `BLE_MESH_GW_XXXX` 的设备，即可通过 Write Characteristic 发送图片数据。

---

## ⚙️ 配置指南

所有参数集中在 [`mesh/mesh_config.h`](mesh/mesh_config.h) 中，带有详细的注释和调优建议。以下是关键参数速查表：

### 网络基础参数

| 参数 | 默认值 | 说明 | 调优方向 |
|------|:------:|------|----------|
| `MESH_NODE_ADDR` | `0` (自动) | 节点 16-bit 地址 | 手动模式设 0x0001~0xFFFD |
| `MESH_MAX_SERVER_CONN` | `1` | Server 入向连接数 | **芯片硬限制，勿改** |
| `MESH_MAX_CLIENT_CONN` | `4` | Client 出向连接数 | **芯片硬限制，勿改** |
| `MESH_DEFAULT_TTL` | `7` | 帧生存跳数 | 网络直径大时增大 |

### 定时器参数

| 参数 | 默认值 | 说明 | 调优方向 |
|------|:------:|------|----------|
| `MESH_HELLO_INTERVAL_MS` | `5000` | HELLO 心跳间隔 | ↓加速收敛，↑降广播量 |
| `MESH_SCAN_INTERVAL_MS` | `5000` | 扫描周期 | ↓加速发现新邻居 |
| `MESH_ROUTE_TIMEOUT_MS` | `90000` | 路由超时 | 必须 > HELLO × 3 |
| `MESH_ZOMBIE_TIMEOUT_MS` | `30000` | 僵尸检测超时 | ↓加速清理，可能误杀 |

### 流控引擎 (FC) 参数

FC 参数在 `ble_gateway.c` 中定义，支持hop-adaptive（跳数自适应）：

| 参数 | 1-hop | 2-hop | 3-hop | 说明 |
|------|:-----:|:-----:|:-----:|------|
| 初始窗口 (W) | 20 | 15 | 10 | 同时发送的包数 |
| 包间延迟 (D) | 8ms | 10ms | 15ms | 包间隔 |
| 超时 (TO) | 2500ms | 5000ms | 8000ms | CHECKPOINT/RESULT 超时 |
| StartDelay | 300ms | 500ms | 800ms | START→首包发送延迟 |

### SLE 射频参数

| 参数 | 默认值 | 说明 |
|------|:------:|------|
| `MESH_SLE_MTU_SIZE` | `520` | MTU 大小，双端一致 |
| `MESH_SLE_CONN_INTV_MIN` | `0x64` (50ms) | 常规连接间隔 |
| `MESH_SLE_TURBO_INTV` | `0x1E` (15ms) | Turbo 模式间隔 |
| `IMG_FC_PKT_PAYLOAD` | `480` | 单包有效载荷 |

---

## 📚 技术文档

### 消息类型一览

| Type | 名称 | 方向 | 说明 |
|:----:|------|------|------|
| `0x01` | HELLO | 广播 | 心跳 + 邻居列表（2-hop 路由学习） |
| `0x02` | DATA | 单播 | 应用层数据 |
| `0x03` | ACK | 单播 | 数据确认 |
| `0x10` | RREQ | 广播 | 路由请求（AODV 路由发现） |
| `0x11` | RREP | 单播 | 路由应答（沿反向路径回源） |
| `0x12` | RERR | 广播 | 路由错误（链路断裂通知） |
| `0xFE` | TOPO | 广播/单播 | 网络拓扑查询/响应 |
| `0xFD` | TURBO | 单播 | 连接间隔动态调整控制 |

### AODV 路由协议

```
源节点 S 需要发送到目标 D，但没有路由：

  S ──broadcast──► A ──broadcast──► B ──broadcast──► D
  │                │                │                │
  │     RREQ       │     RREQ       │     RREQ       │
  │  (建立反向路由)  │  (建立反向路由)  │  (建立反向路由)  │
  │                │                │                │
  │                │                │   ◄──unicast── │
  │                │   ◄──unicast── │     RREP       │
  │   ◄──unicast── │     RREP       │  (建立正向路由)  │
  │     RREP       │  (建立正向路由)  │                │
  │  (建立正向路由)  │                │                │
  │                │                │                │
  S ──── DATA ────► A ──── DATA ────► B ──── DATA ────► D
```

### 图片传输流程

```
  Phone         Gateway              Relay              Receiver
    │   Write      │                   │                   │
    │──────────►   │   FC_START_WAIT   │                   │
    │   ACK(local) │                   │                   │
    │   ◄──────────│                   │                   │
    │              │   IMG_START ──────►│──────────────────►│
    │              │   Turbo ON ──────►│                   │
    │              │                   │                   │
    │              │   [批量发送 Window 个包]                 │
    │              │   DATA(seq=0) ───►│──────────────────►│
    │              │   DATA(seq=1) ───►│──────────────────►│
    │              │   ...             │                   │
    │              │   CHECKPOINT ────►│──────────────────►│
    │              │                   │   CHKPT_ACK ◄─────│
    │              │   ◄───────────────│                   │
    │              │   [AIMD 窗口调整]   │                   │
    │              │                   │                   │
    │              │   IMG_END ───────►│──────────────────►│
    │              │                   │    RESULT(OK) ◄───│
    │              │   ◄───────────────│                   │
    │   Notify     │                   │                   │
    │   ◄──────────│   Turbo OFF ────►│                   │
    │              │                   │          📺 屏幕刷新！
```

### 补包机制

当 Receiver 发现缺包时，通过 30 字节位图精准上报丢失的包号：

```
MISSING 报文: [0x87, cnt_hi, cnt_lo, bitmap[30]]

bitmap[0] bit0 = seq 0 缺失
bitmap[0] bit1 = seq 1 缺失
...
bitmap[29] bit7 = seq 239 缺失

Gateway 解析位图后进入 FC_RETRANSMIT 状态，仅重传标记为缺失的包。
```

---

## 🔨 补丁与优化索引

本项目经过大量迭代优化，所有补丁和优化在代码中有详细注释标记：

### 优化 (O系列)

| 编号 | 名称 | 说明 |
|:----:|------|------|
| O1 | 流水线发送 | 窗口化批量发送替代逐包等 ACK |
| O2 | Turbo 模式 | 传输时动态降低 SLE 连接间隔 |
| O3 | 批量 TX | 一次 tick 发送多包 |
| O4 | 大包优化 | 单包载荷从 200B 提升到 480B |
| O5 | 跳数自适应 | 1-hop/2-hop/3-hop 各用不同 FC 参数 |
| O6 | 动态 startDelay | 基于路由表跳数动态计算 START 延迟 |

### 补丁 (P系列)

| 编号 | 名称 | 说明 |
|:----:|------|------|
| P1 | 地址锁定 | 防止 HELLO 消息导致 mesh_addr 串台 |
| P2 | 路由通知 | 断连时触发 RERR 通知全网 |
| P3 | Fallback 组网 | 孤立节点放宽地址大小限制加速组网 |
| P4 | Server 地址策略 | Server 端不从 SLE 地址推导 mesh_addr |
| P6 | 僵尸清理 | 定期检测并清理死连接释放资源 |
| P7 | 快速僵尸检测 | 连续发送失败 ≥10 次判定为僵尸 |
| P8 | conn_id 冲突 | 退避+黑名单防护 SLE 协议栈 bug |
| P10 | Notify 延迟 | 新连接等 MTU 协商后再发 Notify |
| P11 | Exchange 重试 | 优雅处理 ssapc_exchange_info_req 瞬态失败 |

---

## 🐛 调试技巧

### 串口日志

连接串口（115200 baud）查看实时日志：

```bash
# 关键日志标签
[sle mesh]         # Mesh 核心模块
[sle uart server]  # SLE Server
[sle uart client]  # SLE Client
[BLE_MESH_GW]      # BLE Gateway
[IMG_RX]           # 图片接收端
```

### 常见日志解读

```
# ✅ 正常组网
[sle mesh] neighbor addr update: conn_id=0, 0x0000 -> 0xB3F1   ← 邻居发现
[sle mesh] HELLO sent, neighbors=2                               ← 心跳正常
[sle mesh] route add: dest=0x7A2C via 0xB3F1, hops=2           ← 路由学习成功

# ⚠️ 需关注
[sle mesh] P7 zombie detected: conn_id=1, inactive 35000ms      ← 僵尸已清理
[sle mesh] WARN: conn_id=0 addr locked 0xB3F1, reject 0x7A2C   ← P1 地址锁定生效

# ❌ 异常
[sle mesh] conn pool full                                        ← 连接池满
[sle mesh] RREQ timeout for dest 0x7A2C, retry 2/3              ← 路由发现失败
```

---

## ❓ 常见问题

<details>
<summary><b>Q: 最多支持多少个节点？</b></summary>

理论上 Mesh 地址空间为 65533 个（0x0001~0xFFFD）。实际受限于：
- 单板最多 5 个并发 SLE 连接（1 Server + 4 Client）
- 路由表默认 32 条（可在 `MESH_ROUTE_TABLE_SIZE` 中增大）
- 已测试 **4 节点 3 跳**稳定运行
</details>

<details>
<summary><b>Q: 图片传输速度如何？</b></summary>

- **1 跳直连**：240×360 (10.8KB) 约 3~5 秒完成
- **2 跳中继**：约 8~12 秒
- **3 跳中继**：约 15~20 秒

传输速度受 SLE 连接间隔、丢包率、中继转发延迟等因素影响。
</details>

<details>
<summary><b>Q: 支持哪些电子墨水屏？</b></summary>

`image_receiver.c` 中通过 SPI GPIO 驱动墨水屏，当前适配 1-bit 黑白模式。修改 SPI 引脚定义和初始化序列即可适配其他型号墨水屏。
</details>

<details>
<summary><b>Q: 能跑在其他 HiSilicon 芯片上吗？</b></summary>

核心 Mesh 协议栈仅依赖 OSAL 和 SLE API，理论上可移植到其他支持 SLE 的海思芯片（如 WS63E）。需要适配的主要是 SLE 连接管理 API 和 GPIO 引脚配置。
</details>

<details>
<summary><b>Q: 如何调试路由问题？</b></summary>

1. 检查串口 HELLO 日志确认邻居发现是否正常
2. 使用手机 APP 发起拓扑查询（0xFE 0x01）查看全网拓扑
3. 在 `mesh_config.h` 中设置 `MESH_CONNECT_FILTER_ADDR` 精确控制拓扑
4. 搜索日志中的 `RREQ`/`RREP`/`RERR` 关键字追踪路由消息
</details>

---

## 🤝 Contributing

欢迎贡献代码！可以参与的方向：

- 🧪 更多节点规模的测试与性能数据
- 📱 配套手机 APP（Flutter / React Native）
- 🎨 支持灰度/彩色墨水屏
- 🔋 低功耗优化（休眠唤醒机制）
- 📊 可视化拓扑监控工具
- 📖 英文文档翻译

请提交 Issue 或 Pull Request。

---

## 📄 License

本项目基于 [Apache License 2.0](LICENSE) 开源。

依赖的 HiSilicon SDK 部分代码受其原始许可证约束。

---

## ⭐ Star History

如果这个项目对你有帮助，请给个 Star 支持一下！

[![Star History Chart](https://api.star-history.com/svg?repos=YOUR_USERNAME/sle-mesh-networking&type=Date)](https://star-history.com/#YOUR_USERNAME/sle-mesh-networking&Date)

---

<p align="center">
  <b>🌐 让星闪 Mesh 组网变得简单</b><br>
  <sub>Made with ❤️ for the NearLink/SparkLink community</sub>
</p>
