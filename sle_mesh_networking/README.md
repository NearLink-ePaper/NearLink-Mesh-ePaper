<p align="center">
  <img src="https://img.shields.io/badge/Platform-BearPi--Pico%20H3863-blue?style=for-the-badge&logo=hisilicon" alt="Platform"/>
  <img src="https://img.shields.io/badge/Protocol-SparkLink%20SLE-00b96b?style=for-the-badge" alt="Protocol"/>
  <img src="https://img.shields.io/badge/Routing-AODV%20Mesh-orange?style=for-the-badge" alt="Routing"/>
  <img src="https://img.shields.io/badge/License-Apache%202.0-lightgrey?style=for-the-badge" alt="License"/>
  <img src="https://img.shields.io/badge/Language-C%20%7C%20Kotlin-yellowgreen?style=for-the-badge" alt="Language"/>
  <img src="https://img.shields.io/badge/OS-LiteOS-red?style=for-the-badge" alt="OS"/>
</p>

<h1 align="center">🌐 SLE Mesh — 星闪多跳自组网图传系统</h1>

<p align="center">
  <b>首个基于 HiSilicon H3863 的 SparkLink (NearLink) SLE Mesh 开源实现</b><br>
  手机 ←BLE→ 网关 ←SLE Mesh→ 中继 ←SLE→ 电子墨水屏
</p>

<p align="center">
  <a href="#-快速开始">快速开始</a> •
  <a href="#-系统架构">架构</a> •
  <a href="#-核心特性">特性</a> •
  <a href="#-配置指南">配置</a> •
  <a href="#-技术文档">文档</a> •
  <a href="#-性能数据">性能</a> •
  <a href="#-常见问题">FAQ</a> •
  <a href="#-contributing">Contributing</a>
</p>

---

## 📖 项目简介

**SLE Mesh** 是运行在 [BearPi-Pico H3863](https://gitee.com/bearpi/bearpi-pico_h3863) 开发板上的**星闪 (SparkLink/NearLink) 多跳 Mesh 自组网系统**，可在多块开发板之间自动组建 Mesh 网络，实现**手机通过 BLE 远程控制多块电子墨水屏显示图片**，并支持**一键组播同步**到最多 8 块屏幕。

> **🎯 一句话概括：** 手机发图 → BLE 到网关 → SLE Mesh 多跳中继 → 多块电子墨水屏同步刷新

<details>
<summary><b>English Summary</b></summary>

**SLE Mesh** is the **first open-source multi-hop Mesh networking stack** built on top of HiSilicon H3863's SparkLink (NearLink) SLE radio. It enables a phone to send images over BLE to a gateway node, which then routes them via a self-organizing SLE Mesh to one or more e-ink displays. Key features include AODV routing, AIMD flow control, window-based bulk transfer, Turbo mode (dynamic connection interval), and multicast delivery to up to 8 nodes.

</details>

### 为什么做这个项目？

- **星闪 (NearLink/SparkLink)** 是华为主导的新一代近距离无线通信技术，但当前几乎没有开源的 Mesh 组网实现
- 官方 SDK 仅提供点对点 SLE UART 示例，不支持多跳、路由、拓扑自发现
- 本项目从零实现了**完整的自研 Mesh 协议栈**，覆盖传输层 → 路由层 → 应用层，可作为**星闪 Mesh 开发的参考框架**

### 适合谁？

| 人群 | 收获 |
|------|------|
| 🎓 在校学生 / 大创项目 | 完整的嵌入式 + 无线协议栈工程实践 |
| 📡 星闪/NearLink 开发者 | 目前最完整的 SLE Mesh 开源参考实现 |
| 🔧 嵌入式 Mesh 研究者 | AODV 路由 + AIMD 流控 + 多角色管理实现细节 |
| 📱 BLE/IoT 应用开发者 | BLE ↔ Mesh 网关设计模式 + Kotlin/Compose 配套 App |

---

## ✨ 核心特性

<table>
<tr>
<td width="50%">

**🔗 Mesh 自组网**
- 上电自动扫描、建连、组网，无需手动配置
- AODV 按需路由，支持任意拓扑（星形、线性、混合）
- HELLO 心跳 + 2-hop 路由学习，TTL=7 支持多跳
- RERR 快速故障收敛，僵尸连接三维判定清理

</td>
<td width="50%">

**📡 BLE + SLE 双射频**
- BLE 作为手机入口（GATT Write），无 App 也可用 nRF Connect
- SLE 作为 Mesh 骨干网，节点间最大 5 并发连接
- 广播名 `sle_mesh_XXXX` 自动地址解析，兼容自动/手动寻址

</td>
</tr>
<tr>
<td width="50%">

**🖼️ 可靠图片传输**
- **FAST 模式**：窗口化批量发送 (AIMD 流控，窗口最大 60)
- **ACK 模式**：逐包确认，极低丢包场景
- **组播 v3**：一次广播最多 8 节点同步接收，位图补包
- CHECKPOINT 分段验证 + 位图重传，保证 100% 完整性

</td>
<td width="50%">

**💪 健壮性设计**
- P1-P11 系列补丁修复生产环境真实 Bug
- Turbo 模式：传输期间把 SLE 连接间隔从 50ms 压缩到 15ms
- 跳数自适应：1/2/3-hop 分别使用不同 FC 参数
- 去重缓存防洪泛环路，F30 消息帧级别过滤

</td>
</tr>
</table>

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         📱 手机 App (Kotlin/Compose)                     │
│  BleManager → MeshProtocol (0x01~0x0A / 0x81~0x8A) → MainActivity UI   │
└─────────────────────────┬───────────────────────────────────────────────┘
                           │ BLE GATT (Write / Notify)
                           ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      Gateway Node (BearPi-Pico H3863)                   │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  BLE Layer: ble_gateway.{h,c}                                    │   │
│  │  FAST/ACK/MCAST 流控引擎 • AIMD 窗口控制 • CHECKPOINT 验证       │   │
│  └──────────────────┬───────────────────────────────────────────────┘   │
│  ┌──────────────────▼───────────────────────────────────────────────┐   │
│  │  Mesh Protocol: mesh_main.c / mesh_forward.{h,c}                 │   │
│  │  消息路由 • 洪泛去重 • Unicast/Broadcast/Multicast               │   │
│  └──────────────────┬───────────────────────────────────────────────┘   │
│  ┌──────────────────▼───────────────────────────────────────────────┐   │
│  │  Mesh Route: mesh_route.{h,c}  (AODV)                            │   │
│  │  路由表 • RREQ/RREP/RERR • 2-hop 路由学习 • 超时清理             │   │
│  └──────────────────┬───────────────────────────────────────────────┘   │
│  ┌──────────────────▼───────────────────────────────────────────────┐   │
│  │  Mesh Transport: mesh_transport.{h,c}                             │   │
│  │  连接池(5×) • Server/Client 双角色 • Zombie 清理 • Turbo 模式     │   │
│  └──────────────────┬───────────────────────────────────────────────┘   │
└─────────────────────┼───────────────────────────────────────────────────┘
                       │ SLE (SparkLink) Physical Layer
          ┌────────────┴────────────┐
          ▼                         ▼
┌──────────────────┐       ┌──────────────────┐
│  Relay Node      │       │  Receiver Node   │
│  (纯转发)         │──SLE──│  image_receiver  │
│  AODV 中继       │       │  SPI E-Paper驱动  │
└──────────────────┘       └──────────────────┘
```

### 协议栈层次

```
应用层    │ BLE 命令协议 (0x01~0x0A) │ 图片流控 (FC) │ 拓扑查询 (TOPO)
──────────┼───────────────────────────────────────────────────────────────
Mesh 协议 │ 消息类型分发 │ 单播/广播/组播 │ 去重 │ MCAST 目标管理
──────────┼───────────────────────────────────────────────────────────────
路由层    │ AODV: RREQ → RREP → RERR │ 路由表 │ HELLO 邻居发现
──────────┼───────────────────────────────────────────────────────────────
传输层    │ 连接池管理 │ Server/Client 角色 │ CRC16 │ Turbo 模式
──────────┼───────────────────────────────────────────────────────────────
物理层    │ SLE (SparkLink 2M PHY) ←→ SLE │ BLE 5.0 ←→ Phone
```

---

## 🚀 快速开始

### 硬件准备

| 数量 | 硬件 | 角色 | 说明 |
|:---:|------|------|------|
| 1块 | BearPi-Pico H3863 | **Gateway** | 连接手机 (BLE) + 连接 Mesh (SLE) |
| 0~N 块 | BearPi-Pico H3863 | **Relay** | 纯中继转发，扩展网络覆盖范围 |
| 1块 | BearPi-Pico H3863 + E-Paper | **Receiver** | 接收图片 + 墨水屏显示 |

> **最小配置**：2 块板（1 Gateway + 1 Receiver）即可体验端到端图传  
> **推荐配置**：3~4 块板体验多跳 Mesh 的魅力  
> **组播演示**：6+ 块板可体验一键同步到多屏幕

### 环境搭建

**1. 获取 SDK**
```bash
git clone https://gitee.com/bearpi/bearpi-pico_h3863.git
cd bearpi-pico_h3863
```

**2. 复制本项目**
```bash
# 克隆本项目
git clone https://github.com/time2024/sle-mesh-networking.git

# 复制到 SDK samples 目录
cp -r sle-mesh-networking/ application/samples/products/sle_mesh_networking/
```

**3. 启用编译选项**

编辑 `build/config/target_config/ws63/config.cmake`（或通过 menuconfig），启用：
```cmake
set(SAMPLE_SUPPORT_SLE_MESH ON)
```

**4. 配置节点地址**（可选）

默认使用 **EFUSE Die-ID 自动地址**，无需修改即可组网。如需手动指定：
```c
// mesh/mesh_config.h
#define MESH_NODE_ADDR  0x0001   // 每块板设不同值：0x0001 ~ 0xFFFD
```

**5. 编译**
```bash
python build.py ws63-liteos-app
```

**6. 烧录**

使用 HiBurn 工具烧录 `output/ws63/` 下生成的固件到每块开发板。

> 💡 **提示**：所有板子烧录**同一份固件**，节点角色（Gateway/Relay/Receiver）由运行时自动协商。

**7. 上电组网**

所有板子上电后将**自动扫描、建连、组网**。通过串口（115200 baud）可看到：
```
[sle mesh] transport init ok
[sle mesh] my addr: 0xEBD8 (auto, from EFUSE)
[sle mesh] server connected: conn_id=0, total=1
[sle mesh] neighbor addr update: conn_id=0, 0x0000 -> 0xB3F1
[sle mesh] HELLO sent, neighbors=1
[sle mesh] route add: dest=0x7A2C via 0xB3F1, hops=2
```

**8. 手机 App 连接**

使用配套 Android App（见 [`meshgateway/`](meshgateway/)）或 nRF Connect / LightBlue 连接名为 `BLE_MESH_GW_XXXX` 的设备，通过 Write Characteristic 发送图片。

---

## ⚙️ 配置指南

所有参数集中在 [`mesh/mesh_config.h`](mesh/mesh_config.h)，带有详细注释和调优建议。

### 网络基础参数

| 参数 | 默认值 | 说明 | 调优方向 |
|------|:------:|------|----------|
| `MESH_NODE_ADDR` | `0` (自动) | 节点 16-bit 地址 | 手动模式设 0x0001~0xFFFD |
| `MESH_MAX_SERVER_CONN` | `1` | Server 入向连接数 | ⚠️ 芯片硬限制，勿改 |
| `MESH_MAX_CLIENT_CONN` | `4` | Client 出向连接数 | ⚠️ 芯片硬限制，勿改 |
| `MESH_DEFAULT_TTL` | `7` | 帧生存跳数 | 网络直径 > 7 时增大 |
| `MESH_MCAST_MAX_TARGETS` | `8` | 单次组播最大目标数 | 受 Mesh 拓扑限制 |
| `MESH_CONNECT_FILTER_ADDR` | `0x0000` | 连接白名单（调试用） | 生产环境保持 0x0000 |

### 定时器参数

| 参数 | 默认值 | 说明 | 约束关系 |
|------|:------:|------|----------|
| `MESH_HELLO_INTERVAL_MS` | `5000ms` | HELLO 心跳间隔 | ≥ 3000ms |
| `MESH_SCAN_INTERVAL_MS` | `5000ms` | 邻居扫描周期 | 建议 ≈ HELLO_INTERVAL |
| `MESH_ROUTE_TIMEOUT_MS` | `90000ms` | 路由条目超时 | **必须 > HELLO × 3** |
| `MESH_ZOMBIE_TIMEOUT_MS` | `30000ms` | 僵尸连接超时 | **必须 > HELLO × 3** |
| `MESH_SEND_FAIL_THRESHOLD` | `10` | 快速僵尸判定阈值 | 5 ~ 20 |

### 跳数自适应流控参数

| 跳数 | 初始窗口 (W) | 包间延迟 (D) | 超时 (TO) | 起始延迟 |
|:----:|:-----------:|:-----------:|:---------:|:--------:|
| 1-hop | 20 | 8ms | 2500ms | 300ms |
| 2-hop | 15 | 10ms | 5000ms | 500ms |
| 3-hop | 10 | 15ms | 8000ms | 800ms |

### SLE 射频参数

| 参数 | 默认值 | 说明 |
|------|:------:|------|
| `MESH_SLE_MTU_SIZE` | `520B` | MTU 大小，双端需一致 |
| `MESH_SLE_CONN_INTV_MIN` | `0x64` (50ms) | 常规连接间隔 |
| `MESH_SLE_TURBO_INTV` | `0x1E` (15ms) | Turbo 模式连接间隔 |
| `IMG_FC_PKT_PAYLOAD` | `480B` | 单包有效载荷 |

---

## 📚 技术文档

### 帧格式

所有 Mesh 帧共用 16 字节帧头（小端字节序），之后紧跟 0~504 字节有效载荷：

```
 Byte:  0      1      2       3     4-5    6-7    8-9   10-11  12-13  14-15
       ┌──────┬──────┬───────┬─────┬─────┬─────┬─────┬──────┬──────┬──────┐
       │magic │ ver  │ type  │ ttl │ src │ dst │ seq │ len  │ hop  │ CRC  │
       │ 0xAE │ 0x01 │(1byte)│     │ (2B)│ (2B)│ (2B)│ (2B) │ (2B) │(2B) │
       └──────┴──────┴───────┴─────┴─────┴─────┴─────┴──────┴──────┴──────┘
       │                  payload (0 ~ 504 bytes)                           │
       └────────────────────────────────────────────────────────────────────┘
  CRC: CRC16-CCITT, Little-Endian
```

### 消息类型一览

**Mesh 核心消息**

| Type | 名称 | 方向 | 说明 |
|:----:|------|------|------|
| `0x01` | UNICAST | 单播 | 应用层有效载荷，逐跳路由转发 |
| `0x02` | BROADCAST | 广播 | 全网洪泛，去重缓存防环路 |
| `0x10` | RREQ | 广播 | 路由请求（AODV 路由发现） |
| `0x11` | RREP | 单播 | 路由应答（沿反向路径回源） |
| `0x12` | RERR | 广播 | 路由错误（链路断裂通知） |
| `0x20` | HELLO | 广播 | 心跳 + 邻居列表（2-hop 路由学习） |

**应用层消息（BLE↔Mesh）**

| Type | 名称 | 方向 | 说明 |
|:----:|------|------|------|
| `0xF0` | IMG_START | GW→RX | 单播图传开始（FAST/ACK 模式） |
| `0xF1` | IMG_DATA | GW→RX | 图像数据包（seq 编号） |
| `0xF2` | IMG_END | GW→RX | 图传结束（含 CRC32 校验） |
| `0xF3` | IMG_ACK | RX→GW | 逐包确认（ACK 模式） |
| `0xF4` | CHECKPOINT | GW→RX | 分段检查点 |
| `0xF5` | CHKPT_ACK | RX→GW | 检查点状态（OK / 位图缺包） |
| `0xF6` | RESULT | RX→GW | 传输结果（成功/CRC 错误） |
| `0xF7` | MCAST_START | GW→ALL | 组播图传开始（含目标列表） |
| `0xFD` | TURBO | GW→RX | 动态调整 SLE 连接间隔 |
| `0xFE` | TOPO | 广播/单播 | 拓扑查询/响应 |

### AODV 路由发现

```
源节点 S 需要发送到目标 D（无路由时）：

  S ──RREQ(bcast)──► A ──RREQ(bcast)──► B ──RREQ(bcast)──► D
  │  (建反向路由S)    │  (建反向路由S)   │  (建反向路由S)   │
  │                  │                  │                   │
  │                  │                  │◄──RREP(unicast)── │
  │                  │◄──RREP(unicast)──│  (建正向路由D)    │
  │◄──RREP(unicast)──│  (建正向路由D)   │                   │
  │  (建正向路由D)   │                  │                   │
  │                  │                  │                   │
  S ─────DATA───────► A ─────DATA──────► B ─────DATA───────► D
```

HELLO 报文携带本节点邻居列表，远端节点通过它**直接学习 2-hop 路由**，减少 RREQ 泛洪。

### 图片传输流程（FAST + 组播）

```
  Phone         Gateway              Relay              Receiver(s)
    │   Write      │                   │                   │
    │ ──────────► │   FC_START_WAIT   │                   │
    │   ACK(BLE)  │                   │                   │
    │ ◄────────── │                   │                   │
    │             │  MCAST_START ───► │ ─────────────────► │×N
    │             │  TURBO ON    ───► │ (15ms interval)   │
    │             │                   │                   │
    │             │  ┌─ 批量发送 Window 个包 ─────────────┐│
    │             │  │  DATA(seq=0) ──►│──────────────────►││
    │             │  │  DATA(seq=1) ──►│──────────────────►││
    │             │  │  ...           │                   ││
    │             │  └────────────────────────────────────┘│
    │             │  CHECKPOINT ────► │──────────────────► │
    │             │                   │ CHKPT_ACK(bitmap)◄──│
    │             │ ◄─────────────── │ [AIMD 窗口调整]    │
    │             │                   │                   │
    │             │  IMG_END ───────► │──────────────────► │
    │             │                   │  RESULT(status) ◄── │
    │   Notify    │                   │                   │
    │ ◄────────── │  TURBO OFF  ────► │ (50ms interval)   │
    │             │                   │          📺 刷新屏幕！
```

### AIMD 流控算法

```
初始:  W=20, D=8ms (1-hop 为例)

每个 CHECKPOINT 周期:
  if (全部收到, 无丢包):
    W = min(W + W/4, W_MAX)   // Additive Increase（加性增）
    D = max(D - 1, D_MIN)     // 减少包间延迟
  else (有丢包, 位图补包):
    W = max(W / 2, W_MIN)     // Multiplicative Decrease（乘性减）
    D = min(D + 2, D_MAX)     // 增加包间延迟

W_MAX=60, W_MIN=5, D_MIN=4ms, D_MAX=30ms
```

### 位图补包机制

Receiver 通过 30 字节位图上报缺失包，精准触发重传：

```
CHKPT_ACK 报文: [0xF5, cnt_hi, cnt_lo, status, bitmap[30]]

bitmap[0] bit0 = seq 0 缺失
bitmap[0] bit1 = seq 1 缺失
...
bitmap[29] bit7 = seq 239 缺失

Gateway 仅重传 bitmap 中标记的包（FC_RETRANSMIT 状态），不重传全部。
最大可追踪 240 个包的缺失状态（30×8 bits）。
```

### 组播 v3 机制

```
1. App 发送 MCAST_START（含目标地址列表，最多 8 个）
2. Gateway 广播 MCAST_START 到 Mesh 全网
3. 目标节点接收并锁定 gateway 地址开始接收
4. Gateway 自身（若在目标列表中）自动标记 done
5. 数据包广播发送，所有目标节点同时接收
6. 每个 Receiver 独立回复 CHKPT_ACK / RESULT
7. Gateway 等待所有目标 RESULT，或超时后重试（最多 3 轮）
8. 全部完成后通过 BLE Notify 向 App 上报结果
```

---

## 📊 性能数据

> 数据来自实测，硬件：BearPi-Pico H3863，图片：240×360 (10,800 字节)，PHY：SLE 2M

| 配置 | 跳数 | 单节点传输时间 | 6节点组播时间 | 备注 |
|------|:----:|:------------:|:-----------:|------|
| FAST 1-hop | 1 | ~2~4 秒 | ~5~8 秒 | Turbo+AIMD 最优 |
| FAST 2-hop | 2 | ~6~10 秒 | ~15~20 秒 | 中继转发延迟增加 |
| FAST 3-hop | 3 | ~12~18 秒 | ~25~35 秒 | 保守参数保稳定 |
| ACK 模式 | 1 | ~8~15 秒 | N/A | 极可靠，用于弱信号 |

**网络规模**
- 已测试：**6 节点，1跳/2跳混合拓扑**，稳定运行
- 地址空间：65,533 个节点（0x0001~0xFFFD）
- 单节点最大并发连接：5（1 Server + 4 Client）
- 路由表：默认 32 条（`MESH_ROUTE_TABLE_SIZE` 可调）

---

## 📂 项目结构

```
sle_mesh_networking/
├── mesh/                          # Mesh 协议栈核心
│   ├── mesh_config.h              #   ⚙️ 所有可配参数（含详细注释）
│   ├── mesh_types.h               #   📐 帧格式、消息类型、数据结构
│   ├── mesh_api.{h,c}             #   🔌 应用层公共 API (send/broadcast/query)
│   ├── mesh_main.c                #   🚀 入口：任务调度 + 各模块初始化
│   ├── mesh_transport.{h,c}       #   🔗 传输层：连接池 + Server/Client 双角色
│   ├── mesh_forward.{h,c}         #   📦 协议层：消息分发 + 洪泛去重 + MCAST
│   └── mesh_route.{h,c}           #   🗺️ 路由层：AODV RREQ/RREP/RERR + HELLO
├── ble_gateway.{h,c}              # 🌉 BLE-Mesh 网关：图传流控引擎 (AIMD/FC)
├── image_receiver.{h,c}           # 🖼️ 接收端：图片重组 + SPI E-Paper 驱动
├── sle_uart_server/               # 📥 SLE Server 角色（广播 + notify 发送）
│   └── sle_uart_server.{h,c}
├── sle_uart_client/               # 📤 SLE Client 角色（扫描连接 + write 发送）
│   └── sle_uart_client.{h,c}
├── meshgateway/                   # 📱 Android App (Kotlin + Compose)
│   ├── BleManager.kt              #   BLE 扫描连接管理
│   ├── MeshProtocol.kt            #   协议编解码
│   └── MainActivity.kt            #   UI + 图片发送 + 拓扑展示
├── sle_uart.c                     # 🔧 SLE UART 桥接层入口
├── CMakeLists.txt                 # 🏗️ 构建配置
└── Kconfig                        # 🔧 编译选项定义
```

---

## 🔨 优化与补丁索引

所有补丁和优化在代码中以 `P<N>`/`O<N>` 注释标记，便于追溯。

### 性能优化 (O 系列)

| 编号 | 名称 | 说明 | 相关文件 |
|:----:|------|------|---------|
| O1 | 流水线发送 | 窗口化批量发送，替代逐包等 ACK | `ble_gateway.c` |
| O2 | Turbo 模式 | 传输时动态降低 SLE 连接间隔 (50ms→15ms) | `mesh_transport.c` |
| O3 | 批量 TX | 一次 tick 发送多包，减少调度开销 | `ble_gateway.c` |
| O4 | 大包优化 | 单包载荷从 200B 提升到 480B | `mesh_config.h` |
| O5 | 跳数自适应 | 1/2/3-hop 分别使用差异化 FC 参数 | `ble_gateway.c` |
| O6 | 动态 StartDelay | 基于路由表跳数动态计算 START 延迟 | `ble_gateway.c` |

### 补丁修复 (P 系列)

| 编号 | 名称 | 说明 | 相关文件 |
|:----:|------|------|---------|
| P1 | 地址锁定 | 防止 HELLO 消息导致 neighbor addr 串台 | `mesh_transport.c` |
| P2 | 路由通知 | 断连时触发 RERR 通知全网，加速收敛 | `mesh_transport.c` |
| P3 | Fallback 组网 | 孤立节点放宽地址大小限制，加速组网 | `mesh_transport.c` |
| P4 | Server 地址策略 | Server 端不从 SLE 地址推导 mesh_addr | `mesh_transport.c` |
| P6 | 僵尸清理 | RX超时/发送失败/单向检测 三维判定 | `mesh_transport.c` |
| P7 | 快速僵尸检测 | 连续发送失败 ≥10 次即判定僵尸 | `mesh_transport.c` |
| P8 | conn_id 冲突 | 退避+黑名单防护 SLE 协议栈 conn_id bug | `mesh_transport.c` |
| P10 | Notify 延迟 | 新连接等 MTU 协商完成后再发 Notify | `mesh_transport.c` |
| P11 | Exchange 重试 | 优雅处理 ssapc_exchange_info_req 瞬态失败 | `sle_uart_client.c` |
| F30 | 帧级过滤 | 过滤传输后残留的旧消息帧，防幽灵重传 | `ble_gateway.c` |

---

## 🐛 调试技巧

### 串口日志

```bash
# 关键日志前缀（115200 baud）
[sle mesh]         # Mesh 核心模块
[sle uart server]  # SLE Server
[sle uart client]  # SLE Client
[ble gw]           # BLE Gateway 流控引擎
[img rx]           # 图片接收端
```

### 常见日志解读

```
# ✅ 正常组网
[sle mesh] neighbor addr update: conn_id=0, 0x0000 -> 0xB3F1   ← 邻居发现
[sle mesh] HELLO sent, neighbors=2                               ← 心跳正常
[sle mesh] route add: dest=0x7A2C via 0xB3F1, hops=2           ← 2-hop 路由学习

# ✅ 组播传输（实测 6 节点）
[ble gw] MCAST START: 6 targets, total=10800B pkts=54
[ble gw] FC MCAST mode: 6 targets, using conservative params
[ble gw] MCAST RESULT from 0x1D22: status=0 (2/6 done)
[img rx] MCAST accepted: 240x360 total=10800B pkt=23 mode=0 xfer=FAST

# ⚠️ 需关注
[sle mesh] P7 zombie detected: conn_id=1, inactive 35000ms      ← 僵尸已清理
[ble gw] FC MCAST RESULT timeout, round=1 (3/6 done)            ← 重试第1轮

# ❌ 异常
[sle mesh] conn pool full                                        ← 连接池满
[sle mesh] RREQ timeout for dest 0x7A2C, retry 2/3              ← 路由发现失败
```

### 调试拓扑控制

强制指定连接拓扑用于隔离问题：
```c
// mesh/mesh_config.h（调试专用，生产环境必须设为 0x0000）
#define MESH_CONNECT_FILTER_ADDR    0xB3F1   // 仅连接此地址的节点
```

---

## ❓ 常见问题

<details>
<summary><b>Q: 最多支持多少个节点？</b></summary>

理论地址空间 65,533 个（0x0001~0xFFFD）。实际受限于：
- 单板最多 5 个并发 SLE 连接（1 Server + 4 Client，芯片硬限制）
- 路由表默认 32 条（`MESH_ROUTE_TABLE_SIZE` 可修改）
- 已测试：**6 节点稳定运行**（1跳/2跳混合拓扑）

</details>

<details>
<summary><b>Q: 图片传输速度如何？</b></summary>

实测 240×360（约 10.8KB），SLE 2M PHY：
- **1 跳直连**：单节点 ~2~4 秒，6 节点组播 ~5~8 秒
- **2 跳中继**：单节点 ~6~10 秒
- **3 跳中继**：单节点 ~12~18 秒

Turbo 模式（连接间隔 15ms）+ AIMD 窗口（最大 60 包）时性能最优。

</details>

<details>
<summary><b>Q: 支持哪些电子墨水屏？</b></summary>

`image_receiver.c` 通过 SPI GPIO 驱动墨水屏，当前适配 1-bit 黑白模式（240×360）。修改 SPI 引脚定义、初始化序列和像素格式即可支持其他型号（灰度/彩色/不同分辨率）。

</details>

<details>
<summary><b>Q: 能移植到其他 HiSilicon 芯片吗？</b></summary>

核心 Mesh 协议栈仅依赖 OSAL 和 SLE API（`ssapc_`/`ssaps_` 前缀函数），理论上可移植到其他支持 SLE 的海思芯片（如 WS63E）。需适配：SLE 连接管理 API、GPIO 引脚配置、OSAL tick 频率。

</details>

<details>
<summary><b>Q: 组网失败，节点无法发现彼此？</b></summary>

1. 确认所有节点烧录的是同一版本固件
2. 检查串口是否打印 `server connected`（SLE 连接建立）
3. 使用 `MESH_CONNECT_FILTER_ADDR` 强制点对点连接，排查问题
4. 确认 `MESH_ADV_NAME_PREFIX = "sle_mesh_"`（双端必须一致）
5. 检查节点间距离（SLE 2M PHY 建议 < 10 米室内）

</details>

<details>
<summary><b>Q: 如何调试路由问题？</b></summary>

1. 串口搜索 `RREQ`/`RREP`/`RERR` 追踪路由消息
2. 使用手机 App 发送拓扑查询（0xFE 0x01）查看全网拓扑
3. 在 `mesh_config.h` 中设置 `MESH_CONNECT_FILTER_ADDR` 强制线性拓扑
4. 检查 `MESH_ROUTE_TIMEOUT_MS > MESH_HELLO_INTERVAL_MS × 3`

</details>

---

## 🤝 Contributing

欢迎贡献代码！方向包括：

- 🧪 更多节点规模（8+节点）测试与性能数据
- 📱 配套手机 App 功能完善（Flutter / React Native）
- 🎨 支持灰度/彩色/更大分辨率墨水屏
- 🔋 低功耗优化（SLE 超低功耗模式 + LiteOS 休眠）
- 📊 实时拓扑可视化监控工具
- 🌐 English documentation
- 🧩 移植到其他 SLE 芯片

**提交前请确认：**
- 代码风格与现有代码一致（函数注释参考 `mesh_config.h` 格式）
- 关键修改在代码中以 `/* Pxx: 说明 */` 或 `/* Oxx: 说明 */` 标注
- 补充串口日志示例便于验证

请提交 [Issue](https://github.com/time2024/sle-mesh-networking/issues) 或 [Pull Request](https://github.com/time2024/sle-mesh-networking/pulls)。

---

## 📄 License

本项目基于 [Apache License 2.0](LICENSE) 开源。

依赖的 HiSilicon BearPi SDK 部分代码受其原始许可证约束，详见各文件头部声明。

---

## ⭐ Star History

如果这个项目对你有帮助，请给个 Star 支持一下！它是国内首个开源的星闪 SLE Mesh 自组网实现。

[![Star History Chart](https://api.star-history.com/svg?repos=time2024/sle-mesh-networking&type=Date)](https://star-history.com/#time2024/sle-mesh-networking&Date)

---

<p align="center">
  <b>🌐 让星闪 Mesh 组网变得简单 · Making NearLink Mesh Simple</b><br>
  <sub>Made with ❤️ for the NearLink / SparkLink open-source community</sub>
</p>