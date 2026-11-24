# Mid 360 农机版 · 非商用源码公开工具集

**面向 DJI T100 农机版激光雷达研究的独立数据解析、点云处理与白模工具。**

本项目整理自「Mid 360 农机版建图工作台」的 Windows 与 Android 工程，先公开可以独立阅读、运行和验证的部分工具，方便研究雷达数据、开发点云处理流程，以及比较不同几何处理方法的效果。

**允许授权范围内的个人自用、学习和非商业研究；未经另行书面许可，不得商用。具体以 [LICENSE](LICENSE) 为准。**

> **下载前请先了解发布范围：** 本仓库是独立工具源码子集，没有完整工作台界面、EXE、APK、FAST-LIO2／Point-LIO 原生库及集成工程，不能直接构建完整电脑建图软件或手机 App。当前公开内容与原工作台的全部功能并不相同。
>
> 本仓库采用自定义非商用许可，属于“源码公开”，不属于 OSI 定义的开源。第三方依赖、算法及商标仍遵循各自许可，不因本仓库公开而重新授权。

---

## 阅读导航

- [1. 项目背景与适用人群](#1-项目背景与适用人群)
- [2. 当前发布了什么](#2-当前发布了什么)
- [3. 当前没有发布什么](#3-当前没有发布什么)
- [4. 目录与源码入口](#4-目录与源码入口)
- [5. 环境与依赖](#5-环境与依赖)
- [6. Python 安装与离线自检](#6-python-安装与离线自检)
- [7. 示例：被动接收点云与 IMU](#7-示例被动接收点云与-imu)
- [8. 示例：读取、清理并另存点云](#8-示例读取清理并另存点云)
- [9. 八种修复操作与参数](#9-八种修复操作与参数)
- [10. 示例：从点云生成白模](#10-示例从点云生成白模)
- [11. Java 独立工具与测试](#11-java-独立工具与测试)
- [12. GPU、CPU 与大点云处理](#12-gpucpu-与大点云处理)
- [13. 协议、时间、单位与坐标](#13-协议时间单位与坐标)
- [14. 数据保护与结果复核](#14-数据保护与结果复核)
- [15. 已验证内容与证据边界](#15-已验证内容与证据边界)
- [16. 常见问题与排错](#16-常见问题与排错)
- [17. 许可、第三方依赖与贡献](#17-许可第三方依赖与贡献)
- [18. 更多文档](#18-更多文档)

## 1. 项目背景与适用人群

原工作台围绕农机版雷达的数据接收、手持／车载建图、录包和地图后处理展开开发。本次从中提取不依赖完整 SLAM 应用的工具，保留中文代码注释，并补充独立运行方式、合成示例和回归测试。

本仓库适合以下用途：

- **研究雷达数据：** 了解点云报文、反射率、标签、IMU、逐点时间与单位转换。
- **开发点云后处理：** 在自己的程序中接入去噪、碎片清理、地面识别、调平和局部几何处理。
- **尝试白模：** 比较局部三角面、建筑结构面与体素外表面的区别。
- **阅读跨端组件：** 对照 Python 与纯 Java 的点云工具、运动预览和数据帧工具。
- **学习验证方法：** 用可重复的合成数据检查边界、取消、资源预算和文件保护。

如果你希望“下载后双击就能开始完整建图”，当前仓库尚不能满足这一需求。建议先运行离线示例，确认这里的工具是否适合你的二次开发目标。

本项目为非官方研究工具。DJI、Livox、MID-360 等名称用于说明研究对象或格式参考，不代表制造商授权、背书或全型号兼容承诺。

## 2. 当前发布了什么

| 模块 | 已提供能力 | 使用入口 | 需要了解的边界 |
| --- | --- | --- | --- |
| 点云／IMU 解析 | 36 字节 Livox SDK2 式头结构解析，高／低精度直角坐标、球坐标、IMU，保留包字段与原始点序 | `radar_client.t100_stream.decode_packet` | 结构检查不等于所有固件的 CRC 验证 |
| 被动 UDP 接收 | 后台收包、源地址过滤、点云／IMU 状态、有限预览缓存、批量解码路径 | `Receiver`、`StreamState` | 不发送控制命令，不自动配置雷达网络 |
| 运动预览 | IMU 旋转补偿与时序检查相关组件 | `t100_preview_motion.py`、`MotionPreview.java` | 用于显示，不提供 SLAM 定位或全局轨迹 |
| 去噪与碎片清理 | 半径去噪、统计去噪、小连通片清理 | `repair_core.apply_operation` | 阈值过强也可能删除真实细节 |
| 地面与几何处理 | 地面识别、刚体调平、局部平整、平面拟合与小孔洞补点候选 | `repair_core.py` | 无法仅凭最终点云恢复真实运动轨迹 |
| 文件清理工具 | 接受受支持的二进制 PLY／PCD，清理后另存坐标点云及统计 | `white_cloud_filter.py` | 输出坐标点云不保留全部原始属性 |
| Python 白模 | 有支持点的局部表面、建筑墙面与水平结构面构造 | `white_mesh_core.py`、`white_building.py` | 推断几何需要检查，不是实测表面保证 |
| 模型导出 | OBJ、GLB 与模型统计、坐标说明 | `white_mesh_io.export_model` | 不是照片贴图或彩色纹理重建 |
| GPU 辅助 | 部分坐标变换、选区与半径查询的 OpenCL 接口 | `repair_compute.py` | 部分步骤加速，未提供 GPU SLAM 或 NPU 算法 |
| 纯 Java 工具 | 点云操作、体素外表面、撤销／重做、运动预览与网络帧编解码 | `java/src/cn/t100/mapping` | 不是安卓 App，也不会自行驱动 USB 网卡 |
| 测试与发布检查 | 五组 Python 回归、Java 验证、合成示例、文件清单与哈希检查 | `python/windows`、`java/test`、`tools` | 离线通过不能代替现场兼容性和精度验收 |

## 3. 当前没有发布什么

以下内容没有包含在本次公开子集中，请勿根据原工作台演示推断本仓库已提供：

| 内容 | 当前状态 |
| --- | --- |
| 完整 Windows／Android 建图工作台 | 未提供完整界面、打包工程或安装包 |
| FAST-LIO2、Point-LIO 及其原生集成 | 未包含算法源码、JNI／ctypes 集成、DLL／SO |
| 一键开始建图、轨迹估计、回环与整图重建 | 不是本工具集提供的独立功能 |
| ROI 全周恢复、启动采样、待机与电机控制 | 不包含对应控制帧发送器 |
| 安卓 eth0 配置、USB 权限界面与 OTG 驱动 | 纯 Java 字节处理组件不包含这些系统集成 |
| 厂商固件、烧录工具、驱动及 SDK 完整归档 | 未再分发 |
| 个人地图、真实录包、PCAP、现场照片和视频 | 未附带；示例使用合成数据 |
| 手机拍照贴图、SfM、多视图纹理重建 | 未包含完整流程 |
| 完整 GUI 中的一键修复流程与全部高级功能 | 本仓库只提供表中列出的基础组件，不是全部界面功能的等价替代 |

完整工程需要进一步处理第三方来源、许可和集成边界。仅把 GPL 算法移到另一个目录，并不能使带有禁商用条件的组合成品自动满足许可。详细说明见[发布范围与隔离](docs/发布范围与隔离.md)。

## 4. 目录与源码入口

```text
.
├── README.md                         # 总览、安装、示例与排错
├── LICENSE                           # 本项目独立代码的非商用许可
├── NOTICE.md                         # 参考来源与第三方依赖说明
├── requirements.txt                  # Python 最小依赖范围
├── SOURCE_MANIFEST.json              # 受控文件的相对路径、来源和 SHA-256
├── docs/
│   ├── 源码索引.md
│   ├── 协议与单位.md
│   ├── 工具使用.md
│   ├── 验证记录.md
│   ├── 发布范围与隔离.md
│   └── 发布到GitHub.md
├── python/
│   ├── radar_client/
│   │   ├── t100_stream.py            # 报文解析、接收和预览缓存
│   │   └── t100_preview_motion.py    # 预览旋转补偿
│   ├── desktop/
│   │   ├── repair_core.py            # 八种几何处理操作
│   │   ├── repair_resources.py       # 内存预算与资源检查
│   │   ├── repair_compute.py         # 可选 OpenCL 点运算
│   │   ├── white_cloud_filter.py     # 文件级孤点清理与另存
│   │   ├── white_mesh_io.py          # PLY／PCD 读取、OBJ／GLB 导出
│   │   ├── white_mesh_core.py        # 局部表面网格
│   │   ├── white_building.py         # 建筑结构面
│   │   └── field_geometry.py         # 剖面、几何与文件辅助
│   └── windows/                     # 沿用原项目命名的五组离线测试
├── java/
│   ├── src/cn/t100/mapping/
│   │   ├── PointTools.java
│   │   ├── WhiteMesh.java
│   │   ├── ToolHistory.java
│   │   ├── MotionPreview.java
│   │   └── UsbFrames.java
│   └── test/cn/t100/mapping/ToolsCheck.java
├── examples/repair_synthetic.py      # 不依赖真实地图的去噪示例
└── tools/
    ├── Test-Java.ps1                 # 编译、运行独立 Java 验证
    └── check_release.py              # 发布内容与文件哈希检查
```

建议阅读顺序：先跑合成示例 → 看 `repair_core.py` 与测试 → 用自己的点云验证 → 再接入实时接收或白模。逐文件入口见[源码索引](docs/源码索引.md)。

## 5. 环境与依赖

| 项目 | 要求或建议 | 是否必需 |
| --- | --- | --- |
| Python | 建议 Python 3.12；本次整理使用 CPython 3.12.10 | 运行 Python 工具时必需 |
| NumPy | `>=1.26,<3` | Python 工具必需 |
| SciPy | `>=1.11,<2` | Python 点云处理与当前完整测试必需 |
| JDK | 建议 JDK 17，使用 `--release 11` 编译 | 只运行 Java 组件时需要 |
| OpenCL | 操作系统及显卡厂商提供的兼容运行时／驱动 | 可选 |
| 雷达与网络 | 独立供电、接收端可达、正确的目标地址与 UDP 端口 | 仅实时接收时需要 |
| Android SDK／ROS／Docker | 当前独立工具的快速验证不需要这些环境 | 不需要 |

本仓库不附 Python、JDK、显卡驱动或预装虚拟环境。依赖范围不是锁定环境，也不表示每一种版本组合都已测试。

当前离线发布验证在 Windows x64 完成；Linux／macOS 的基本运行方式如下，但不将其视为已完成跨平台实机验收。OpenCL 加速实现的系统限制另见第 12 节。

## 6. Python 安装与离线自检

### 6.1 获取源码

在 GitHub 仓库页面点 **Code → Download ZIP**，解压后进入同时包含 `README.md`、`python`、`java` 和 `tools` 的目录。也可以通过 Git 克隆当前仓库。

以下所有相对路径命令都在这个**仓库根目录**运行。不要直接把 `python/desktop` 当作工作目录。

### 6.2 Windows PowerShell

```powershell
# 建立独立环境，不影响电脑已有项目。
py -3.12 -m venv .venv

# 安装依赖。直接调用虚拟环境 Python，不必先激活环境。
.\.venv\Scripts\python.exe -m pip install -r requirements.txt

# 让本次 PowerShell 会话能导入仓库内的两个 Python 包。
$env:PYTHONPATH = (Join-Path $PWD 'python')

# 先运行不读设备、不写地图的合成示例。
.\.venv\Scripts\python.exe -B examples/repair_synthetic.py

# 运行五组离线回归。
.\.venv\Scripts\python.exe -B -m unittest discover -s python/windows -p 'test_*.py' -v

# 检查发布文件清单、哈希与脚本定义的禁止入包项。
.\.venv\Scripts\python.exe -B tools/check_release.py
```

合成示例的预期输出：

```text
合成点云：963 -> 961，移除2个孤点，原平面保留。
```

这一示例构造 961 个平面点和两个远离平面的孤点，检查清理后平面坐标逐值不变。它展示的是这个合成输入下的结果，不是对所有真实地图的无损承诺。

### 6.3 Linux／macOS 的对应命令

```bash
python3.12 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
export PYTHONPATH="$PWD/python"
./.venv/bin/python -B examples/repair_synthetic.py
./.venv/bin/python -B -m unittest discover -s python/windows -p 'test_*.py' -v
./.venv/bin/python -B tools/check_release.py
```

`PYTHONPATH` 只对当前终端会话生效，重新打开终端后需要重新设置。仓库目前没有提供 `pip install .` 的项目打包入口。

测试中的 UDP 仅在本机环回地址与临时端口上使用合成报文，不会给真实雷达发送采样、待机或其他控制命令。`-B` 用于避免生成 Python 字节码文件。

## 7. 示例：被动接收点云与 IMU

### 7.1 先核对网络

参考设备为 AG901LD10／T100 农机版雷达。以下仅是原工作台使用过的参考配置，实际地址必须以自己的设备为准：

| 配置项 | 参考值 | 含义 |
| --- | --- | --- |
| 雷达源 IP | `192.168.1.10` | 用于可选源地址过滤 |
| 接收端 IP | `192.168.1.20` | 雷达数据包的目标主机地址 |
| 子网掩码 | `255.255.255.0`，即 `/24` | 参考局域网配置 |
| 点云目的端口 | UDP `60001` | 点云接收端口 |
| IMU 目的端口 | UDP `60003` | 内置 IMU 接收端口 |

准备事项：

1. 雷达按设备要求独立供电。以太网线、普通 USB 或 OTG 网卡不构成本仓库提供的雷达供电方案。
2. 雷达与接收端之间的有线／交换机链路可达，雷达目的 IP 与接收端实际 IP 一致。
3. 同一局域网没有重复 IP，两个接收程序没有同时占用目的 UDP 端口。
4. 防火墙允许当前 Python 程序在所需网络接收 UDP；不要用长期关闭整个防火墙的方式代替规则配置。
5. 雷达已经在输出目标数据。这个被动接收器不会替你启动采样或更改雷达控制状态。

`bind_ip='0.0.0.0'` 表示监听本机所有 IPv4 接口，**不表示给网卡设置了 IP，也不会改变雷达数据目的地**。

### 7.2 同时监听两种数据

将下面代码保存为自己的脚本，例如 `receive_preview.py`。在设置好 `PYTHONPATH` 的终端使用虚拟环境 Python 运行：

```python
import time
from radar_client.t100_stream import Receiver, StreamState

state = StreamState(max_points=300_000)
receivers = [
    Receiver(state, bind_ip='0.0.0.0', port=60001,
             source_ip='192.168.1.10'),
    Receiver(state, bind_ip='0.0.0.0', port=60003,
             source_ip='192.168.1.10'),
]

try:
    for receiver in receivers:
        receiver.start()

    # 仅观察 10 秒，不保存地图，也不发送任何控制命令。
    for _ in range(10):
        time.sleep(1)
        xyz, reflectivity, tags, meta = state.snapshot(seconds=0.5)
        print({
            'preview_points': len(xyz),
            'cloud_packets': meta['cloud_packets'],
            'imu_packets': meta['imu'],
            'invalid_packets': meta['invalid'],
            'cloud_age_s': meta['udp_age'],
            'imu_age_s': meta['imu_age'],
            'error': meta['error'],
        })
finally:
    for receiver in receivers:
        receiver.close()
```

如果修改了雷达地址，请同步修改 `source_ip`。该参数是**源地址过滤项**，不是接收端本机地址。

### 7.3 如何理解状态

- `cloud_packets`／`imu`：累计成功处理的点云包和 IMU 包数量。
- `invalid`：解析器判为无效的报文数量；应结合 `error` 和实际数据格式排查。
- `udp_age`／`imu_age`：相应数据距最近到达的主机单调时间间隔；没有收到过时可能为 `None`。
- `pps`：最近窗口中接收的有效点数指标，不能直接当作机械转速。
- `evicted`：因预览窗口时间或容量限制淘汰的点数，不能直接认定为网络丢包数。

`StreamState` 保存的是有界预览窗口，不是完整录包容器；转动雷达时，直接叠加窗口中的局部坐标也不等于世界坐标地图。这里没有 SLAM 位姿求解。

如果使用 `raw_callback`／`decoded_callback`，回调由接收线程调用，应快速返回。耗时匹配、同步磁盘导出或无界队列会拖慢接收；需要另行设计有界工作队列、积压观测与完整性检查。

## 8. 示例：读取、清理并另存点云

### 8.1 接受哪些输入

| 输入 | 当前读取范围 |
| --- | --- |
| 二进制 PLY | `binary_little_endian 1.0`，解析器接受的 float32 XYZ，或其规定的 XYZ＋RGB 布局 |
| 二进制 PCD | 解析器规定的 PCD 0.7、`FIELDS x y z`、float32、`DATA binary` 非组织点云布局 |
| NumPy 数组 | `repair_core` 接受有限值组成的 `N×3 float32` 坐标 |
| ASCII PLY／PCD、压缩 PCD、LAS／LAZ／E57 等 | 当前读取器不提供通用兼容 |
| 原手机 `map.sqlite` | 依赖完整 SLAM 集成的读取分支未发布；先用原软件导出支持的 PLY／PCD |

请直接传入具体的 `.ply`／`.pcd` 文件路径。不要仅凭扩展名判断兼容：多余字段、属性排列、精度、点数或数据长度不符合当前解析器，也可能被明确拒绝。

### 8.2 文件级孤点清理与导出

以下示例读取 `data/input.ply`，检查 10 cm 半径内是否至少有三个其他点作为邻居，将结果保存到新的输出目录：

```python
from pathlib import Path
from desktop.white_cloud_filter import clean_source, export_clean

source = Path('data/input.ply')

# 注意：这个接口的半径单位是厘米。
retained, discarded_sample, stats = clean_source(
    source,
    radius_cm=10.0,
    min_neighbors=3,
)
print(stats)

# 请先检查统计和局部细节，再决定是否采用候选。
# 导出器新建 clean_points_* 目录，不覆盖 source。
output = export_clean(Path('outputs'), source, retained, stats)
print('结果目录：', output)
```

输出目录包含坐标点云 `cleaned.ply` 与统计文件 `filter.json`。返回的 `discarded_sample` 仅是删除点的预览样本，不保证包含所有被删除点。

**属性范围：** 此处理与导出路径以 XYZ 坐标为主。即使输入含 RGB，也不表示输出会保留颜色；反射率、逐点时间、标签和自定义属性同样不能假设自动保留。

### 8.3 使用修复核心，保留原行对应关系

```python
from pathlib import Path
import numpy as np
from desktop.white_mesh_io import iter_exported_points
from desktop.repair_core import apply_operation

chunks = list(iter_exported_points(Path('data/input.ply')))
if not chunks:
    raise ValueError('输入没有点')
points = np.concatenate(chunks).astype(np.float32, copy=False)

# 这个接口的距离单位是米；0.10 m = 10 cm。
result = apply_operation(
    points,
    'radius',
    {'radius': 0.10, 'min_neighbors': 3},
    compute_mode='cpu',
)

print(result.stats)
retained = result.points[result.keep]
candidate = np.concatenate((retained, result.added))
print('输入：', len(points), '候选：', len(candidate))
```

上例将分批读取结果集中到内存，适合能够容纳的数据量。它不是完整的大地图流式管线。

`RepairResult` 的字段约定：

| 字段 | 含义 |
| --- | --- |
| `points` | 与原始输入行对应的候选坐标，不因删除操作提前压缩 |
| `keep` | 原始行是否保留的布尔掩码 |
| `moved` | 原始行是否发生移动的标记 |
| `added` | 另外生成的点，与原始实测点分开返回 |
| `stats` | 操作统计、诊断和部分操作的附加结果 |

需要处理选区时传入与点数等长的布尔 `selection`。大部分操作保护未选中点；**`level` 是例外：选区用于拟合参考面，但最终刚体变换作用于整幅点云**，避免只移动局部而破坏全图坐标关系。

`cancel` 可传入带有 `is_set()` 的事件对象或可调用检查函数；取消会抛出 `RepairCancelled`。`progress` 接收 `[0,1]` 进度值。调用者应把取消和失败与正常完成分别处理。

## 9. 八种修复操作与参数

下表列出当前 `apply_operation` 的操作名和默认参数。默认值仅用于说明接口，不表示适合所有室内、室外或不同密度数据。

| 操作 | 主要用途 | 默认参数 | 使用时重点复核 |
| --- | --- | --- | --- |
| `radius` | 删除邻域过于稀疏的孤点 | `radius=0.10`、`min_neighbors=3` | 电线、树枝、边缘和远处稀疏结构可能也很少邻居 |
| `statistical` | 按近邻平均距离识别离群点 | `neighbors=16`、`std_ratio=2.0` | 点密度变化明显时，远处有效点可能更容易被删 |
| `components` | 删除小型分离连通片 | `distance=0.15`、`min_points=30`、`min_extent=0.10` | 独立小物体也可能属于真实目标 |
| `ground` | 寻找受低处空间证据支持的地面候选 | `distance=0.05`、`max_slope_deg=25`、`seed=0` | 返回 `stats['ground_mask']`，不会自动删除地面或改写坐标 |
| `level` | 利用参考平面执行全图刚体调平 | `distance=0.05`、`max_slope_deg=35`、`target_height=0` | 旋转和平移整个点云，不能消除沿路累积的非刚性弯曲 |
| `flatten` | 局部平整，限制移动并保护边缘 | `radius=0.30`、`strength=0.5`、`max_move=0.08`、`edge_threshold=0.08` | 真实坡面和小结构可能被抹平，应限制选区 |
| `planes` | 将受支持的点向拟合平面靠拢 | `distance=0.05`、`max_move=0.05`、`strength=0.8`、`min_points=100`、`max_planes=8` | 弧面、细节和多层结构不一定适合平面化 |
| `fill` | 对有平面支持的小型内部孔洞生成补点候选 | `spacing=0.05`、`max_hole=0.25`、`plane_tolerance=0.03`、`max_added=100000` | 新点是推断结果；门窗、真实开口不能随意填满 |

距离、间距、高度与移动量均以**米**计，`max_slope_deg` 以**度**计，计数为整数，`strength` 为无量纲强度。完整参数限制由源码校验。

推荐按“复制原始输入 → 小选区试用 → 检查删除／移动／新增点 → 检查墙角和地面等局部剖面 → 另存候选”的顺序操作，避免一次叠加多种处理后难以确定哪一步产生了变化。

**关于重影：** 如果双墙、错层、地面弯曲来自轨迹估计、时间同步、去畸变、外参、动态物体或运动退化，几何清理只能处理部分表象。可靠修正通常还需要原始点云、IMU、时序与轨迹约束；仅对最终点云平整不能证明恢复了真实形状。

## 10. 示例：从点云生成白模

### 10.1 三类白模的区别

| 方法 | 本仓库入口 | 更适合尝试的输入 | 结果特征 |
| --- | --- | --- | --- |
| 局部表面网格 | Python `build_mesh_from_points` | 有足够邻域覆盖的表面点云 | 根据支持点构造局部三角面；覆盖不足处不保证连接 |
| 建筑结构面 | Python `build_building` | 已调平、Z 向上、具有墙面与地面证据的建筑场景 | 尝试提取主方向、直立墙面和水平面，不是任意建筑的完整重建 |
| 体素外露面 | Java `WhiteMesh` | 需要快速形成占用体素外壳的点集 | 具有体素外观，不能等同于精细拟合真实表面 |

白模是以几何形状为主的模型。它不等于照片贴图、BIM 语义模型，也不自动具备测绘级精度或完全封闭、可制造的拓扑。

### 10.2 生成并导出局部网格

```python
from pathlib import Path
import numpy as np
from desktop.white_mesh_io import iter_exported_points, export_model
from desktop.white_mesh_core import build_mesh_from_points

source = Path('data/input.ply')
chunks = list(iter_exported_points(source))
if not chunks:
    raise ValueError('输入没有点')
points = np.concatenate(chunks).astype(np.float32, copy=False)

# 5 cm 构面间距只是示例，不代表 5 cm 测量精度。
vertices, faces, stats = build_mesh_from_points(points, step_m=0.05)

# 可用 build_building(points, step_m=0.05) 尝试建筑结构面；
# 请先从 desktop.white_building 导入该函数，并确保 Z 向上。

output = export_model(Path('outputs'), source, vertices, faces, stats)
print('模型目录：', output)
print(stats)
```

成功导出后，新建的 `white_mesh_*` 目录内包含 OBJ、GLB、相关材质与 `metadata.json`。导出器拒绝空网格、无效坐标、越界三角形索引，以及将结果写入受保护的源地图目录。

坐标约定：OBJ 保持地图 **Z 向上**；GLB 转换为 **Y 向上**，变换为 `(x, z, -y)`。如果查看器中的模型朝向不同，先检查查看器坐标约定与元数据。

`build_building` 接受的 `step_m` 范围为 0.02～0.50 m，内部建筑网格还受自身最小尺度约束。把参数设为 2 cm，不表示最终每一处面片间距都是 2 cm，更不表示恢复了 2 cm 的真实细节。

### 10.3 为什么“点更密”不一定“质量更好”

- 减小体素或构面间距可能增加顶点、三角面、内存与处理时间。
- 插值、补点或铺面只能增加表示密度，不能创造新的测量信息。
- 缺少支持的表面不应仅为了视觉完整就连接；门窗与遮挡区域需要区分。
- 输入已经存在重影时，直接生成网格可能把重影固化成两层表面，应先分析输入质量。

## 11. Java 独立工具与测试

这些组件使用普通 Java 标准库，可以在没有手机、Android SDK、雷达或 USB 网卡时编译并执行其离线测试。

| 文件 | 职责 | 不承担的职责 |
| --- | --- | --- |
| `PointTools.java` | 点云修复、选区与推断点标记等基础操作 | 不提供完整地图编辑 UI |
| `WhiteMesh.java` | 占用体素外露面与 OBJ／GLB 输出 | 不等同于 Python 建筑面算法 |
| `ToolHistory.java` | 专用历史目录内的磁盘快照、撤销／重做 | 不代表完整 App 的数据管理功能 |
| `MotionPreview.java` | 有界陀螺历史与显示投影，四元数 `[w,x,y,z]` | 不求解完整 SLAM 轨迹 |
| `UsbFrames.java` | Ethernet／ARP／非分片 IPv4／UDP 字节帧编解码 | 不申请 USB 权限、不实现整个网卡驱动 |

Windows 下确认 `java`、`javac` 来自可用 JDK 后运行：

```powershell
java -version
javac -version
powershell -NoProfile -ExecutionPolicy Bypass -File tools/Test-Java.ps1
```

脚本用 JDK 17、Java 11 目标编译，将 `.class` 写入 `build/java`，随后运行 `cn.t100.mapping.ToolsCheck`。成功时输出包含：

```text
TOOLS_CHECK_OK
```

这不是 APK 构建命令。`build` 已在发布检查的忽略目录中，但通过网页手动上传时仍应自行排除生成物。

## 12. GPU、CPU 与大点云处理

### 12.1 哪些操作可用 GPU

`repair_compute.py` 提供可选 OpenCL 路径，部分点变换、选区计算与半径查询可以尝试在兼容设备上执行。当前底层加载 Windows 的 `OpenCL.dll`，不能据此推断 Linux／macOS 的 GPU 后端已经可用。

```python
from desktop.repair_compute import device_info
print(device_info())
```

返回包含 `available`；不可用时有 `reason`，可用时会报告设备名和部分资源信息。设备可枚举不等于所有计算核已通过数值与性能验收。

修复核心支持：

- `compute_mode='cpu'`：明确使用 CPU 路径，便于建立参考结果。
- `compute_mode='auto'`：符合步骤、点数阈值和资源条件时尝试 GPU；不可用或失败时按实现回退。

当前没有 `compute_mode='gpu'` 的强制模式。自动模式也不会把全部工作迁移到 GPU：KD 树、部分几何分析、文件读写及其他步骤仍可能使用 CPU。请检查结果统计中的计算后端信息，而不是仅看显卡占用。

本仓库不包含 CUDA SLAM、手机 NPU 建图或适用于全部显卡的性能保证。GPU 是否更快，应在同一输入、同一参数下比较总耗时、传输开销和结果一致性。本次源码公开验收没有把特定显卡实机加速作为完成条件。

### 12.2 大地图的资源边界

仅 XYZ float32 坐标每点需要 12 字节：一千万点约占 120 MB（十进制），一亿点约占 1.2 GB。这还没有计入输入副本、布尔掩码、KD 树、邻接图、排序索引、网格和临时工作区。

因此，即使分批读取文件，后续 `np.concatenate`、建树或全局连通性计算仍可能需要大量内存。`repair_resources.py` 提供预算估计与检查，但不意味着任意规模都能处理。

二次开发时建议：

1. 先记录输入点数、包围盒、选区点数、可用内存和磁盘空间。
2. 在小区域和低成本候选上验证参数，避免直接对整幅大地图强力处理。
3. 按算法需求设计分块与邻域重叠；不能把整图任意切块后假定与全局结果完全相同。
4. 将耗时工作放到后台，并保留取消、进度、失败诊断和结果完整性校验。
5. 使用磁盘映射也要统计剩余工作区、随机读写与磁盘容量；它不能消除所有算法开销。

原完整工作台的全部分块会话存储、界面调度与长距离采集逻辑没有在此子集发布。这里没有经过 3～10 公里现场采集的独立工具整体验收。

## 13. 协议、时间、单位与坐标

### 13.1 报文与单位

| 内容 | 协议或接口约定 |
| --- | --- |
| 数据头 | 36 字节 Livox SDK2 式结构；解析器核对版本、长度、类型、样点数量等 |
| 高精度直角坐标 | int32 毫米，解析后转换为米 |
| 低精度直角坐标 | int16 厘米，解析后转换为米 |
| 球坐标距离 | 毫米转米 |
| 球坐标角度 | 0.01 度；`theta` 为相对 +Z 的天顶角，`phi` 为 XY 平面方位角 |
| IMU 陀螺 | rad/s |
| IMU 原始加速度 | g；接入其他算法前核对是否、在哪一层转为 m/s² |
| 设备时间戳 | 纳秒，不等同于主机当前时间 |
| 包内 interval | 0.1 微秒单位，不能当作纳秒直接相加 |
| 修复核心距离参数 | 米，角度参数单独以 `_deg` 标明 |
| `clean_source.radius_cm` | 厘米，是需要特别注意的接口例外 |

原样点序很重要：过滤零回波之后，`point_indices` 保留过滤前索引，不能给剩余点重新编号后当作原采样时刻。否则可能引入逐点时间误差。

### 13.2 三种时间不能混用

- **设备时间：** 用于雷达点与 IMU 的时间对应。
- **主机单调时间：** 用于收包年龄、缓存窗口和断流观测。
- **墙上时间／系统日期：** 用于文件命名或用户日志，不应直接替代前两者。

预览旋转补偿需要覆盖相应时段的 IMU、兼容时间类型和正确外参。覆盖不足、时钟不一致或状态陈旧时，应明确回退，不能把旧姿态持续套用到新点云。

### 13.3 不同“频率”的区别

点云输出包率、有效点率、IMU 频率、采样使能状态、光学扫描周期、机械转速、SLAM 更新率和屏幕刷新率是不同指标。

被动收到点云／IMU，只能确认相应数据正在输出；包率增加、画面更密或显示更流畅，都不能单独证明电机已提高转速。当前公开子集不提供相应控制或转速验证工具。

更多内容见[协议与单位](docs/协议与单位.md)。

## 14. 数据保护与结果复核

### 14.1 对输入和输出的约定

- 将原始点云、原始录包和处理候选分开保存，不用候选覆盖唯一原件。
- 修复核心通过掩码、移动标记与新增数组表达变化，调用者决定是否应用和持久化。
- 文件级清理与模型导出会创建新结果目录，并检查受保护的源地图路径。
- 取消或失败不应当成有效完成；调用者应保留错误状态并复核输出完整性。
- 补点、平面化与建筑面必须保留“推断几何”的含义，不能在说明中改称新增测量。

### 14.2 判断是否有正向效果

单看点数变少、网格更平、残差更低或画面更干净，不足以判断质量提高。建议至少同时比较：

1. 原始／候选在相同视角下的外观和局部剖面。
2. 删除点是否来自真实细节，墙角、台阶、细杆和门窗是否被误删或抹平。
3. 是否产生跨越空区的连接、双层表面或被错误填补的开口。
4. 坐标系、尺度、模型方向以及输入输出包围盒是否符合预期。
5. 删除、移动、新增数量、参数与处理顺序是否可追溯。

### 14.3 分享结果前检查隐私

本仓库初始公开包不附带用户地图、轨迹、录包、照片、设备身份、ADB 记录或签名密钥。你在本机处理数据后生成的新文件不自动继承这一结论。

尤其注意：`filter.json` 和模型 `metadata.json` 等导出说明可能记录输入源路径。对外分享结果前，需要检查这些新生成文件中的路径、现场位置和其他信息。

`check_release.py` 只能检查其定义的模式与文件范围，不是通用脱敏工具，也不能证明不存在所有敏感内容。忽略规则同样不会阻止网页上传或移除已被 Git 跟踪的文件。

## 15. 已验证内容与证据边界

2026-11-24 整理时的离线验证结果如下，详细日志见[验证记录](docs/验证记录.md)：

| 检查 | 结果与覆盖范围 |
| --- | --- |
| Python 环境 | Windows x64；CPython 3.12.10、NumPy 2.5.5、SciPy 1.17.0 |
| Python 回归 | 五组共 27 项通过，涉及报文／单位／坏包／环回接收、修复几何、资源预算、孤点清理与建筑面 |
| 合成示例 | 963 点处理后保留 961 点，两个孤点被移除，原平面逐值保留 |
| Java | JDK 17 按 Java 11 目标编译五个独立组件与测试，输出 `TOOLS_CHECK_OK` |
| 源码整理 | 除已说明的手机 SQLite 读取分支裁剪外，选取的 Python 源码 AST 与整理时原工程一致 |
| 发布清单 | 清单记录受控文件 SHA-256；检查器检测文件变化、未登记文件及规定的禁止入包项 |
| 文档 | 检查仓库内相对文件链接，不把本机历史日志作为公开依赖 |

**没有据此宣称完成的项目：** 真实雷达的全部固件兼容性、每一种依赖版本、所有操作系统、特定 GPU 性能、完整 App、长距离实地采集、测绘级精度和全部隐私／著作权来源的独立审计。

发布清单是当前受控快照的校验工具，不能作为官方数字签名。修改代码或 README 后出现哈希不一致是正常提示；维护者需先审查变更，再同步更新清单，不应为消除报错直接绕过检查。

## 16. 常见问题与排错

### Q1：为什么没有 EXE、APK，不能双击运行？

当前发布的是独立源码工具，不是完整工作台发行版。Python 示例需要 Python 与依赖；Java 示例需要 JDK。完整 App 及其原生 SLAM 集成未纳入。

### Q2：出现 `No module named desktop` 或 `radar_client` 怎么办？

先确认处于仓库根目录，并在当前终端设置 `PYTHONPATH` 指向根目录下的 `python`。不要把路径指到 `python/desktop`。IDE 也需要设置相同的模块搜索路径。

### Q3：出现 `No module named numpy` 或 `scipy` 怎么办？

通常是安装依赖与运行脚本使用了不同 Python。使用同一个 `.venv` 内 Python 执行 `-m pip install -r requirements.txt` 和后续脚本，避免混用系统 Python、其他项目环境与 IDE 解释器。

### Q4：点云或 IMU 一直是零，应该检查什么？

先查雷达供电与链路，再核对雷达实际目的 IP、接收端 IP、UDP 端口、防火墙规则、源地址过滤，以及其他程序是否占用端口。检查点云和 IMU 两条流，不能仅凭其中一条正常推断另一条也正常。这个接收器不会主动让待机设备开始输出。

### Q5：报“地址被占用”或绑定失败怎么办？

停止占用同一端口的工作台、抓取程序或旧脚本。若绑定具体本机 IP，先确认该 IP 确实配置在当前接口。不要把雷达源 IP 当作接收端绑定地址。

### Q6：为什么 PLY／PCD 打不开？

当前读取器只接受前文列出的布局。ASCII、大端、压缩、多属性、自定义字段或带额外网格元素的文件均不能仅凭后缀推断支持。请用可信的点云软件转换为解析器接受的二进制 XYZ 格式，保留原件后再试。

### Q7：为什么手机 `.map`／`map.sqlite` 不能直接导入？

部分手机会话读取依赖未公开的完整集成，当前副本明确拒绝相应入口。请先从原工作台导出受支持的 PLY／PCD。本仓库没有保证所有历史 `.map` 格式均可直接读取。

### Q8：清理后点少了、颜色没了，是不是错误？

去噪会按阈值删除点，需要检查是否误删真实结构。当前文件级清理输出是 XYZ 坐标，不自动保留 RGB、反射率、时间和自定义属性。需要保留属性时，应在自己的管线中维护原行索引与属性对应关系。

### Q9：调平后整个场景移动了，正常吗？

`level` 根据参考面计算刚体变换并作用于整图，这是该接口的定义。它不会只移动选区，也不能把沿行进方向弯曲的道路直接恢复为真实地形。检查 `stats['transform']`，并与原图对照。

### Q10：白模有孔、呈方块状或者没有可导出面怎么办？

先检查输入覆盖、密度、尺度、Z 轴方向与参数。Java 体素外表面本来就可能呈方块状；Python 的局部表面和建筑面采用不同方法。无足够表面证据时不生成有效网格，通常比无依据铺满更合理。

### Q11：能靠一键修复消除所有重影、漂移和地面弯曲吗？

不能。去噪、平面化与补洞是几何后处理。由时空不同步、去畸变、轨迹错误或 SLAM 退化引起的问题，通常需要回到原始数据与建图流程分析。更平整的外观不能单独证明几何更真实。

### Q12：有独显，为什么仍显示 CPU？

可能是当前步骤没有 GPU 实现、点数较小、系统 OpenCL 不可用、资源限制或 GPU 执行回退。先看 `device_info()` 和操作统计。GPU 自动模式不承诺所有步骤、显卡和输入规模均更快。

### Q13：内存不足或处理很慢，直接去掉限制行不行？

不建议直接绕过预算检查。先确认是哪一步增加了内存、点密度和邻接规模，再减少试验范围或设计分块方案。盲目调小间距、增加补点或放宽邻接距离，可能使内存和时间大幅增长。

### Q14：`check_release.py` 报哈希变化，是源码损坏了吗？

不一定。手动修改、Git 换行转换、缺失文件或不同版本的清单都可能导致不一致。先查看报错文件和差异。维护者在确认变更后再更新清单；使用者不要仅为得到 PASS 删除检查逻辑。

### Q15：仓库公开了，能不能拿去商用？

公开可见不等于商业授权。本项目独立代码适用自定义非商用许可；商业产品、交付、服务、企业生产经营等用途需按 LICENSE 另行获得书面许可。第三方依赖的原许可权利不受本仓库改变。

## 17. 许可、第三方依赖与贡献

### 17.1 本项目许可

独立自有代码与新编文档采用 **T100 Non-Commercial Source License 1.0**，标识为：

```text
LicenseRef-T100-NonCommercial-1.0
```

授权允许范围及再分发要求以 [LICENSE](LICENSE) 原文为准。请保留许可、作者与来源说明，并标注修改；不能因为代码能在 GitHub 下载或 Fork，就推断允许商业使用。

本许可证不是 GPL、MIT、Apache 或 OSI 认可的开源许可证。它也不会限制第三方 GPL 算法本来允许的商业使用。

### 17.2 参考与依赖

| 内容 | 来源 | 处理方式 |
| --- | --- | --- |
| 报文布局参考 | [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)，MIT | 参考公开数据头与类型定义，未附厂商固件或 SDK 完整归档 |
| 数组计算 | [NumPy](https://numpy.org/)，BSD-3-Clause 及其声明 | 通过 requirements 安装，不附运行库 |
| 科学计算 | [SciPy](https://scipy.org/)，BSD-3-Clause 及其声明 | 通过 requirements 安装，不附运行库 |
| Java 标准库 | 所安装 JDK 的相应许可 | 不再分发 JDK／Android SDK |
| OpenCL | 系统／厂商实现及相应许可 | 调用系统接口，不附 GPU 驱动 |

FAST-LIO2、Point-LIO、HBA 等未纳入当前仓库，不受这里的非商用条款重新许可。更多来源说明见 [NOTICE.md](NOTICE.md)。

### 17.3 如何反馈问题

欢迎在仓库 **Issues** 提交能够复现的问题，建议包含：

```text
操作系统及架构：
Python／JDK 版本：
NumPy／SciPy 版本：
仓库版本或提交编号：
使用的模块和函数：
输入格式、点数、单位和大致尺度：
完整参数：
复现步骤：
预期结果：
实际结果和异常堆栈：
可公开的最小合成输入：
```

涉及性能时，请补充 CPU、内存、GPU／驱动、输入规模、总耗时和输出统计；涉及网络时补充脱敏后的接口、端口与数据类型。优先提供合成复现，勿直接公开含住址、轨迹、设备身份或账户信息的整份地图／抓包。

提交修改时，请注明代码来源，确认有权按本仓库许可提交，并保留原有声明。不要直接引入与当前非商用发布条件冲突的 GPL／AGPL 集成代码。新增依赖、算法或真实数据应先说明来源与许可。

## 18. 更多文档

| 文档 | 适合解决的问题 |
| --- | --- |
| [源码索引](docs/源码索引.md) | 每个模块的用途、入口与单位 |
| [协议与单位](docs/协议与单位.md) | 报文布局、逐点时间、坐标与 IMU 单位 |
| [工具使用](docs/工具使用.md) | 修复候选、白模和 GPU 接口边界 |
| [验证记录](docs/验证记录.md) | 本次公开子集的运行环境与回归日志 |
| [发布范围与隔离](docs/发布范围与隔离.md) | 完整软件为何没有纳入本次非商用发布 |
| [发布到 GitHub](docs/发布到GitHub.md) | 发布目录、上传与检查方式 |
| [第三方声明](NOTICE.md) | 参考、依赖、商标与原许可 |
| [使用许可](LICENSE) | 个人非商用使用、修改与再分发的具体条件 |

文档更新日期：**2026-11-24**。功能是否可用，以当前仓库实际代码、测试与发布说明为准。
