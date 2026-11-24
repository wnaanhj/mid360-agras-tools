# Mid 360 农机版 · 非商用源码公开工具集

面向 DJI T100 农业版激光雷达研究的独立点云工具，整理自电脑版和 Android 建图工作台。
**允许个人自用、学习及非商业研究；未经另行书面许可，不得商用。** 具体授权以 [LICENSE](LICENSE) 为准。

> 这是“源码可见的非商用公开版”，不是 OSI 意义上的开源项目，也不是完整建图软件安装包。
> FAST-LIO2、Point-LIO、JNI/ctypes 集成和完整桌面/手机 App 暂未纳入，不能从此仓库构建完整建图程序或 APK。
> 第三方 GPL 代码本身允许商用，不能靠这个仓库的非商用条款改变它们的权利。

## 可以做什么

| 内容 | 当前提供 |
| --- | --- |
| 雷达点云/IMU协议 | 独立 UDP 数据报解析、单位换算、时间/点序保留、被动接收缓存 |
| 预览运动补偿 | Python/Java 的 IMU 旋转补偿与时序检查；用于显示，不是 SLAM 定位 |
| 点云清理与几何修复 | 半径/统计去噪、碎片清理、地面识别、刚体调平、局部平整、平面与补洞候选 |
| 白模 | Python 有证据的局部/建筑面构造，Java 占用体素外表面；支持 OBJ/GLB 导出 |
| GPU辅助 | 独立 Python OpenCL 点运算接口；依赖系统 OpenCL 驱动，CPU 工具可独立使用 |
| Java工具 | 纯 JDK 点云工具、撤销/重做、运动预览、Ethernet/ARP/IPv4/UDP 帧工具 |
| 质量与测试 | 合成输入、回归测试、原数据保护和边界检查；不附个人地图或真实抓包 |

源代码已带中文注释。白模/补洞生成的是推断几何，不能当作雷达实测点。体素大小、点数、残差或测试通过都不等于绝对精度保证。

## 目录

```text
.
├── README.md / LICENSE / NOTICE.md
├── docs/                      # 协议、源码索引、使用范围和发布说明
├── python/
│   ├── desktop/               # 独立修复、白模、文件读写与可选OpenCL
│   ├── radar_client/          # 被动收包与预览运动计算
│   └── windows/               # 沿用原项目命名的离线回归
├── java/
│   ├── src/cn/t100/mapping/   # 纯JDK独立组件，不是Android App
│   └── test/cn/t100/mapping/  # 点云/白模/撤销重做验收
├── examples/                  # 只用合成点云的示例
├── tools/                     # Java验证与发布文件检查
├── requirements.txt
└── SOURCE_MANIFEST.json       # 相对路径、来源与SHA-256
```

## Python快速验证

建议 Python 3.12+，在仓库根目录运行（Windows PowerShell）：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
$env:PYTHONPATH = (Join-Path $PWD 'python')
.\.venv\Scripts\python.exe -B -m unittest discover -s python/windows -p 'test_*.py' -v
.\.venv\Scripts\python.exe -B examples/repair_synthetic.py
.\.venv\Scripts\python.exe -B tools/check_release.py
```

Linux/macOS 可用对应 Python/venv，设置 `PYTHONPATH="$PWD/python"` 后运行相同测试；本次发布整理只在 Windows 验证，不承诺跨平台全部路径均已实测。
依赖范围在 requirements.txt，整理时使用的本机实际版本及结果见[验证记录](docs/验证记录.md)。不附带 Python 运行库或预装环境。

测试中的 UDP 只绑定本机环回地址的临时端口并发合成数据，不发送真实雷达控制命令。示例不连接雷达、不写入地图；需要处理自己的点云时参考[工具使用](docs/工具使用.md)。

## Java快速验证

需要 JDK 17（按 Java 11 目标编译），不需要 Android SDK 或手机：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/Test-Java.ps1
```

脚本只编译本目录中的五个独立组件和测试，结果写入 `build/java`，随后运行 `cn.t100.mapping.ToolsCheck`。该步骤不生成 APK，也不连接 OTG 设备。

## 硬件与接收约定

原工作台的参考设备为 AG901LD10 / T100 农业版雷达。雷达独立供电；不要将网线、USB/OTG 当作雷达供电方案。
参考网络是雷达 `192.168.1.10`、接收端 `192.168.1.20/24`，UDP `60001` 点云、`60003` 内置 IMU；地址应以自己的设备配置为准。
接收端不能与其他设备重复使用 IP，同一端口也不要被两个接收程序占用。

本公开子集只提供被动数据接收，不包含 ROI 全周恢复、采样、待机或固件工具。
“T100 农业版”的控制和兼容性不能直接等同于普通 Livox MID-360；本项目不代表 DJI/Livox 官方。

## 许可和完整软件的边界

- [LICENSE](LICENSE)：独立自有代码和新编文档采用自定义非商用条款。
- [NOTICE.md](NOTICE.md)：第三方格式参考与依赖来源；依赖仍保留各自许可。
- [发布范围与隔离](docs/发布范围与隔离.md)：为何完整桌面/安卓建图软件没有作为非商用成品发布。
- [源码索引](docs/源码索引.md)：每个文件的用途、关键接口和单位。

未附带用户地图、录包、PCAP、现场照片/截图、设备身份、ADB记录、厂商固件/驱动、APK、DLL/SO、运行库、签名密钥或上游完整归档。
也没有给第三方算法重新添加禁商用条款。需要完整建图功能时，应先解决组合许可和历史客户端来源，再发布独立完整版本。

## 反馈与贡献

请附脱敏的最小合成输入、依赖版本、操作步骤和异常信息。地图/轨迹/照片/抓包可能包含位置、设备和个人信息，请勿直接贴整份现场数据。
提交修改时注明来源、保留已有署名，并确认自己有权按本仓库非商用条款提交；不要引入与该条款冲突的 GPL/AGPL 代码。
