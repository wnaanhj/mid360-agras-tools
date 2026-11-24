# 来源、参考与第三方依赖

本公开目录仅选取可独立运行、不依赖原生SLAM库的项目工具源码。源文件保留原中文注释；选择清单、原相对位置和校验值见 SOURCE_MANIFEST.json。来源审查未发现这些候选文件的GPL/JNI调用或未知来源声明，不构成对所有著作权来源的独立法律证明。

| 内容 | 来源/许可 | 本目录处理 |
| --- | --- | --- |
| 点云报文布局参考 | [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2) 与设备协议观察；SDK2 为 MIT | 独立实现解析，没有复制SDK库/固件；参考36字节数据头及类型定义，农业版兼容性仍需实测 |
| NumPy | [numpy.org](https://numpy.org/)，BSD-3-Clause及其第三方声明 | requirements依赖，不附wheel/运行库 |
| SciPy | [scipy.org](https://scipy.org/)，BSD-3-Clause及其第三方声明 | requirements依赖，不附wheel/运行库 |
| Java标准库 | 用户所安装JDK的相应许可 | 不再分发JDK/Android SDK |
| OpenCL | 系统厂商实现及其相应许可 | 仅通过系统接口调用，不附GPU驱动/运行库；本项目点计算核与SLAM滤波无关 |

未纳入的FAST-LIO2、Point-LIO、HBA等第三方项目不受本仓库非商用许可约束。它们的许可仍允许相应的商业使用；仅将这些项目放到另一个目录并不能解决与非商用成品组合的兼容问题。

本项目不包含制造商的授权背书。DJI、Livox、MID-360、Android等名称用于说明设备/平台兼容性，商标归各自所有者。
