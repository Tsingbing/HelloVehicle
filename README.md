# HelloVehicle

基于 ArduPilot 的最小应用示例，用于学习和验证参数管理、MAVLink 通信及日志记录。
源码位于 `XueVehicle/`，构建目标为 `HelloVehicle/XueVehicle`。
本仓库依赖 ArduPilot 的库和工具链，不能独立编译。

## 接入 ArduPilot

在 ArduPilot 工程根目录执行：

```bash
git clone https://github.com/Tsingbing/HelloVehicle.git modules/HelloVehicle
```

如果目录已存在，使用已有仓库即可。

ArduPilot 默认不会自动扫描 `modules` 中的应用。在工程根目录的
`wscript` 中，找到 `_build_recursion()`，在 `dirs_to_recurse.sort()`
之前添加一次：

```python
dirs_to_recurse.append('modules/HelloVehicle/XueVehicle')
```

这样会直接加载 `XueVehicle/wscript`，不需要本仓库顶层的 `wscript`。
普通克隆不会自动将本仓库注册为 ArduPilot 的 Git 子模块。

## 编译 FMUv2 固件

准备好 ArduPilot 编译环境及依赖后，在 **ArduPilot 工程根目录**执行：

```bash
./waf configure --board fmuv2
./waf build --target HelloVehicle/XueVehicle
```

生成的固件为：

```text
build/fmuv2/bin/XueVehicle.apj
```

当前版本已在本地通过 FMUv2 编译验证；编译通过不代表已完成硬件运行验证。

## SITL 编译与运行

在 ArduPilot 工程根目录执行：

```bash
./waf configure --board sitl
./waf build --target HelloVehicle/XueVehicle
./build/sitl/HelloVehicle/XueVehicle -M rover --console
```

以上为当前目标名称对应的 SITL 命令，本次未验证 SITL 运行。
切换回硬件固件时，重新执行 `./waf configure --board fmuv2`。

## 源码说明

| 文件 | 用途 |
| --- | --- |
| `XueVehicle/Hello_Vehicle.cpp`、`Hello_Vehicle.h` | 应用入口、初始化、主循环和日志记录 |
| `XueVehicle/Parameters.cpp`、`Parameters.h` | 参数定义及参数组注册 |
| `XueVehicle/GCS_Hello.cpp`、`GCS_Hello.h` | MAVLink 通信适配和心跳发送 |
| `XueVehicle/wscript` | 设置程序名 `XueVehicle` 和分组 `HelloVehicle` |

应用提供以下示例参数：

| 参数 | 默认值 |
| --- | --- |
| `XUE_GAIN` | `1.0` |
| `XUE_SPEED` | `2.5` |
| `XUE_ENABLE` | `1` |
| `XUE_TEST` | `42` |

固件通过 USB（SERIAL0）提供 MAVLink 心跳和参数读写，可通过 Mission Planner
连接对应串口，在完整参数列表中查看和修改 `XUE_*` 参数。

构建产物使用 `XueVehicle` 名称，C++ 应用类仍叫 `Hello_Vehicle`。
