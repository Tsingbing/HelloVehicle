# XueVehicle

XueVehicle 是基于 ArduPilot 库的最小应用，用于学习和验证参数管理、MAVLink
通信与日志记录。构建产物名为 `XueVehicle`，C++ 应用类仍为 `Hello_Vehicle`，
通过 `AP_HAL_MAIN_CALLBACKS` 执行 `setup()` 和 `loop()`。

本目录依赖完整的 ArduPilot 工程，不能独立编译。当前未实现车辆控制、
飞行模式或任务执行；MAVLink 中的地面车类型仅用于标识应用。

## 源码结构

| 文件 | 用途 |
| --- | --- |
| `Hello_Vehicle.h` / `Hello_Vehicle.cpp` | 应用对象、初始化、主循环和日志写入 |
| `Parameters.h` / `Parameters.cpp` | 参数类型、默认值及参数组注册 |
| `GCS_Hello.h` / `GCS_Hello.cpp` | MAVLink 通道适配、参数收发和心跳 |
| `wscript` | 定义程序名 `XueVehicle`，构建分组为 `HelloVehicle` |
| `Makefile` | 遗留 make 入口；引用的 `../../mk/apm.mk` 当前不可用，请使用 waf |

## 构建与运行

以下命令均在 **ArduPilot 工程根目录**执行，需提前准备对应工具链及依赖。

根目录 `wscript` 的 `_build_recursion()` 需要包含以下注册项，当前工作区已添加，
无需重复添加：

```python
dirs_to_recurse.append('modules/HelloVehicle/XueVehicle')
```

### FMUv2 固件

```sh
./waf configure --board fmuv2
./waf build --target HelloVehicle/XueVehicle
```

固件输出路径：

```text
build/fmuv2/bin/XueVehicle.apj
```

### SITL

```sh
./waf configure --board sitl
./waf build --target HelloVehicle/XueVehicle
./build/sitl/HelloVehicle/XueVehicle -M rover
```

需要使用终端代替 TCP 串口时，可添加 `--console`；通过网络连接地面站时
不要添加该选项。切换回硬件构建时，重新执行对应板型的 `configure`。

上述目标和输出路径依据当前构建脚本整理；本次文档更新未重新编译或验证运行。

## 参数

应用通过 `AP_Param` 加载默认值和已保存值。`Parameters g` 保存顶层参数，
`ParametersG2 g2` 中的 `Xue xue` 通过 `AP_SUBGROUPINFO` 注册为 `XUE_` 参数组。

| 参数 | 类型 | 默认值 | 元数据声明范围 | 用途 |
| --- | --- | --- | --- | --- |
| `XUE_GAIN` | `AP_Float` | `1.0` | 0～10 | 示例增益 |
| `XUE_SPEED` | `AP_Float` | `2.5` | 0～100 m/s | 示例速度 |
| `XUE_ENABLE` | `AP_Int8` | `1` | 0：禁用，1：启用 | 示例开关值 |
| `XUE_TEST` | `AP_Int32` | `42` | -2147483648～2147483647 | 示例整数 |

`XUE_GAIN`、`XUE_SPEED`、`XUE_ENABLE` 现在参与软件虚拟速度控制，
不连接真实执行器。控制器检查速度、增益范围及开关值；无效参数使输出归零。
`XUE_ENABLE` 不控制主循环或日志开关，`XUE_TEST` 仍用于参数和日志演示。

顶层还注册了 `FORMAT_VERSION`、`LOG_BITMASK` 和 `BRD_` 参数组。
启用日志编译选项时注册 `LOG` 参数组；启用空速编译选项时注册 `ARSPD`
参数组，但当前应用未实现空速传感器初始化与周期更新。

## MAVLink 与 Mission Planner

启用 `HAL_GCS_ENABLED` 时，应用初始化串口和 GCS，并在通道 0 上处理接收数据、
发送参数及约每秒发送一次心跳。系统 ID 为 `1`，类型为地面车，状态为待机。
参数读写复用 ArduPilot 的标准 MAVLink 处理逻辑。

在 FMUv2 上可按以下步骤检查参数：

1. 刷入 `build/fmuv2/bin/XueVehicle.apj`，通过 USB 连接板卡。
2. 在 Mission Planner 中连接对应串口。
3. 打开完整参数列表，搜索 `XUE_`。
4. 修改参数并写入，重启后重新读取，检查保存结果。

## 日志行为

启用 `HAL_LOGGING_ENABLED` 时，应用启动会将 `LOG_BACKEND_TYPE` 设置并保存为
`1`（文件后端），并强制允许未解锁状态记录日志。因此，手动修改的其他后端
配置会在下次启动时被覆盖。

日志后端通过主循环中的 `logger.periodic_tasks()` 异步启动。
检测到日志开始后，应用写入一次 `Hello_Vehicle started` 消息，随后约每秒
写入一条 `XUE` 记录：

| 字段 | 内容 |
| --- | --- |
| `TimeUS` | 启动后的微秒时间戳 |
| `Gain` | 当前 `XUE_GAIN` |
| `Speed` | 当前 `XUE_SPEED` |
| `Enable` | 当前 `XUE_ENABLE` |
| `Test` | 当前 `XUE_TEST` |

文件日志需要可用的存储介质和文件系统。当前写入逻辑没有根据
`LOG_BITMASK` 判断是否记录 `XUE`，因此不要将其视为该记录的开关。

# 学习路线：接下来添加哪些代码

第 1～3 阶段的基础实现已加入程序；PID 和第 4～7 阶段仍是后续练习。
建议先把参数、计算、
通信和日志连起来，每完成一步都通过日志验证，再接入更多库。

### 1. 周期任务与时间统计

在 `Hello_Vehicle` 中增加 `update_control()`、`update_status()` 等任务函数，
先使用时间戳按不同频率调用，例如控制任务 50 Hz、状态记录 1 Hz。
保留现有 GCS 更新和日志后台处理。

- 用 `AP_HAL::micros64()` 记录实际调用间隔和任务执行耗时，两者分别记录。
- 增加 `TIME` 日志，包含时间戳、控制任务间隔、控制任务耗时。
- 使用无符号时间差判断周期，避免将计数器回绕误判为长时间停顿。

原先的 `hal.scheduler->delay(10)` 不保证循环恰好为 100 Hz，循环内的处理也会
占用时间。验收时查看实际周期、抖动和耗时，理解阻塞调用对其他任务的影响。

### 2. 用现有参数实现虚拟速度控制

建议将控制计算放入新增的 `control.cpp`，先在 SITL 中使用软件模型，
让现有参数真正影响程序行为：

| 参数 | 建议用途 |
| --- | --- |
| `XUE_SPEED` | 目标速度，单位 m/s |
| `XUE_GAIN` | 比例控制增益 |
| `XUE_ENABLE` | 是否允许控制器产生驱动力 |

下面是逻辑示意，需要补充成员变量、初始化和参数校验后才能使用：

```cpp
error = target_speed - simulated_speed;
output = enabled ? constrain_float(gain * error, -1.0f, 1.0f) : 0.0f;
simulated_speed += (output * acceleration_gain - drag * simulated_speed) * dt;
```

`dt` 使用实测任务间隔并换算为秒；首次调用应初始化时间戳，对异常大的
间隔应有明确处理。`acceleration_gain` 和 `drag` 是虚拟模型系数。
关闭控制时输出归零，虚拟速度在阻力作用下逐渐下降。
参数注释中的 `@Range` 不代替运行时校验，使用前需要检查有效范围。

增加 `CTRL` 日志，记录 `TimeUS,Target,Actual,Error,Output,Dt`。
通过地面站修改目标速度和增益，观察响应速度、稳态误差及输出限幅；
关闭 `XUE_ENABLE` 后确认输出归零。这一步串起：

```text
地面站修改参数 → AP_Param → 控制计算 → 虚拟速度模型 → AP_Logger
```

随后参考 [AC_PID 示例](../../../libraries/AC_PID/examples/AC_PID_test/)，
尝试将比例控制替换为 PID，观察积分项对稳态误差的影响，并学习控制器复位。

### 3. 增加简单模式管理

在新增的 `mode.cpp` 中实现 `STOP`、`MANUAL`、`AUTO` 三种模式，以及统一的
`set_mode()` 入口。初期可通过新增的学习参数请求切换模式。

- `STOP`：输出为零。
- `MANUAL`：使用受限的测试输入直接指定输出，后续替换为 RC 输入。
- `AUTO`：运行虚拟速度闭环。
- 切换时校验进入条件、复位控制器状态，并记录旧模式、新模式和切换原因。
- 将 `GCS_Hello::custom_mode()` 接到实际模式状态；自定义编号要写入文档，
  地面站不一定能自动显示自定义模式名称。

使用 `logger.Write_Message()` 记录低频事件，用结构化日志记录周期变量。
验收时确认每个模式的输出符合定义，非法模式请求不会改变当前状态。
参考 [Rover 模式实现](../../../Rover/mode.h) 和
[模式切换入口](../../../Rover/mode.cpp)。

### 4. 接入一个传感器

建议先学习气压计，将初始化、周期更新和健康检查放入新增的 `sensors.cpp`。
参考 [BARO_generic 示例](../../../libraries/AP_Baro/examples/BARO_generic/)，
理解 `AP_Baro` 前端与具体设备后端的分工。

记录气压、高度和健康状态，验证数据是否随环境或仿真状态变化。
初始化失败或数据不健康时，应记录事件并避免继续使用无效数据。
当前 `ARSPD` 仅注册了参数组；注册参数不等于完成传感器初始化和采样。

### 5. RC 输入与失效处理

参考 [RCInput 示例](../../../libraries/AP_HAL/examples/RCInput/) 和
[RC_Channel 示例](../../../libraries/RC_Channel/examples/RC_Channel/)，
学习原始输入、校准、死区、归一化及有效性判断。

将一个通道映射到 `MANUAL` 模式的虚拟输出，记录原始值和归一化结果。
增加输入超时处理：失效后进入 `STOP`，记录原因，并明确恢复输入后是否
需要重新请求模式。验收时模拟输入中断，确认输出按预期归零。

### 6. 使用 AP_Scheduler 组织任务

在手动周期调度已能运行后，参考
[Scheduler_test 示例](../../../libraries/AP_Scheduler/examples/Scheduler_test/)
和 [Rover 主循环与任务表](../../../Rover/Rover.cpp)，接入 `AP_Scheduler`。

将传感器、控制、通信和日志任务整理成任务表，学习任务频率、优先级和
执行时间预算。注意 `hal.scheduler` 属于 HAL 调度接口，和
`AP_Scheduler` 不是同一个对象；接入时需要理解示例的主循环节拍来源，
不能只复制任务表。

对比接入前后的 `TIME` 日志，检查任务频率及耗时；在 SITL 中加入可控的
耗时任务，观察执行预算不足时的行为。

### 7. 最后学习 INS、AHRS 和完整车辆结构

基础链路稳定后，再参考
[INS_generic 示例](../../../libraries/AP_InertialSensor/examples/INS_generic/)
和 [AHRS_Test 示例](../../../libraries/AP_AHRS/examples/AHRS_Test/)，
学习 IMU 采样、单位、坐标系、校准和姿态估计。

先记录加速度和角速度，再接入姿态解算，观察滚转、俯仰和航向变化。
这些模块具有初始化顺序和更新频率要求，应沿示例逐项理解依赖。
之后对照 [AP_Vehicle](../../../libraries/AP_Vehicle/) 和
[Rover](../../../Rover/)，理解完整车辆如何组织公共基础设施与车辆专用逻辑。

### 建议的文件组织和完成标准

按学习进度逐步拆分文件：`control.cpp` 放控制和虚拟模型，`mode.cpp` 放模式
管理，`sensors.cpp` 放传感器，`Log.cpp` 放日志写入函数；在
`Hello_Vehicle.h` 中声明对应成员和方法。`Hello_Vehicle.cpp` 保留初始化和
任务组织，参数统一在参数相关文件维护。

每一步完成后，都应能回答：输入来自哪里、何时更新、输出被谁使用、
失败时如何处理，以及日志如何证明行为正确。优先完成第 1、2 步，
得到一个可调参、可观测的闭环实验，再逐步扩展。

### 第 1、2 阶段：当前实现与验收

- `control.cpp` 实现比例控制和虚拟速度模型，约 50 Hz 更新；不驱动硬件输出。
- `Log.cpp` 每次控制更新写入 `TIME` 和 `CTRL`，原有 `XUE` 保持约 1 Hz。
- `TIME.Interval` 是实际控制间隔，`TIME.Elapsed` 是控制计算耗时，均为微秒；
  耗时不包含日志写入和 GCS 处理。
- `CTRL` 记录目标、更新前速度、误差、输出、秒单位的 `Dt` 和有效标志 `Valid`。
- 间隔超过 0.1 秒时输出归零并跳过本次模型积分；下一次正常更新可自动恢复。
- 参数无效时输出归零，正常时间步内虚拟速度仍按阻力衰减。

建议运行并保存日志，按以下顺序验收：

1. 设置 `XUE_MODE=2`（AUTO），其余保持默认参数，检查 `TIME.Interval` 接近 20000 微秒，`CTRL.Valid` 为 1。
2. 默认目标 2.5 m/s、增益 1 时，速度应趋近约 2.27 m/s；比例控制在阻力下
   存在稳态误差，这是后续学习积分项的对照。
3. 修改 `XUE_SPEED` 和 `XUE_GAIN`，观察响应变化，确认输出始终在 [-1, 1]。
4. 设置 `XUE_ENABLE=0`，确认输出归零、速度逐渐下降，日志仍持续记录。
5. 设置超范围参数，确认 `Valid=0` 且输出为零；恢复有效参数后确认恢复控制。

`logger.Write_Message("Hello_Vehicle started")` 对应 `.BIN` 中的 `MSG` 文字记录。
FMUv2 默认文件位于 SD 卡 `/APM/LOGS/`；SITL 默认位于启动工作目录的 `logs/`。
`TIME`、`CTRL` 和 `XUE` 用于查看数值曲线，`MSG` 用于查看事件文字。
这些验收步骤需要实际运行后检查日志，编译成功不能代替运行验收。

本阶段已通过 `./waf build --target HelloVehicle/XueVehicle` 的 FMUv2 构建；
尚未进行板上运行和日志验收。

### 第 3 阶段：模式管理

`mode.cpp` 提供统一切换入口，控制任务每次更新时检查 `XUE_MODE` 是否变化。
模式参数可以持久化；启动时先为 STOP，首次控制更新处理保存的模式请求。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `XUE_MODE` | 0 | 请求模式：0=STOP、1=MANUAL、2=AUTO |
| `XUE_MANUAL` | 0 | MANUAL 模式的归一化驱动力，范围 [-1, 1] |

STOP 始终输出零，模型速度按阻力衰减；MANUAL 直接使用手动驱动力；AUTO
使用目标速度和比例增益。`XUE_ENABLE=0` 在所有模式下都使驱动力归零。
切换保留模型速度，清除控制输出、目标和误差，随后按新模式计算。

进入 MANUAL 时校验手动输入和开关；进入 AUTO 时校验速度、增益和开关。
STOP 无参数进入条件。非法编号或不满足进入条件时保留原模式。
请求只在参数值变化时处理：被拒绝后，应修正相关参数，再将 `XUE_MODE`
改为 0，等待一次控制更新后重新请求目标模式。运行中输入变为无效时，
当前模式保留但输出归零，输入恢复后可继续运行。

`CTRL.Mode` 持续记录实际模式。`XMOD` 记录旧模式、请求编号、实际模式、
原因（0=参数请求）和是否接受；同时写入一条 `MSG` 文字记录。
日志后端启动之前发生的切换不会补写事件，可从后续 `CTRL.Mode` 查看状态。

心跳 `custom_mode` 使用实际模式编号，并设置自定义模式有效标志。
这些编号是本程序的学习约定，不是 Rover 的模式编号；地面站可能显示
不同名称，请以原始编号及日志为准。当前通过参数切换，未增加地面站
标准模式切换命令的处理。心跳仍报告未解锁、待机状态。

验收步骤：

1. 设置 `XUE_MODE=0`，确认 `CTRL.Mode=0`、输出为零。
2. 设置 `XUE_MANUAL=0.3`、`XUE_ENABLE=1`，再设置 `XUE_MODE=1`，
   确认输出为 0.3，速度逐渐趋近 3 m/s。
3. 设置 `XUE_MODE=2`，确认开始速度闭环；切换前后模型速度连续。
4. 设置 `XUE_MODE=99`，确认模式不变，`XMOD.Accepted=0`，且不会每周期重复记录。
5. 回到 STOP，设置 `XUE_MANUAL=2` 后请求 MANUAL，确认请求被拒绝；
   修正为 0.3 后按上述方式重新请求，确认切换成功。
6. 在 MANUAL 或 AUTO 中设置 `XUE_ENABLE=0`，确认输出归零且模式保持不变。

### IMU 接入：采样与日志

已通过 `AP_InertialSensor` 接入板载 IMU，代码位于 `sensors.cpp`。
参数加载、板级初始化和 GCS 初始化完成后调用 `ins.init(100)`，由板型配置
选择传感器后端。启动时保持板卡静止，供默认的陀螺仪零偏校准使用。
注册 `INS` 参数组，沿用库内的校准和滤波参数命名。

主循环通过 `ins.wait_for_sample()` 和 `ins.update()` 以约 100 Hz 节拍更新，
替代末尾固定延时；传感器硬件内部采样率可以更高。原有控制任务现由 `AP_Scheduler` 以约 50 Hz 调用，
时间差只用于计算实际控制步长。IMU 数据参与 DCM 姿态估计，不参与虚拟速度控制。

每次控制记录同时为每个检测到的传感器实例写入一条 `XIMU`：

| 字段 | 含义 |
| --- | --- |
| `TimeUS` | 写入时的启动时间，微秒，不是硬件采样时间 |
| `I` | 从 0 开始的传感器实例编号 |
| `AX,AY,AZ` | INS 输出的加速度，m/s²，包含重力对应的比力 |
| `GX,GY,GZ` | INS 输出的角速度，rad/s |
| `AH,GH` | 加速度计、陀螺仪健康状态，1=健康，0=无效或缺失 |

数据经过驱动旋转、校准与滤波，不是芯片原始寄存器数值；使用默认板体
坐标方向（前、右、下）；接入 AHRS 后可通过 `AHRS_ORIENTATION` 设置板卡安装朝向。不健康数据在日志中
填零，必须结合健康标志判断，不能将零解释为有效测量。健康不等于已完成
加速度计校准，本阶段未接入完整六面校准交互。

板上验收：

1. 上电静置，等待初始化和陀螺仪校准结束。
2. 打开 SD 卡 `/APM/LOGS/` 下本次 `.BIN`，查看 `XIMU`，按 `I` 区分实例。
3. 正常静置时确认 `AH=GH=1`，角速度接近零，加速度模长接近 9.8 m/s²；
   正向水平放置时 AZ 通常接近 -9.8 m/s²。
4. 缓慢绕各轴转动，观察对应角速度和加速度分量变化。
5. 同时检查 `TIME`、`CTRL`，确认采样接入后控制和日志继续运行。

未检测到 IMU 时，库的初始化可能进入传感器错误处理，程序未必能进入
主循环生成 `XIMU`，需要检查板型配置和启动诊断。当前仅完成 FMUv2 构建
验证，实际采样、健康状态及日志内容仍需板上验收。

### Mission Planner 实时姿态与 IMU

在 IMU 基础上接入 `AP_AHRS` 的 DCM 姿态解算，约 100 Hz 调用
`ahrs.update(true)`；`true` 表示本循环已更新 INS，避免重复获取采样。
罗盘初始化后约 10 Hz 读取，用于辅助航向。GPS 对象作为 AHRS 依赖存在，但尚未初始化或更新，不提供定位；
气压计已按下文接入，可提供气压高度。

本学习程序启动时选择 DCM（`AHRS_EKF_TYPE=0`），并在内存中禁用 EKF2/3，
避免启动尚未配套完整车辆基础设施的导航滤波器。此阶段保持 DCM 配置。
新增 `AHRS_`、`COMPASS_` 参数组，可检查板卡安装方向、罗盘偏置等配置。
罗盘机载校准现已接入，见下文；加速度计六面校准尚未接入。健康标志不能代替校准结果。

GCS 通道 0 每 100 ms 尝试发送下列标准 MAVLink 消息；发送前检查串口缓冲区
空间，链路繁忙时实际频率会降低。当前是固定周期发送，不能通过地面站的
流速请求修改这两处发送周期。

| 消息 | 内容 | 用途 |
| --- | --- | --- |
| `ATTITUDE` | 滚转、俯仰、航向（rad）和角速度（rad/s） | Mission Planner HUD 姿态显示 |
| `RAW_IMU` | 实例 0 的加速度（milli-g）、角速度（mrad/s）及磁场 | 实时 IMU 数值查看 |

`RAW_IMU` 复用 ArduPilot 的标准发送函数，其数值是经过驱动处理的测量值，
不是芯片 ADC 原始计数。消息本身不包含本程序的 `AH/GH` 健康标志；
判断传感器有效性时仍应对照 `XIMU` 日志。

使用步骤：

1. 刷入 `build/fmuv2/bin/XueVehicle.apj`，上电保持静止，等待陀螺仪校准结束。
2. Mission Planner 连接飞控 USB 串口，打开飞行数据中的 HUD。
3. 缓慢改变板卡滚转和俯仰，检查人工地平线随之变化；不需要解锁，
   `XUE_MODE=STOP` 也会持续发送姿态。
4. 如 HUD 不变化，使用地面站的 MAVLink 消息检查功能确认是否收到
   `ATTITUDE`，再检查字段是否随转动变化；只有心跳并不代表收到姿态。
5. 本地 `.BIN` 新增 `XATT`，记录以度为单位的 `Roll,Pitch,Yaw` 和
   AHRS 的 `Healthy` 标志，可与实时显示交叉检查。

没有有效罗盘时，航向主要依赖陀螺仪积分，会漂移，不能作为准确的绝对航向。
持续加速度也会影响 DCM 倾角估计。当前实现用于台架学习和显示，尚未接入
位置估计或真实车辆姿态控制。已通过 FMUv2 构建，尚未进行实板和 Mission
Planner 联调。

### AP_Scheduler 接入

现已用任务表替代主循环中的手工周期判断。`Hello_Vehicle::loop()` 只调用
`scheduler.loop()`，由调度器等待 INS 采样、推进 tick、计算剩余预算并执行任务。
`update_imu()` 不再重复调用 `wait_for_sample()`，只更新 INS 和 AHRS。

注册了 `SCHED_` 参数组，本程序将 `SCHED_LOOP_RATE` 默认值设为 100 Hz；
已有保存值仍然生效。INS 使用调度器的实际配置频率初始化，修改循环频率
需要重启。建议本阶段保持 100 Hz，避免改变现有任务频率和预算假设。

任务表位于 `Hello_Vehicle.cpp`，低数值优先级先执行：

| 任务 | 频率 | 预算（微秒） | 优先级 |
| --- | --- | --- | --- |
| `update_imu`：INS 和 AHRS | 每个主循环 | 快任务，按循环周期统计 | 0 |
| `update_compass` | 10 Hz | 1000 | 6 |
| `control_task`：控制和 TIME/CTRL 日志 | 50 Hz | 1000 | 9 |
| `gcs_task`：MAVLink 更新 | 100 Hz | 1500 | 12 |
| `logger_task`：日志后台与启动消息 | 100 Hz | 1000 | 15 |
| `sensor_log_task`：XIMU/XATT | 50 Hz | 1500 | 18 |
| `write_log`：XUE 参数记录 | 1 Hz | 500 | 21 |
| `scheduler.update_logging`：性能统计维护 | 0.2 Hz | 1000 | 24 |

GCS 和日志任务受对应编译开关控制。预算是初始估计，仍需实板测量。
普通任务剩余预算不足时可能被推迟，运行超时不会被抢占中断；快任务每轮执行。
任务频率按循环 tick 量化，不能超过主循环频率，非整除配置可能改变实际频率。
罗盘更新发生在本轮 AHRS 之后，新磁场数据供下一轮 AHRS 使用。

控制器继续使用真实调用间隔作为 `dt`，超过 0.1 秒时仍按原逻辑归零输出并
跳过模型积分。`TIME.Elapsed` 只统计控制计算，调度器的控制任务耗时还包含
TIME/CTRL 日志写入。未分配标准 PM 日志位（初始化传入 -1），因此不会因
此次接入自动记录标准 PM 日志；现有 TIME 日志继续提供控制周期观测。

验收建议：

1. 确认 `SCHED_LOOP_RATE=100` 后重启，查看 TIME 间隔是否接近 20000 微秒。
2. 检查 XUE 约 1 Hz、XIMU/XATT 约 50 Hz，Mission Planner 姿态仍持续更新。
3. 在 AUTO 和 MANUAL 间切换，确认模式与输出行为保持一致。
4. 可用 `SCHED_DEBUG=2` 观察任务推迟，`SCHED_DEBUG=3` 同时观察超时诊断；
   这些输出走 HAL console，不等于 Mission Planner 的实时消息。频繁打印
   会扰动运行时间，测量完成后恢复为 0。

本次已通过 FMUv2 构建，尚未实板验证任务耗时和链路更新频率。

### 气压计功能

已启用现有 `AP_Baro barometer` 对象，注册 `BARO` 参数组。初始化在
`sensors.cpp` 的 `init_baro()` 中完成：先 `init()` 探测板型支持的后端，
再 `calibrate()` 等待传感器稳定并校准地面气压。校准会更新库内相关参数，
启动期间保持板卡静止、处于固定高度，等待校准结束后再观察数据。
校准有阻塞等待，因此只在 setup 中执行，不放入调度任务。

任务表新增 `update_baro`，频率 10 Hz，优先级 7，初始预算 1000 微秒
（包含日志写入），排在罗盘之后、控制之前。每次调用更新气压计，并为
每个实例记录一条 `XBAR`：

| 字段 | 含义与单位 |
| --- | --- |
| `TimeUS` | 写入时间，启动后微秒 |
| `I` | 从 0 开始的实例编号 |
| `Press` | 绝对气压，Pa |
| `Temp` | 传感器温度，℃，不应直接当作环境温度 |
| `Alt` | AP_Baro 输出的气压高度，m |
| `Healthy` | 1=健康；0=无效，此时该条数值字段填零 |

默认场地海拔和高度偏置为零时，校准后的气压高度应接近启动位置的零高度；
改变相关 BARO 参数会改变高度基准。气压高度会受到天气、温度和气流影响，
不是测距传感器给出的离地距离。

通道 0 约 10 Hz 发送实例 0 的标准 `SCALED_PRESSURE` 消息，发送前检查
健康状态和缓冲区空间。`press_abs` 单位为 hPa，`temperature` 为 0.01 ℃，
与 XBAR 的气压单位不同。传感器不健康时停止此消息的周期发送，地面站
可能保留旧值，应结合接收时间及 XBAR 健康标志判断。
这条消息携带气压和温度，不直接携带 HUD 高度；本次未增加 HUD 高度消息。

验收步骤：

1. 上电保持静止，等待 IMU 和气压计校准结束，再连接 Mission Planner。
2. 在 MAVLink 消息检查功能中查看 `SCALED_PRESSURE` 是否持续更新。
3. 打开本次 `.BIN` 的 XBAR，确认 Healthy=1，气压、温度合理，启动高度
   接近配置基准。缓慢抬高板卡，观察 Alt 上升、Press 下降。
4. 同时检查 XATT、TIME 和模式切换，确认气压计接入后其他任务继续运行。

未检测到气压计或无法获得健康样本时，库可能进入配置错误处理，无法进入
正常主循环；应检查板型与启动诊断。尚未进行实板测量和地面站联调。

### 单向偏航漂移诊断

罗盘更新任务新增 10 Hz 日志，复用 Compass 状态接口，不修改融合算法：

- `XCST`：Count=检测数量，Healthy=当前可用罗盘健康状态，UseYaw=库判定
  是否可用于航向，Cal=是否正在校准。没有设备时仍记录此状态。
- `XCMP`：每个实例的 MX/MY/MZ 和 Norm（磁场，mGauss）、AgeMS（距最后
  更新的毫秒数）、Healthy、UseYaw、Config（库的 configured 检查结果）。
  不健康时磁场字段保留旧值，因此必须结合 AgeMS 和 Healthy 使用。
- 实例 UseYaw 仅反映使用配置等条件，不包含实例健康检查；XCST.UseYaw
  才包含当前可用罗盘的健康判断。Config=1 也不能证明现场没有磁干扰。

静置记录约一分钟，结合 XATT.Yaw 和 XIMU 的角速度检查：Count=0 表示
未检测到罗盘；Healthy=0 或 AgeMS 持续增长表示数据链路需要检查；
XCST.UseYaw=0 时检查 COMPASS_USE 和 COMPASS_LEARN。库的飞行中学习模式
会暂时禁止罗盘用于航向，本学习程序未接入完整飞行学习流程。
Cal=1 时 DCM 暂停航向修正。UseYaw=1、Cal=0 只是修正的前提条件，不能
单凭该值证明磁场数据正确或航向已稳定。

先在远离磁性物体的位置静置完成启动校准，再收集日志。尚未根据实板日志
定位漂移原因，本次增加的是诊断能力，不代表漂移已经修复。

### Mission Planner 罗盘机载校准

本程序复用 ArduPilot 的 Compass 校准实现：

- GCS_MAVLINK 已处理 `MAV_CMD_DO_START_MAG_CAL`、
  `MAV_CMD_DO_ACCEPT_MAG_CAL` 和 `MAV_CMD_DO_CANCEL_MAG_CAL`，包括命令应答。
- 任务表新增 `Compass::cal_update()`，100 Hz、优先级 8、预算 200 微秒，
  负责校准状态维护及按请求自动保存。拟合计算使用库内校准线程。
- 通道 0 以约 5 Hz 尝试发送 `MAG_CAL_PROGRESS` 和 `MAG_CAL_REPORT`，
  由库检查状态和发送空间；校准停止后仍发送可用结果，以便地面站接受。
- 以上代码受 `COMPASS_CAL_ENABLED` 控制。

操作：刷入新固件，等待启动完成，保持 `XUE_MODE=0`，连接 Mission Planner，
在罗盘设置中启动机载校准（Onboard Mag Calibration）。按进度缓慢转动板卡，
让各个方向都得到采样，远离钢制桌面、磁铁和大电流导线。校准失败时查看
结果和现场环境，不要用强制接受或放宽阈值代替排查。

成功后按地面站提示接受结果；是否自动保存、重试或重启由地面站请求参数
和库内流程决定。重新上电读取参数，确认校准值已保存，检查 XCMP.Config=1，
再静置两分钟记录新日志。校准过程中 XCST.Cal=1，DCM 暂停磁航向修正，
此时的偏航变化不能用于评估静置稳定性。取消操作同样走标准库命令。

若提示优先级变化需要重启，应重启后重新校准。若提示不支持或没有进度，
检查固件版本、命令应答和 MAG_CAL 消息，而不是只检查 HUD。
本次仅构建验证，尚未实板验证校准完成、参数保存及漂移改善。

### 中文参数元数据

`XueVehicle.apm.pdef.xml` 使用 UTF-8 和 ArduPilot pdef XML 结构，包含
8 个应用参数（6 个 XUE 参数和 2 个顶层参数），以及 INS 主参数表的中文
显示名称和说明。保留源码中的范围、单位、只读标志和数值编码；未修改
固件参数或上游库注释。文件包括受编译条件控制的参数，列出参数不代表
当前固件或硬件一定支持它。

这是局部中文元数据，不是完整固件参数全集：未包含 INS 的陷波、批量日志、
温度校准等子参数组，也未翻译 Compass、AHRS、BARO、LOG 等其他库参数。
应用参数使用 `XueVehicle:` 前缀，INS 参数位于 libraries 节点。

Mission Planner 不会仅凭这个自定义文件名自动加载它；当前程序还报告地面车
类型，地面站通常按自己的车型键查询。要用于地面站，需要按实际版本的
加载规则，将这些条目合并进匹配车型的元数据，并调整应用参数车型前缀。
应保留原有其他参数定义，不能用本局部文件覆盖完整参数库。当前只验证了
XML 结构和中文覆盖，未验证 Windows Mission Planner 加载。

### GCS 收发分离

任务表现在以 100 Hz 依次调用 GCS::update_receive（优先级 12）、
GCS_Hello::update_telemetry（13）、GCS::update_send（14）。移除旧 gcs_task
及其嵌套收发调用。自定义遥测仍在通道 0 发送姿态、IMU、气压和校准消息；
库内接收与发送遍历已建立的通道。心跳交由库内发送调度处理。
本应用的标准流表为空，因此通道 0 显式注册 MSG_NEXT_PARAM 的 100 ms
发送周期，参数队列由 update_send 处理，不再直接调用 queued_param_send。
请板上验证完整参数下载、修改参数、HUD 更新和罗盘校准进度。

### 标准遥测流（替代自定义 update_telemetry）

当前任务表只保留 GCS::update_receive 和 GCS::update_send，已移除
update_telemetry 及所有自定义发送计时器。GCS_Hello.cpp 参照 Rover 定义
all_stream_entries，并带有 MAV_STREAM_TERMINATOR 结束标记。
RAW_SENS 默认 10 Hz（RAW_IMU、SCALED_PRESSURE），EXTRA1 默认 10 Hz
（ATTITUDE），EXTRA3 默认 5 Hz（罗盘校准进度和结果），PARAMS 默认
10 Hz（参数队列）。心跳由标准发送初始化设置。地面站可通过标准流速
或消息周期命令调整发送频率，实际速率受链路和调度预算影响。
尚未将通道参数注册成 SR0_ 等顶层参数组，不能通过这些参数持久保存流速。
标准 SCALED_PRESSURE 发送器使用实例 0 的已有测量值，不再使用之前的
自定义健康门控；仍须结合 XBAR.Healthy 和数据更新时间判断有效性。
程序现在自行定义 XueVehicle V0.1.0-dev 固件版本，避免链接进
GCS_Dummy 的空流表。需要实板验证完整参数下载、姿态和罗盘校准流程。

### RC 输入观测（第一阶段）

RC_Hello.h 定义 RC_Channel/RC_Channels 子类；radio.cpp 复用库的
RC_Channels_VarInfo.h 注册 RC1_MIN/MAX/TRIM/REVERSED/DZ 等参数。
BoardConfig 初始化后调用 init_radio；50 Hz read_radio 调用 read_input。
各通道暂统一使用中心对称的 ±1000 控制范围，默认死区 30 微秒，尚未按
油门、转向等功能映射，不用于改变模式或输出，XUE_MANUAL 继续生效。

新增 10 Hz 日志：XRCS 记录通道数、距最近成功输入更新的毫秒数及 Fresh；
从未收到输入时 AgeMS=4294967295。XRC 按从 1 开始的通道号记录 PWM
（微秒）和 Control（库转换后的控制值）。Fresh 表示 500 ms 内有成功更新，
不是接收机失效判定；接收机保持旧值或输出预设失控值时可能仍有数据。
read_input 返回 false 只表示本轮未成功更新，不能直接等同于失联。
本阶段没有启用 RC 模式开关、辅助功能、解锁或车辆失效保护逻辑。

标准 STREAM_RC_CHANNELS 默认 10 Hz 发送 RC_CHANNELS，并在 MAVLink1
链路发送 RC_CHANNELS_RAW；地面站可请求修改消息频率。确认实际接收机
输出模式和接线后，移动摇杆，检查地面站通道值和 XRC 日志，再关闭遥控器
观察接收机的实际输出行为。校准最小值、中位、最大值后检查方向和死区。
尚未实板验证接收机输入，不能据此确认 R12F 的协议或接线正确。
