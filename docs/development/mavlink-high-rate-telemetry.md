# 高频 MAVLink 串口遥测

本实现复用 `FUNCTION_TELEMETRY_MAVLINK`，使用独立硬件 UART。固件需编入
`USE_TELEMETRY_MAVLINK`、`USE_ACC` 和 `USE_GPS`。

## 配置示例

先运行 `serial` 查看端口编号。下例 **1 表示 UART2**，仅在该端口空闲时使用；
将编号替换为实际端口。命令会覆盖该端口的功能配置，不要选接收机、GPS 或 MSP 所用端口。
飞控 TX 接接收端 RX，并共地；若需要运行时调整频率，再接飞控 RX。

```text
feature TELEMETRY
serial 1 512 115200 115200 921600 115200
set mavlink_imu_rate = 250
set mavlink_pos_rate = 10
set mavlink_extra1_rate = 100
save
```

接收端使用 921600 baud、8N1、普通非反相 UART。`512` 是 MAVLink 遥测功能。
`serial` 的四个波特率依次为 MSP、GPS、遥测、Blackbox，以上第三项才是本串口使用的波特率。
AUTO 保留原先的 57600 默认值，不足以承载上述数据流。

| 消息 | 内容 | 默认目标频率 | 配置 |
| --- | --- | --- | --- |
| HIGHRES_IMU (105) | 三轴加速度 m/s²、三轴角速度 rad/s | 250 Hz | `mavlink_imu_rate`，0–500 |
| GPS_RAW_INT (24) | 经纬度、高度、地速、航向及定位质量 | 10 Hz | `mavlink_pos_rate`，0–50 |
| GLOBAL_POSITION_INT (33) | 经纬度、高度、北/东/地向速度 cm/s | 10 Hz | 同上 |
| ATTITUDE (30) | 欧拉角 rad 和三轴角速度 rad/s | 100 Hz | `mavlink_extra1_rate`，0–100 |

设 `mavlink_extra1_rate = 0` 可关闭姿态；设 `mavlink_imu_rate = 500` 可提高 IMU 输出目标频率。
IMU 数值经过飞控校准、安装方向旋转和滤波，使用前/右/下机体系，静止水平时 z 加速度约
-9.80665 m/s²，未扣除重力。未提供磁场、气压、温度，`fields_updated` 仅置六轴位。
时间戳现在来自飞控传感器成功更新后的六轴快照，不是发送时间，也不是硬件同步采样时间。
IMU、GPS 和 TIMESYNC 使用扩展后的 64 位启动微秒时间，处理底层 `micros()` 回绕。
GPS 毫秒字段仍按 MAVLink 标准使用 32 位值，接收端需结合同步时钟扩展。

## 频率与带宽

遥测任务在 MAVLink 启用时以 1 kHz 目标频率调度，消息最短间隔 2 ms。
CLI 频率转换为整数毫秒间隔，因此不能精确表达所有整数 Hz；250/500/100/10 Hz 均可整除。
这些是调度目标，不保证在所有硬件负载下达到；高优先级飞控任务优先执行。
发送最新传感器快照，重复采样不再发送；不缓存每个 IMU 采样，也不会提高传感器自身采样率。

GPS 输出频率不等于新定位频率。若使用支持自动配置的 GPS，可按模块能力设置
`gps_update_rate_hz = 10` 或 `20`，同时调整 `mavlink_pos_rate`；已有 GPS 功能和端口应保持正确配置。
只发送新 GPS 解算；GPS_RAW_INT 和 GLOBAL_POSITION_INT 使用配对快照及同一解算时刻。
POSITION 组还包含已建立的 home/origin 消息。

500 Hz HIGHRES_IMU + 100 Hz ATTITUDE 加上 GPS 和状态消息约需 44 kB/s，
8N1 对应约 440 kbit/s，460800 余量很小，建议使用 921600。
串口缓冲不足时丢弃整个待发包，不阻塞飞控任务，也不发送半包；接收端应统计实际频率和丢包。
共享接收机串口仍遵守现有无线链路流控，高频需求应使用独立 UART。

支持 `MAV_CMD_SET_MESSAGE_INTERVAL`，param1 为消息 ID，param2 为微秒间隔：
2000 = 500 Hz，4000 = 250 Hz，10000 = 100 Hz，-1 = 关闭，0 = 恢复 CLI 默认。
运行时请求不写入持久配置。

本次遥测配置结构版本由 6 升至 7，升级会重置 telemetry 参数组。
升级前保存 `diff all`，升级后检查并重新应用所需遥测设置。

## 验证

```sh
CCACHE_DISABLE=1 LC_ALL=C make TARGET=SITL EXTRA_FLAGS='-DUSE_TELEMETRY -DUSE_TELEMETRY_MAVLINK' -j4
python3 src/test/sitl/mavlink_telemetry_harness.py obj/main/betaflight_SITL.elf
```

测试依赖 pymavlink，使用本机 TCP 5761/5762 和 UDP 仿真端口，检查解码后的 IMU 单位、
GPS 毫秒时间戳、三种数据流频率，以及运行时升频、关闭和恢复。
SITL 按飞控时间统计频率，不能验证物理 UART 吞吐率或实际硬件 CPU 负载。
实际飞控还应在拆除螺旋桨的台架环境下检查 `tasks` 和接收端频率。

本次 SITL 验证结果：默认 IMU 250.0 Hz、姿态 100.0 Hz、GPS 10.0 Hz；
运行时切换 IMU 500 Hz 后测得 500.0 Hz，恢复配置后 249.9 Hz，关闭输出测试通过。
测试同时修复了 SITL TCP 驱动从飞控线程并发调用 dyad 写入导致的包损坏：
TX 现在由网络线程每毫秒检查并排出缓冲。这项修复只影响仿真串口。


## ROS 2 接入版更新

UBX NAV-PVT 保留椭球高和真实定位等级，支持 DGPS/RTK float/fixed。
未知 `alt_ellipsoid` 使用 INT32_MIN（本固件约定，不是通用 MAVLink 哨兵），未知精度扩展使用 0。
没有有效三轴速度时不发送 GLOBAL_POSITION_INT，不以零速度代替。
不提供双天线航向；COG 不等于机头方向。GPS 时间仍含接收机与串口延迟。

伴随端使用独立 `mavlink_sensor_node`，默认请求 500 Hz IMU、10 Hz GPS、关闭姿态；
TIMESYNC 需要接通飞控 RX 和 TX。MSP 控制保持在另一串口。
ROS 配置说明位于 agi_ros2/MAVLINK_SENSORS.md。不得将普通定位伪装为原融合器的 RTK 输入。

时钟单元测试：`make -C src/test CC=gcc CXX=g++ test_mavlink_time_unittest`。
更新后的 SITL 测试使用高采样率静止输入，并验证 50 Hz 传感器源只输出约 50 Hz 新样本。

### 独立航向 topic

ROS 的 `/sensors/fc_heading`（`agi_ros2/msg/Heading`）直接使用
GLOBAL_POSITION_INT.hdg，以 `remainder(pi/2 - hdg*pi/18000, 2*pi)` 转换为
从东逆时针的 ENU 弧度。北为 +pi/2、东为 0、南为 -pi/2、西为 -pi。
UINT16_MAX 或超出 0..35999 的值标记无效，数值为 NaN。
飞控将航向与 GPS 解算一起保存快照，避免使用发送时的新航向配旧时间戳。
其来源是飞控估计的 attitude.yaw，不是 COG 或双天线 RTK 航向；该 topic 不输入现有融合器。
它随 GLOBAL_POSITION_INT 输出，通常为 GPS 更新频率，与可选 100 Hz ATTITUDE 分开。
