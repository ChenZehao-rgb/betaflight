# 气压遥测与 ROS 2 融合接入

本固件通过独立的 `SCALED_PRESSURE`（消息 29）回传温度补偿后的绝对气压。
它来自 `baro.pressure`，没有融合 GPS/IMU，也没有减去地面参考气压或飞控的高度零点。
MS5611 驱动内部的 ADC 计数 `ms5611_up/ms5611_ut` 不作为 ROS 融合输入。

**本文的 ROS 接收与 EKF 部分是后续实施步骤；本次只修改 Betaflight，未修改
`agilicious_internal-main`。收到压力消息不会自动使现有 ROS EKF 融合气压。**

## 1. 固件接口

需要编入 `USE_BARO`、`USE_TELEMETRY`、`USE_TELEMETRY_MAVLINK`，并检测到气压计。
原有 MAVLink 专用 UART、921600 baud、TIMESYNC 接线保持现有配置；配置方法见
[高频遥测说明](mavlink-high-rate-telemetry.md)。飞控 TX/RX 与伴随端 RX/TX 交叉连接并共地。

| 字段 | 固件值 | ROS 转换 |
| --- | --- | --- |
| `time_boot_ms` | 压力读取完成后的启动时钟，64 位微秒时钟转换为 32 位毫秒 | 用 TIMESYNC 和 `bootMs()` 映射到 ROS 时间 |
| `press_abs` | `baro.pressure / 100.0f`，hPa | 乘 100 得到 Pa |
| `temperature` | `baro.temperature`，0.01 ℃ | 除 100 得到 ℃ |
| `press_diff` | 0，本固件没有差压观测 | 不作为差压或空速输入 |
| `temperature_press_diff` | 0，没有差压温度 | 不发布为有效温度 |

单位遵循 [MAVLink SCALED_PRESSURE 定义](https://mavlink.io/en/messages/common.html#SCALED_PRESSURE)。
时间戳对应驱动读取和补偿完成的时刻，不是 ADC 开始转换、转换中点或硬件同步采样时刻；
MS5611 的压力转换等待仍构成传感器延迟，后续应测量再决定是否补偿。

默认发送检查间隔为 25 ms，即目标 40 Hz。只发送新、有效且年龄不超过 250 ms 的样本，
不重复发送上次压力；压力必须大于零，温度必须可由消息的 `int16_t` 字段表示。
MS5611 当前温度/压力转换及状态机周期名义约 25 ms，实际接收频率取决于采样、调度和链路；
提高消息请求频率不会提高 MS5611 采样频率。气压消息独立于 GPS 是否定位、GPS 更新频率和
`HIGHRES_IMU` 消息。压力采样序号用于固件内部去重，不在标准消息 29 中传输。

支持 `MAV_CMD_SET_MESSAGE_INTERVAL`（命令 511）：

| `param1` | `param2` | 行为 |
| --- | --- | --- |
| 29 | 25000 | 请求 40 Hz，间隔单位为微秒 |
| 29 | 50000 | 请求 20 Hz |
| 29 | -1 | 停止气压遥测 |
| 29 | 0 | 恢复固件默认 40 Hz |

这是运行时请求，不写入持久配置。本次没有新增 CLI 参数或改变 telemetry 参数组版本。
检查 `COMMAND_ACK`，再测量收到的新样本频率；
ACK 通过只证明请求被接受，不证明传感器存在或板上达到该频率。

固件源文件：

- [barometer.c](../../src/main/sensors/barometer.c)：`baroUpdate()` 完成读数并生成快照。
- [barometer.h](../../src/main/sensors/barometer.h)：压力和温度结构。
- [MS5611 驱动](../../src/main/drivers/barometer/barometer_ms5611.c)：ADC 转换、补偿和延时。
- [mavlink.c](../../src/main/telemetry/mavlink.c)：气压快照、消息 29 发送、去重、时效及间隔命令处理。

## 2. ROS 接收节点

下列路径均位于 `/home/sia/agilicious_internal-main`。接入点是
`agi_ros2/src/mavlink_sensor_node.cpp` 的构造函数、`configureRates()`、`handle()`、
`resetEpoch()` 和 `diagnose()`；无需另开进程争用 `/dev/ttyAMA2`。

### 2.1 参数与请求列表

1. 声明只读参数 `baro_rate_hz`，默认 40，0 表示关闭；拒绝非法频率。
   在 `agi_ros2/config/hardware.yaml` 的 `mavlink:` 节点加入 `baro_rate_hz: 40`。
   当前 `runtime_launch.py` 将此字典传给节点，因此 YAML 路径可直接传递；如需 launch
   参数覆盖，还要在 `DEFAULTS` 和频率覆盖列表增加相应参数。
2. `configureRates()` 当前的 `ids/hz` 数组有 4 项，完成条件及 `rate_commands_ok`
   也写死为 4。改为统一的消息 ID/频率请求表，追加 29 和 `_baro_hz`，所有边界与完成条件
   使用表的 `.size()`；同步检查 `_command_index`、重试和 ACK 前进逻辑。
3. 例如请求表可表达为下面的片段。它只展示数据结构，成员声明、初始化、诊断等仍需配套修改：

   ```cpp
   const std::array<std::pair<int, int>, 5> requests{{
       {MAVLINK_MSG_ID_HIGHRES_IMU, _imu_hz},
       {MAVLINK_MSG_ID_GPS_RAW_INT, _gps_hz},
       {MAVLINK_MSG_ID_GLOBAL_POSITION_INT, _gps_hz},
       {MAVLINK_MSG_ID_ATTITUDE, _attitude_hz},
       {MAVLINK_MSG_ID_SCALED_PRESSURE, _baro_hz},
   }};
   ```

   建议把该表作为成员统一使用，避免发送路径和诊断分别维护不同的数量。
   `baro_rate_hz: 0` 应请求 `-1` 关闭，而不是发 `0` 恢复默认。

### 2.2 解码、单位和时间

包含 `sensor_msgs/msg/fluid_pressure.hpp`，创建 `sensors/baro/pressure` 发布者，采用
`rclcpp::SensorDataQoS()`。可选包含 `sensor_msgs/msg/temperature.hpp`，发布
`sensors/baro/temperature`。当前 CMake/package 已依赖 `sensor_msgs`。

在 `handle()` 已有的源 ID 检查、时钟同步和发布者冲突检查之后加入消息 29 分支。
核心转换如下；这是接入片段，尚不是完整可编译补丁：

```cpp
mavlink_scaled_pressure_t value{};
mavlink_msg_scaled_pressure_decode(&m, &value);
const double pressure_pa = static_cast<double>(value.press_abs) * 100.0;
if (!std::isfinite(pressure_pa) || pressure_pa <= 0.0) return;
sensor_msgs::msg::FluidPressure pressure;
if (!stamp(MAVLINK_MSG_ID_SCALED_PRESSURE, bootMs(value.time_boot_ms),
           pressure.header.stamp)) return;
pressure.header.frame_id = "baro_link";
pressure.fluid_pressure = pressure_pa;
pressure.variance = _baro_pressure_variance_pa2;
_pressure_pub->publish(pressure);
```

`bootMs()` 将 32 位毫秒扩展到最近的 TIMESYNC 时钟周期，`stamp(29, ...)` 沿用现有时钟映射、
重复/倒序检查。必须使用独立键 29；禁止改用 `now()` 为旧样本赋新采样时间。
当前 `stamp()` 对非 IMU 消息统一允许到达年龄至 0.5 s；应为气压加入有依据的独立时效阈值，
并与融合端的延迟窗口一致，不能仅依赖该宽松默认值。

`baro_link` 应有实际安装位置对应的静态 TF；它是标量压力，不做 FRD/FLU 轴翻转。
若同时发布温度，复用刚生成的 `pressure.header`，`temperature = value.temperature * 0.01`，
不要再次调用 `stamp(29, ...)`，否则同一时间戳会被当成重复样本拒绝。

`FluidPressure.variance` 单位是 Pa²，0 表示方差未知；`Temperature.variance` 单位是 ℃²。
可配置压力方差并由静态数据估计，不能把 0 当作零噪声传给 EKF。参见
[FluidPressure 定义](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/sensor_msgs/msg/FluidPressure.msg)
和 [Temperature 定义](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/sensor_msgs/msg/Temperature.msg)。
MS5611 温度用于芯片补偿，不能直接视为机外空气柱的平均温度。

### 2.3 状态与重连

为压力增加频率计数、最新接收的单调时钟、样本年龄、压力有效性和发布者冲突诊断。
更新 `diagnose()` 中的名字/计数数组及其大小。`resetEpoch()` 清空压力有效标记及缓存，
重发包含消息 29 的请求列表；时钟未同步时停止发布。

标准 `FluidPressure` 没有 session 或有效性字段。因此必须另外传递气压源会话/同步状态
（专用诊断或伴随状态消息），使融合节点在飞控重启、断线、ROS 时钟跳变时明确清除旧气压队列。
仅停止发布并不能立即通知融合器一次会话已经结束。

## 3. 气压转换为本地高度

保留 `GPS_RAW_INT.alt` 和现有 `altitude_source: msl` 的 GNSS 海拔语义。
气压单独形成 ENU 向上为正的 z 观测，不能装入 `Navigation.altitude` 伪装成 GNSS。

在压力稳定、飞机静止且 ROS 本地导航原点已建立后，用同一时间段的压力估计参考 `p0`，
保存该时段对应的本地高度 `z0` 和参考不确定度。用于短距离相对高度的标准大气近似为：

```text
A = 44330 m
k = 0.190295
h_baro = A * (1 - (p / p0)^k)
z_baro = z0 + h_baro
```

压力降低时 `h_baro` 增大。`p`、`p0` 必须使用相同单位；此处均使用 Pa。
这只是气压到相对高度的模型，不能直接产出可靠的 MSL 或椭球高；天气、气流、安装位置和温升
会造成偏置。参考零点必须与 ROS `odom` 原点一致，初始化时已有的 `z0` 不一定为零。

最少要实现带不确定度的参考对齐。长期融合建议增加气压偏置状态 `b_baro`：

```text
z_baro = z_imu + b_baro + noise
b_baro_dot = random_walk
```

GNSS 高度提供长期约束，气压提供较快的相对高度变化。若考虑传感器安装杆臂，
观测模型中的 `z_imu` 改为 `z_imu + (R_body_to_enu * r_imu_to_baro).z`，并更新雅可比。
初始 `b_baro` 的协方差应包含参考对齐不确定度；`p0` 的误差是跨样本相关误差，
不能当作每个 40 Hz 样本的独立白噪声反复融合而虚假缩小协方差。

将压力方差传播到高度方差：

```text
dh/dp = -(A * k / p0) * (p / p0)^(k - 1)
R_height = (dh/dp)^2 * R_pressure + R_model
```

`R_height` 的单位为 m²。海平面附近约 1 Pa 对应 0.083 m 的高度变化，实际灵敏度随参考压力变化。
`R_model` 用于未建模的快速干扰，慢偏置由 `b_baro` 及其过程噪声承担；避免把同一误差重复计入。
不通过对气压高度简单差分再同时融合来制造独立的垂直速度观测。

气压参考随导航坐标原点/会话显式管理，不跟随 FC ARM/DISARM 清零。飞控重启、ROS 源时钟
epoch 改变、导航 session 改变或 EKF 重新初始化时，清空旧观测并记录新的参考/偏置初始化原因。
若需要保持原来的导航坐标，则重新对齐压力偏置并增加不确定度，不能静默把当前高度改为零。

## 4. 接入现有 EKF 的顺序

现有硬件入口是 `agi_ros2/src/state_fusion_node.cpp`：`onNavigation()` 保存 GNSS 观测，
`onImu()` 调用 `EkfImu::addImu()`、`addRtk()` 和 `getAt()`。
对应库位于 `agilib/include/agilib/estimator/ekf_imu/ekf_imu.hpp` 和
`agilib/src/estimator/ekf_imu/ekf_imu.cpp`，目前没有气压观测接口。

### 4.1 先解决不同传感器的到达顺序

当前 `addRtk()` 要求 `t > t_posterior_`，成功后丢弃较早的 IMU；`getAt()` 可以从最近后验
沿 IMU 预测到当前时刻，但没有保存任意旧后验及所有观测的完整历史。
所以直接在压力回调调用一次新 `addBaro()` 会出错：若压力先把后验推进到 10.10 s，
随后到达的 10.05 s GNSS 会被视为过旧，即使它本来处于允许的 GNSS 延迟范围。

第一版可以实现有界观测排序缓冲：

1. 在融合层分别接收并保存所有 GNSS/气压观测，按采样时间排序，附带会话和唯一观测标识。
   当前 `_rtk` 仅保存最新导航观测，延迟缓冲应改为队列，不能只新增气压队列。
2. IMU 到来后先 `addImu()`，仅处理时间早于 `最新 IMU 时间 - 延迟窗口` 且已有 IMU 覆盖的观测，
   严格按时间依次更新后验。延迟窗口根据实测 GNSS、压力及调度延迟选择，记录超窗丢弃。
3. 同一时间戳的不同传感器需支持同一预测点的多观测更新，或形成联合更新；不能沿用
   `t <= t_posterior_` 对所有同时间戳观测一概拒绝。去重按传感器和观测标识处理。
4. `getAt(当前 IMU 时间)` 从较旧但经过校正的后验预测当前状态。历史长度必须覆盖该窗口，
   增大 `MAX_QUEUE_SIZE` 本身不能修复观测乱序；需记录内存、CPU 和预测跨度。

若要低延迟地立即融合先到观测，再接纳稍后到达的更早观测，则需要真正的历史回放：
保存有界的后验状态、协方差、偏置、IMU 和原始观测；退到目标观测之前的检查点，插入观测后
按采样时间重做所有传播及测量更新，再预测到当前。必须同时处理历史裁剪、同时间戳、
NIS 重新计算、失败回滚和避免诊断计数重复累加，不能只重放 IMU 而漏掉中间 GNSS/气压修正。

### 4.2 新增独立的一维气压更新

1. 在 `state_fusion_node.h/.cpp` 新增压力订阅、参考状态、采样/接收时间及观测队列，
   增加启用参数、压力有效范围、最大年龄、噪声与一维 NIS 参数；验证有限值和正方差。
   在初始化成功且参考有效后才投递气压观测，不能以气压替代现有导航/姿态初始化条件。
2. 在 `EkfImu` 新增类似 `addBaro(t, z_baro, variance, nis_limit)` 的接口。
   气压应有独立的接受/拒绝计数、创新、NIS、最后接受时间，不能更新 `_last_rtk_time`
   或累计 GNSS 就绪次数，也不能将气压健康等同于 `navigation_ready`。
3. 若使用偏置状态，在内部 `IDX` 的 16 维状态后增加一维 `BARO_BIAS`；同步扩展
   `StateVector/StateMatrix`、状态初始化、`Q_/Q_init_`、状态转换和回放快照。
   参数定义及加载分别在 `ekf_imu_params.hpp/.cpp`，节点重建 EKF 的 `reset()` 路径也需覆盖。
   当前过程噪声实现为 `dt² * Q_`；若新偏置噪声定义为连续随机游走谱密度，应独立按
   `q_baro * dt` 离散化，不能直接把谱密度塞入现有 `Q_` 当作同义参数。
4. 复用现有传播和数值保护，在观测时刻使用一维模型：

   ```text
   residual = predicted_z + predicted_b_baro - z_baro
   H[POSZ] = 1, H[BARO_BIAS] = 1，其余为 0（无杆臂时）
   S = H * P * H^T + R_height
   NIS = residual^2 / S
   K = P * H^T / S
   corrected = prior - K * residual
   ```

   使用一维 NIS 门限，不能照搬 GNSS 6/7 维门限。延续 Joseph 协方差更新、对称化、
   四元数归一化及其协方差雅可比；失败时恢复更新前的时间、状态和协方差。
5. 在 `reset()`、导航 session 切换、时钟跳变和压力源失效路径清空相应缓存。
   缺气压时按明确策略退回既有 IMU/GNSS 融合并发布退化原因；不要沿用最后一个压力不断更新。

## 5. 验证顺序

先实现接收、记录，再实现融合；首次运行使用既有 `shadow_only` 路径计算和记录，禁止实机接管。

1. 飞控端确认 `status` 检测到 MS5611，`tasks` 的气压/遥测任务正常。
   在专用 MAVLink 端口查看消息 29；静止时压力应为正且量级合理，缓慢抬高时压力下降。
   检查默认、20 Hz、关闭和恢复请求；统计不同 `time_boot_ms` 的数量，确认没有重复采样冒充高频。
2. ROS 接收改完后检查 `ros2 topic hz /sensors/baro/pressure` 和消息内容，核对
   Pa、Pa²、温度转换、时间戳单调性、TIMESYNC RTT、年龄及重启/断线恢复。
   相同压力数值可以来自不同的新采样，去重依据应是时间/样本标识，不是压力数值变化。
3. 扩展 `agi_ros2/test/test_mavlink_sensor.py` 的串口仿真，覆盖消息 29 请求/ACK、
   1013.25 hPa → 101325 Pa、2500 → 25 ℃、重复/过旧消息、32 位毫秒回绕和时钟重建。
4. 扩展 `agilib/tests/estimator/ekf/ekf_imu_rtk_test.cpp` 或新增相邻测试，覆盖单位/方向、
   一维 NIS、同时间戳、GNSS 晚于气压到达、延迟窗口边界、压力阶跃/丢失/慢偏置、
   会话重置，以及历史回放与按采样时间排序输入的一致性。检查协方差有限、对称及半正定。
5. 录制 `/sensors/baro/pressure`、可选温度、`/sensors/imu`、`/sensors/navigation`、
   `/sensors/local_navigation`、`/sensors/mavlink/status`、融合状态及新增气压诊断。
   对同一段静止和缓慢升降记录做 IMU/GNSS 基线与加入气压的离线对比，检查高度/垂直速度、
   创新、NIS、拒绝率、延迟和参考重置；没有真值时只能报告漂移/重复性，不能宣称绝对精度改善。

构建通过、消息解码通过和实际融合效果是三种不同证据。该固件改动不替代板上频率、
物理串口吞吐、气流干扰及实际运行负载验证。

## 6. 本次固件验证记录

- `CCACHE_DISABLE=1 LC_ALL=C make CONFIG=GEPRC_TAKER_H743 -j4`：通过，生成
  `obj/betaflight_2026.6.1_STM32H743_GEPRC_TAKER_H743.hex`；未刷写飞控。
- `make -C src/test CC=gcc CXX=g++ test_mavlink_time_unittest test_baro_ms5611_unittest`：
  1 项时钟测试和 7 项 MS5611 测试通过。
- 按[高频遥测说明](mavlink-high-rate-telemetry.md)构建 SITL 后运行
  `python3 src/test/sitl/mavlink_baro_harness.py obj/main/betaflight_SITL.elf`：通过。
  默认气压 40.0 Hz、20 Hz 请求实测 19.9 Hz，单位、温度、独立启停、去重、无效值恢复通过。
  请求 500 Hz 时虚拟气压计输出 81.1 Hz，只证明输出受新样本限制，不代表 MS5611 可达该频率。
- 原有 `mavlink_telemetry_harness.py` 首次在 500 Hz 请求下测得 467.7 Hz，低于原断言；
  同进程气压开关对照未复现持续瓶颈，且一次降频同时影响姿态/GPS。保留原断言重跑后完整通过，
  500 Hz 请求实测 500.5 Hz。此结果不能证明物理串口的 500 Hz 吞吐。
- 250 ms 过期保护已通过代码审查，尚未独立注入总线卡死来验证；板上采样频率和 ROS 融合未验证。
