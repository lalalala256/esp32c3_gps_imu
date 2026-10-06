# esp32c3_gps_imu

ESP32-C3 SuperMini 采集固件：同时采集 **GNSS / BMI160(加计+陀螺) / QMC5883L(磁力计) / 按键**
四路数据，统一时间基准，按数据源分别写入 TF 卡 CSV，并镜像到 USB 串口。
目标是为后续**组合导航（INS/GNSS 融合）**提供干净、带时间戳、无丢帧的原始数据。

本版本**只做采集**，不包含姿态解算 / EKF / 融合算法。
另带一个 **WiFi 网页文件浏览**（自带 SoftAP，见第 6 节），方便在手机上查看和下载 TF 卡里的数据。

---

## 1. 硬件接线

| 模块 | 信号 | ESP32-C3 GPIO |
|---|---|---|
| TF 卡 (SPI) | SCK | GPIO4 |
| | MISO | GPIO5 |
| | MOSI | GPIO6 |
| | SS / CS | GPIO7 |
| I2C 总线 | SDA | GPIO8 |
| | SCL | GPIO9 |
| BMI160 | INT1 (data-ready) | GPIO0 |
| | INT2 | GPIO1 |
| 按键 | KEY1（日志开关） | GPIO2 |
| | KEY2（事件标记） | GPIO3 |
| GNSS | PPS | GPIO10 |
| | GNSS_TX → ESP RX | GPIO20 |
| | GNSS_RX ← ESP TX | GPIO21 |

### ⚠️ 引脚/板级注意事项

1. **控制台已改到 USB Serial/JTAG**（`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`）。
   GPIO20/21 是 UART0 的默认引脚，必须让出来给 GNSS，否则会冲突。
   GNSS 走 **UART1 重映射**到 GPIO20(RX)/GPIO21(TX)。
2. **GPIO4–7 复用为 MTMS/MTDI/MTCK/MTDO**。占用为 SPI 后，**引脚 JTAG 不可用**；
   GPIO18/19 的 USB-JTAG 仍可正常调试/烧录。
3. **GPIO2 / GPIO8 / GPIO9 是 strapping 引脚**，上电时必须为高：
   - KEY1(GPIO2) 用内部上拉，**上电瞬间不要按住 KEY1**；
   - SDA/SCL 依赖上拉电阻（本项目已额外打开 i2cdev 内部上拉作为兜底，
     `CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS=y`）；
   - GPIO9 同时是 **BOOT 键**，按下即进入下载模式。
4. **部分 SuperMini 克隆板把板载 RGB 灯接在 GPIO8**。若 I2C 扫描不到 0x0D / 0x68，
   优先怀疑这里（见第 8 节排查）。
5. **GPIO0/1 同时是 XTAL_32K_P/N**。本项目 32 kHz 源用内部 RC，无冲突。
6. **ESP32-C3 没有 SDMMC 控制器，且只有 1 个 I2C 外设**：
   TF 卡必须走 SDSPI（`SPI2_HOST`）；BMI160 与 QMC5883L 必须共用
   `I2C_NUM_0`，引脚与速率一致。
7. I2C 工作在 **400 kHz**：BMI160 驱动默认 1 MHz，QMC5883L 上限 400 kHz，
   本项目统一覆盖为 400 kHz（`CONFIG_APP_I2C_FREQ_HZ`）。
   **400 kHz 需要模块自带外部上拉电阻**（常见 QMC5883L/BMI160 模块都有）。

---

## 2. 构建与烧录

需要 ESP-IDF **v5.5**（本工程在 v5.5.1 上验证通过）。

```bash
source /home/nuc/github/esp-hosted/esp_hosted_ng/esp/esp_driver/esp-idf/export.sh
cd /home/nuc/github/esp32c3_gps_imu

idf.py set-target esp32c3      # 首次：同时下载组件依赖
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

依赖（由 ESP Component Registry 自动下载到 `managed_components/`）：

| 组件 | 用途 |
|---|---|
| `esp-idf-lib/i2cdev` | I2C 总线/设备抽象（多设备共享） |
| `esp-idf-lib/bmi160` | BMI160 驱动，含 INT1 data-ready 中断 |
| `esp-idf-lib/qmc5883l` | QMC5883L 驱动（输出 mG） |

> 注意：`/dev/ttyACM0` 是 **USB Serial/JTAG**，不是 UART 桥。退出 monitor 用 `Ctrl+]`。

---

## 3. 数据文件

每次上电生成 **一套新的 4 个文件**，索引 `idx` 由扫描卡上现有 `imu_*.csv` 取 `max+1`
（不依赖 NVS，也绝不覆盖旧数据）：

```
/sdcard/imu_0007.csv
/sdcard/mag_0007.csv
/sdcard/gnss_0007.csv
/sdcard/events_0007.csv
```

每个文件开头是 `#` 元数据行 + 列名行，可直接用 `pandas.read_csv(..., comment='#')`：

| 文件 | 列 |
|---|---|
| `imu_*.csv` | `t_us, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, valid` |
| `mag_*.csv` | `t_us, mx_mG, my_mG, mz_mG, valid` |
| `gnss_*.csv` | `t_us, pps_t_us, pps_seq, utc_date, utc_time, lat_deg, lon_deg, alt_msl_m, geoid_sep_m, speed_mps, course_deg, fix_type, fix_mode, sats, hdop, valid` |
| `events_*.csv` | `t_us, src, event` |

`events` 的取值：`BOOT` / `LOG_START` / `LOG_STOP` / `MARK` /
`SD_MOUNTED` / `SD_MOUNT_FAIL` / `SD_ERROR` / `SD_REMOUNT` / `SENSOR_MISSING`。
`src` 为 `SYS` / `KEY1` / `KEY2`。

默认数据率：IMU 100 Hz、磁力计 100 Hz、GNSS 1 Hz（每次有效 UTC 秒 1 行 + PPS）。
IMU CSV 约 **4 KB/s**（≈ 14 MB/小时）。

---

## 4. 按键功能

| 按键 | 行为 |
|---|---|
| **KEY1**（GPIO2） | 开始 / 暂停记录（写 `LOG_START` / `LOG_STOP` 行） |
| **KEY2**（GPIO3） | 打事件标记（写 `MARK` 行） |

暂停期间**只停止 IMU/磁/ GNSS 行，事件行照常写入**，因此开关记录本身不会丢。

---

## 5. 时间基准（组合导航关键）

每一行都带 `t_us`：来自 `esp_timer_get_time()` 的 64 位微秒、单调递增、分辨率 1 µs。
取值点尽量贴近物理事件：

- **IMU**：在 **BMI160 INT1 data-ready 中断里打时间戳**（不在任务里补），
  这样 IMU 时间抖动最小，后续紧耦合融合才能用；
- 磁力计 / GNSS：在读 / 解析时刻打戳。

GNSS 行额外给出绝对 UTC（`utc_date` / `utc_time`），用于把 `t_us` 锚定到 UTC。

### PPS ↔ UTC 配对规则

> **PPS 沿在整秒的起点，而描述这一秒的 NMEA 语句在 PPS 之后才发出。**

因此配对方向是：**「最近一个尚未被消费的 PPS 沿」配「下一个完成的 UTC 整秒」**。
配错方向就是经典的**差一秒**错误。代码里对配对年龄做了校验（`age > 1.5 s` 会告警），
`gnss.csv` 的 `pps_t_us` + `pps_seq` 就是这条链路。

无定位（无 fix）时没有 RMC 语句，固件会在静默 1.5 s 后写一条**仅含 PPS 的记录**，
保证 PPS 时序不丢（该行其他字段为 0 / invalid）。

---

## 6. WiFi 网页文件浏览

板子自带一个 SoftAP + 网页文件浏览器，**不需要路由器**。

| 项 | 值 |
|---|---|
| SSID | `esp32c3-supermini-gnss` |
| 密码 | `12345678`（WPA2-PSK） |
| 信道 | 1（2.4 GHz；ESP32-C3 不支持 5 GHz） |
| 发射功率 | 20 dBm（`APP_WIFI_TX_POWER_DBM`） |
| 地址 | <http://192.168.4.1/> |

用法：手机或电脑连上该 WiFi → 浏览器打开 <http://192.168.4.1/> → 列出 TF 卡里的文件，
每行有 **download / head / tail** 三个按钮：

- `download`：下载整个文件；
- `head`：预览前 200 行；
- `tail`：预览**最后** 200 行 —— 看正在写入的日志最实用。

页面每 5 秒自动刷新文件列表。预览行数由 `APP_WEB_PREVIEW_LINES` 控制。

HTTP 接口（页面是自包含的，无外部依赖）：

| 接口 | 说明 |
|---|---|
| `GET /` | 单页界面 |
| `GET /files` | JSON：`{mount, logging, files:[{name,size}]}` |
| `GET /download?f=NAME` | 下载原文件 |
| `GET /preview?f=NAME&tail=0\|1` | 预览前/后 N 行（`text/plain`） |

> 安全与并发：
> - 文件名白名单校验（禁止 `/`、`\`、`..`），只能访问挂载点根下的**普通文件**；
> - 网页与采集线程共享一把 SD 锁，但**读文件与写 socket 分离**，
>   慢速客户端不会拖慢 100 Hz 采集；
> - WiFi 会带来额外功耗与少量时序抖动；要最干净的数据可在 menuconfig 里关掉
>   `APP_WIFI_ENABLE`。

## 7. 配置项（`idf.py menuconfig` → *GPS/IMU acquisition*）

| 选项 | 默认 | 说明 |
|---|---|---|
| `APP_I2C_FREQ_HZ` | 400000 | I2C 速率，两个传感器统一覆盖 |
| `APP_BMI160_ACC_RANGE_*` | ±8 g | 加速度量程 |
| `APP_BMI160_GYR_RANGE_*` | ±1000 dps | 陀螺量程 |
| `APP_BMI160_ODR_HZ` | 100 | ODR（会自动选 ≥ 该值最近的一档） |
| `APP_BMI160_USE_INT1` | y | 用 INT1 做 data-ready（关掉则轮询） |
| `APP_MAG_RATE_HZ` | 100 | 磁力计轮询率 |
| `APP_QMC5883L_ODR/OSR/RANGE` | 100 Hz / 256 / ±2 G | 见注释 |
| `APP_GNSS_BAUD` | 9600 | GPS 串口波特率 |
| `APP_GNSS_UART_PORT` | 1 | GPS 使用的 UART 外设 |
| `APP_LOG_TO_SD` | y | 关掉则只输出串口 |
| `APP_LOG_SERIAL_MIRROR` | y | 镜像到串口 |
| `APP_SERIAL_MIRROR_DECIM` | 10 | 串口只镜像每 N 条 IMU/磁行（保护采集链路） |
| `APP_LOG_AUTOSTART` | y | 上电即开始记录 |
| `APP_LOG_FLUSH_MS` | 1000 | 落盘间隔 |
| `APP_SD_SPI_FREQ_KHZ` | 10000 | TF 卡 SPI 时钟（失败会自动降到 4M/400k） |
| `APP_SD_RETRY_MS` | 10000 | 未挂载时每 N ms 自动重试挂载（0 = 关闭）；每次重试都会做一次原始 SPI 握手，便于示波器触发 |
| `APP_KEY_DEBOUNCE_MS` | 30 | 按键消抖 |
| `APP_STATUS_PERIOD_S` | 5 | 状态行周期 |
| `APP_WIFI_ENABLE` | y | SoftAP + 网页文件浏览总开关 |
| `APP_WIFI_SSID` | `esp32c3-supermini-gnss` | SoftAP 名称 |
| `APP_WIFI_PASSWORD` | `12345678` | 8~63 字符；留空为开放网络 |
| `APP_WIFI_CHANNEL` | 1 | WiFi 信道（1 最兼容；扫不到时可试 1/6/11） |
| `APP_WIFI_TX_POWER_DBM` | 20 | 最大发射功率；SuperMini 天线小，默认拉满；欠压重启时调低 |
| `APP_WIFI_MAX_STA` | 4 | 最大客户端数 |
| `APP_WEB_PORT` | 80 | HTTP 端口 |
| `APP_WEB_PREVIEW_LINES` | 200 | 预览行数 |

---

## 8. 启动日志与状态行

启动时会打印接线清单、传感器检测结果（BMI160 chip id 应为 `0xD1`，
QMC5883L chip id 应为 `0xFF`）、TF 卡挂载与本次 run 索引。

每 5 秒一行状态：

```
up=125 imu=100Hz mag=100Hz gnss=1/s log=on run=7 utc=14:30:05.000 fix=1/3 sats=9 hdop=0.85
pps=125 iv=1000000us jit=12us rows=12500/12500/125/3 bytes=1048576 err=0 rst=0 drop=0/0/0/0 sd=ok
```

重点看 **`drop=`**：非 0 说明消费端（通常是 SD 卡）跟不上，需要降低速率或换卡。

---

## 9. 排查

| 现象 | 处理 |
|---|---|
| `BMI160 not found` / `QMC5883L not found` | 查 I2C 接线与上拉；GPIO8 若接了板载 RGB 灯会干扰 SDA；确认 BMI160 的 SDO 接法对应 0x68/0x69 |
| 板子反复重启 / 进不了下载模式 | GPIO2/8/9 上电必须为高：别按住 KEY1/BOOT，确认上拉到位 |
| 串口无输出 | 确认控制台是 USB Serial/JTAG，且连的是 USB-C（不是 UART 脚） |
| `TF card mount failed` | 固件会用 10M/4M/400k 三档速率自动重试；三档全失败则是电气问题 |
| `sdmmc_card_init failed (0x107)` | 0x107 = 超时，发生在文件系统之前，**与是否 FAT32 无关**：优先查 MISO/MOSI 是否接反、卡是否插紧、模块 3.3V 供电、CS(GPIO7) 是否接上，以及 MISO/CS 是否缺上拉 |
| GNSS 无数据 | 波特率（默认 9600）、TX/RX 是否交叉、天线是否在室外 |
| `fix=0` 但 `pps` 在跳 | 未定位，属正常；`gnss.csv` 会出现 PPS-only 行 |
| `drop` 持续增长 | 换更快的 SD 卡或调低 ODR |
| 连上 WiFi 打不开网页 | 确认设备已广播 `esp32c3-supermini-gnss`；手机需关闭移动数据；地址是 `http://192.168.4.1/`（不是 https） |
| 网页显示 `TF card is not mounted` | TF 卡未挂载，按上面 `TF card mount failed` 排查 |

---

## 10. 后续标定（预留钩子）

磁力计的**硬铁/软铁标定**目前是恒等变换，位置在
`sensor_mag.c` 的 `MAG_HARD_IRON[3]` 与 `MAG_SOFT_IRON[3][3]`。
标定后只需改这两个常量，采集链路与 CSV 格式都不变。

IMU 零偏标定可调用驱动自带的 `bmi160_calibrate()`（需设备静止）。

---

## 11. 目录结构

```
main/
  app_main.c        启动顺序（队列 → I2C → 传感器 → GNSS/PPS → 日志 → 任务）
  app_state.c       共享队列/计数/互斥量的定义与访问器
  board_pins.h      全部引脚定义（硬件固定）
  app_config.h      Kconfig → 常量映射
  data_types.h      记录结构体与共享状态声明
  sensor_imu.c      BMI160：INT1 data-ready 中断 + 高优先级任务读 12 字节
  sensor_mag.c      QMC5883L：DRDY 轮询 + 标定钩子
  gnss_nmea.c       自写 NMEA 解析（GGA/RMC/GSA/GSV/VTG，校验和）
  gnss_uart.c       UART 泵 + PPS↔UTC 配对 + 逐秒出记录
  pps_capture.c     GPIO10 上升沿 ISR（只做打戳 + 入队）
  keys.c            KEY1/KEY2 消抖
  logger_sd.c       SD 挂载 / run 索引 / 缓冲落盘 / 串口镜像 / 掉卡重挂
  log_format.c      纯 CSV 格式化函数（可在主机上单测）
  status.c          周期状态行
  wifi_ap.c         WiFi SoftAP（NVS/netif/事件循环自理）
  web_server.c      HTTP 文件浏览：list / download / preview（含路径白名单）
  web/index.html    内嵌的单页前端（EMBED_FILES，无外部依赖）
tools/
  check_logs.py     主机端日志校验（列数 / t_us 单调 / 速率 / 丢点概览）
partitions.csv      3 MB 应用分区（默认 1 MB 装不下 WiFi + HTTP 服务）
```

### 设计取舍

- **一个数据源一个文件**：速率差异大，各文件行自带 `t_us` 即可对齐；
- **生产者绝不阻塞**：ISR/传感器任务只入队，队列满记 `drop` 而不是等待；
- **日志任务兜住慢速 SD**：大缓冲 + 定时 `fflush`/`fsync`，SD 写失败会
  关闭→重挂载→换新 run 索引重开，并写入 `SD_ERROR`/`SD_REMOUNT` 事件；
- **缺传感器/缺卡都非致命**：跳过该源、记事件、其余继续跑。
