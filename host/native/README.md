# Windows / Linux 原生 C++ 接收器

使用原生 socket 收包、连续性检查和 C++ FFT，浏览器负责绘图。
运行时不依赖 Python。收包、信号处理、Web 控制分别使用独立线程。

实时流使用 S31Q v2：44 字节包头，不计算或传输应用层 CRC32。
固件、C++ 和 Python 实时接收器须同步升级。包序号、样点计数和板端 DMA
丢样检测仍启用；它们检查连续性，不验证 IQ 内容的端到端完整性。
以太网硬件 FCS 保留，IPv4 UDP checksum 沿用 0。旧 v1 记录可通过 Python 离线导出。

## Linux 构建与运行

使用 GCC / Clang 和 pthread，无第三方运行时依赖：

```sh
host/native/build.sh
sudo ./artifacts/native/s31_rx --serve --rate 53333333 --core 23 \
  --page host/s31_receiver/index.html
```

默认使用 **UDP**。`--core 23` 是此次 EPYC 主机的测试配置，其他机器应选择其
可用 CPU。程序仍只监听本机 HTTP 端口；远端使用 SSH 转发访问：

```sh
ssh -N -L 8765:127.0.0.1:8765 epyc7443@192.168.2.134
```

浏览器打开 <http://localhost:8765>。此次远端专用口为 `enp193s0`，板子为
`169.254.9.36/16`，主机增加 `169.254.9.35/16`。主机地址是运行时设置，重启后
需要重新配置；已有管理网口和地址不需要更改。

```sh
sudo ip address replace 169.254.9.35/16 dev enp193s0
sudo ./artifacts/native/s31_rx --rate 53333333 --seconds 300 --dsp --core 23 \
  --report artifacts/linux-udp53.json
```

Linux 接收线程使用 `pthread_setaffinity_np` 绑核和 `SCHED_FIFO` 优先级 20；
`mlockall(MCL_CURRENT)` 锁定已分配内存。按实际权限尝试 `SO_RCVBUFFORCE`，
不会修改全局 sysctl 或其他接口。无需特权也可以运行，报告中的 `priority_set`、
`memory_locked` 和 `socket_buffer` 会反映实际结果；Linux 读回的接收缓冲值包含
内核的倍增记账，默认 64 MiB 请求读回 128 MiB。

`kernel_drops` 来自该 UDP socket 的 `SO_RXQ_OVFL` 累计计数，独立于协议序号检查。
发生 socket 溢出后建议重新启动程序进行下一轮验收。网卡硬件丢包需另查
`ethtool -S enp193s0`。当前未修改网卡接收环、IRQ 亲和性或内核实时调度限制。

## Windows 构建与运行

需要 Visual Studio 的“使用 C++ 的桌面开发”组件：

```powershell
cmd /c host\native\build.cmd
.\artifacts\native\s31_rx.exe --tcp --serve --rate 4000000 --page host\s31_receiver\index.html
```

打开 <http://localhost:8765>。支持调频、手动增益、采样率、Hann FFT、去直流、
功率平均、频谱与瀑布图。接收样点与网络连续性统计独立于显示刷新。
需要 IQ 磁盘记录时，可使用 [Python 接收器](../../docs/s31-ethernet.md#记录与导出)。
同一时间仅运行一个接收客户端。

无界面长测（加 `--dsp` 让 FFT 同时运行）：

```powershell
.\artifacts\native\s31_rx.exe --tcp --rate 4000000 --seconds 600 --dsp --report artifacts\native-test.json
```

`--tcp` 使用 TCP 9876 和板端约 8 MiB PSRAM 队列；省略时使用 UDP。
TCP 自动重传网络丢包，板端缓冲满时仍会丢样，报告中的 `buffer_drops` 会记录。

退出码 0 表示此次测试的传输连续性检查通过；2 表示有缺口或设备错误；1 表示
启动或通信失败。报告对比主机与板端包数和字节数，覆盖尾部缺包。按 Ctrl+C 停止并输出报告。

## Windows 实时性设置

默认使用 64 MiB socket 接收缓冲，读回实际值写入报告。进程设为
`ABOVE_NORMAL_PRIORITY_CLASS`，接收线程绑到进程可用的最后一个逻辑 CPU，并设置
高优先级。通过 Windows [MMCSS Pro Audio](https://learn.microsoft.com/zh-cn/windows/win32/procthread/multimedia-class-scheduler-service)
请求时间敏感线程调度；报告记录调用是否成功。未修改系统全局调度策略。

- `--core N` 指定收包线程的逻辑 CPU；自动选择限于当前处理器组。
- `--no-mmcss` 只使用绑核及普通线程优先级提升。
- `--normal` 关闭本程序的优先级和绑核设置，用于对照。
- `--buffer-mb N` 调整 socket 缓冲，范围 1–256 MiB。

IQ 收包热路径没有日志、文件 I/O 或动态内存分配；包头验证后的数据通过预分配
SPSC 环形队列交给 DSP 线程。显示消费过慢时计入 `display_skipped_packets`，
不阻塞接收或隐藏采样缺口。频谱使用最新连续窗口，缺口会重置平均。

提高应用线程优先级无法恢复在网卡、USB 控制器或内核路径中已经丢掉的包。
`max_receive_gap_us` 记录相邻有效包到达应用的最大时间间隔，不能独立区分调度
延迟与网络停顿。

## 当前专用 USB 网卡的缓冲调整

`tune_adapter.ps1` 适用于此次设备名称为 `Ethernet 8` 的 Realtek USB GbE 网卡。
它将 Receive Buffers 从 16 调至 256、Receive URBs 从 6 调至 64，然后重启该网卡。
这些数值来自此机器网卡驱动声明的范围；不是其他网卡的通用配置。
需要在 **Windows 管理员 PowerShell** 中运行，调参前先停止接收。

```powershell
powershell -ExecutionPolicy Bypass -File host\native\tune_adapter.ps1
```

恢复此次机器原值：

```powershell
powershell -ExecutionPolicy Bypass -File host\native\tune_adapter.ps1 -Restore
```

## 合成信号测试

Windows 使用下列命令；Linux 将可执行文件路径换为 `artifacts/native/s31_rx`：

```powershell
python tests\test_s31_native.py --exe artifacts\native\s31_rx.exe
python tests\test_s31_native.py --exe artifacts\native\s31_rx.exe --tcp
```

测试通过本机 UDP / TCP 模拟板子发送已知复数单音，并检查完整 C++ 接收/连续性检查/FFT/HTTP
处理链路的峰值频率、幅度、直流均值及人工注入的缺口。TCP 测试拆开发送包头和
载荷以验证字节流重组；两种模式都验证 UDP 控制报文过长后的接收线程恢复。
它不替代射频硬件验证。

以下实机记录属于启用应用层 CRC32 的历史版本。当前协议为 S31Q v2，已移除实时流的 CRC32。

## Linux 实机结果（2026-10-04）

EPYC 7443 主机、板载 `igc` 网卡、千兆全双工，默认 RX/TX 环均为 256。
沿用板上固件，没有为本次 Linux 测试重新刷写。网线更换后链路恢复。

16 MS/s UDP + C++ FFT 连续运行 **639.928 秒**：

| 检查项 | 结果 |
| --- | --- |
| IQ 有效吞吐 | 255.997 Mbit/s，约 32 MB/s |
| 接收数据帧 | 15,961,719 |
| 复数样点 | 10,238,717,664 |
| 有效 IQ 字节 | 20,477,435,328 |
| 主机缺包 / 缺样 / CRC 错误 | 0 / 0 / 0 |
| 板端 DMA 丢样 / 发送错误 | 0 / 0 |
| socket 溢出 / 网卡硬件丢包 | 0 / 0 |
| 停止后两端包数、字节数 | 完全一致 |

FFT / 频谱 / 瀑布图同时启用。浏览器观测脚本曾因空闲 HTTP 连接收到错误回复而
中断，UDP 收包和 FFT 始终持续；该网页问题已修复并补充回归测试。
639.928 秒报告对应修复前程序，修复只涉及 HTTP 空闲连接处理；修复后的程序
另外通过 60.383 秒页面 / UDP / FFT 复测（56 次 HTTP 状态查询），
调频、8/16 MS/s 切换、停止计数核对及重新启动均通过，最终保持 16 MS/s UDP 运行。

该结果证明此次测试窗口内的 DMA / 网络传输连续性，不构成无限时长不丢包保证。
射频诊断总线位映射、相位及幅度准确性仍需已知信号源验证。Windows 与 Linux 测试
同时改变了主机、网卡和网线，不能单独将改善归因于某一项。

完整报告和截图在本机 `artifacts/s31-linux-20261004/`。远端程序位于
`/home/epyc7443/esp-sdr/artifacts/s31_rx`，当前服务名为 `esp-sdr-udp-test.service`。
停止当前服务可执行 `sudo systemctl stop esp-sdr-udp-test`；服务和额外网口地址
均为此次运行时配置，主机重启后需要重新启动 / 设置。

## Windows 实机结果（2026-10-02）

Windows 网卡 Receive Buffers=256、Receive URBs=64 已执行并读回确认。
调整后 UDP 16 MS/s 的 126.7 秒测试仍缺失 386 包，CRC 和 DMA 丢样计数为 0。
提高 Windows 接收线程优先级和网卡缓冲尚未消除 UDP 偶发缺包。

TCP 开发配置在 4 MS/s、20 秒测试中收到了 80,067,456 个复数样点，
主机和板端计数完全一致，缺样、CRC 错误、DMA 丢样和 PSRAM 队列溢出均为 0。
8 / 16 MS/s TCP 测试出现了板端队列溢出，尚未通过连续接收验收。
上述 TCP 短测不能代表长时间无缝接收已经通过；2026-10-04 已转到 Linux
板载网卡上优先验证 UDP，结果单独记录。

Windows 64 MiB socket 缓冲、CPU 31 绑核、高线程优先级和 MMCSS 调用已在实机报告
中确认生效。瞬时 socket 错误计入 `control_send_errors` / `socket_receive_errors`，
并保留 `last_socket_error`，持续断流会使连续性检查失败。

Windows 测量在本机 `artifacts/s31-ethernet-tuned-20261002/`，原始 UDP 基线在
`artifacts/s31-ethernet-20261002/`。射频诊断总线位映射和相位连续性仍需已知信号源验证。

## 更高采样率测试（2026-10-04）

固件和主机开放了 20 / 32 / 40 MS/s 测试档位，使用 160 MHz 时钟精确分频。
每档 20 秒 UDP + FFT 实测均未通过连续接收：

| 请求采样率 | 有效吞吐 | 板端 DMA 丢样 | 主机 UDP 缺包 / CRC 错误 |
| --- | --- | --- | --- |
| 20 MS/s | 252.896 Mbit/s | 83,823,048 | 0 / 0 |
| 32 MS/s | 234.795 Mbit/s | 346,226,112 | 0 / 0 |
| 40 MS/s | 231.467 Mbit/s | 510,227,046 | 0 / 0 |

板端丢样发生在打包前，因此 UDP 包序号可以连续，但 IQ 的绝对样点位置有缺口。
单看 UDP 缺包为 0 无法判定无缝接收。

双核 PSRAM 队列、CRC 预计算和 slicing-by-eight CRC 的对照版本也没有稳定通过
20 MS/s；这些试验源码及报告留在 `artifacts/s31-higher-rates-20261004/`。
该轮测试后恢复原有发送路径及 CRC 实现，以 **16 MS/s** 作为当时已验证的连续接收档位。
高档位在界面标为“有丢样”，用于进一步诊断；不能作为已验证的宽带连续接收模式。

恢复后的 16 MS/s 固件又通过 60.072 秒 UDP + FFT + 浏览器复测：接收
961,055,424 个复数样点（1,922,110,848 字节），主机/板端包数和字节数一致，
DMA 丢样、UDP 缺包、CRC 错误和内核丢包均为 0；测试后已重新启动 16 MS/s。

## 无 IQ 大块 CPU 拷贝的 UDP 实测（2026-10-04）

实验固件使用预分配 AHB GDMA 将 PARLIO 数据搬到双 SRAM 缓冲，再让 MAC DMA
直接读取 IQ 载荷。保留每包 CRC，CPU 仍构造/复制包头、计算 CRC 和管理描述符。
详细配置及缓冲生命周期见 [DMA 路径说明](../../docs/s31-ethernet.md#iq-数据的-dma-搬运路径)。

16 MS/s + 原生 C++ FFT 实测 **300.012 秒**：

- 接收 7,483,183 包、4,800,122,208 个复数样点、9,600,244,416 字节 IQ。
- DMA 丢样、缓冲丢样、UDP 缺包、CRC 错误、内核丢包和板端发送错误全部为 0。
- 停止后板端/主机包数、字节数一致；`cpu_stage_bytes=0`，
  `dma_stage_bytes=9600244416`，`direct_tx=true`。
- 20 MS/s 的流水线版本短测仍有板端丢样，尚未提高已验证的连续采样率。
- 分段计时中 CRC 区间约占运行时间的 61.5%，包含区间内中断/抢占的影响。

固件、原始报告和 SHA256 保存在 `artifacts/s31-direct-dma-20261004/`。
该结果验证数据传输连续性；射频位映射和相位连续性仍需已知信号源验证。

适配器保留普通 MAC 发送函数的 IRAM 放置，并保持 TCP 原来的单暂存缓冲。
最终固件的 TCP 4 MS/s 通过 20.010 秒短测，160,062,336 字节 IQ，主机/板端计数一致，
缺样、CRC 和缓冲溢出均为 0。五分钟 UDP 报告对应该兼容性修正前的固件；
最终固件另做 UDP + 浏览器回归，原始报告分别记录，避免混淆固件来源。

最终固件 UDP 16 MS/s + FFT + 浏览器复测 60.387 秒、1,931,158,656 字节：
所有丢包/丢样/CRC 检查为 0，两端计数一致，CPU 暂存复制量为 0。
8/16 MS/s 切换、调频、增益、停止/重启通过，测试后保持 16 MS/s 接收。


## 移除应用层 CRC 后的实测（2026-10-04）

固件、原生 C++ 和 Python 实时接收器已同步升级至 S31Q v2。应用层 CRC
字段和计算已移除；UDP DMA 搬运及借用 MAC 描述符的路径保持启用。
以下是同一 EPYC 7443 / 板载 igc 千兆网卡上的新固件实测，均启用 C++ FFT。

| 采样率 | 时长 | 有效 IQ 吞吐 | UDP 缺包 | 板端 DMA 丢样 |
| --- | --- | --- | --- | --- |
| 20 MS/s | 20.004 秒 | 319.887 Mbit/s | 0 | 0 |
| 32 MS/s | 300.011 秒 | 511.810 Mbit/s | 0 | 0 |
| 40 MS/s | 20.010 秒 | 548.993 Mbit/s | 0 | 112,279,068 样点 |

32 MS/s 长测共接收 14,960,986 包、9,596,793,024 个复数样点，
合计 19,193,586,048 字节。停止后的主机与板端包数、字节数完全一致。
格式错误、缺样、内核丢包、缓冲丢样和板端发送错误均为 0。
`cpu_stage_bytes=0`、`dma_stage_bytes=19193586048`、`direct_tx=true`。
该轮验证通过的最高连续档位为 **32 MS/s**；当时 40 MS/s 的瓶颈表现为板端
DMA 丢样，主机未观察到 UDP 丢包。此测试验证传输连续性，不验证 IQ 内容
端到端完整性或射频性能。

Linux / Windows 原生程序均通过模拟设备的 UDP、TCP、FFT 和缺包检测回归。
新录制文件使用 S31IQ v2；旧 v1 文件仍可离线校验并导出。
报告、日志、固件及来源哈希位于 `artifacts/s31-no-crc-20261004/`。

TCP 实机回归在 4 MS/s 下运行 20.009 秒，收到 160,054,272 字节，
主机与板端计数一致，缺样、DMA 丢样、队列溢出和格式错误均为 0。
TCP 沿用 CPU 复制和 PSRAM 队列，未宣称无 CPU 拷贝。

网页回归通过 8/32 MS/s 切换、调频、增益、停止和重启。32 MS/s + FFT + 浏览器
复测 60.049 秒、3,841,742,016 字节，所有丢包/丢样计数为 0，两端计数一致，
JavaScript 错误为 0。测试后服务保持 32 MS/s UDP 接收。


## AXI DMA 和整包发送优化（2026-10-04）

同一 EPYC 7443 / 板载 igc 千兆网卡，S31Q v2、应用层 CRC 关闭。
最终固件在 **53.333333 MS/s UDP + C++ FFT** 下连续运行 **300.008 秒**：

| 指标 | 实测 |
| --- | --- |
| 有效 IQ 吞吐 | 852.665 Mbit/s（106.58 MB/s） |
| 接收复数样点 | 15,987,902,112 |
| 接收 IQ 字节 | 31,975,804,224 |
| 收包 / 板端发包 | 23,791,521 / 23,791,521 |
| UDP 缺包、缺样、迟到、格式错误 | 均为 0 |
| 采集 FIFO 溢出、DMA 丢样、缓冲丢样、发送错误 | 均为 0 |
| Linux socket 内核丢包 | 0 |
| IQ 暂存 CPU 拷贝量 | 0 |
| IQ 暂存 DMA 搬运量 | 31,975,804,224 字节 |

主机与板端最终包数、字节数完全一致；相对此前通过的 32 MS/s，配置采样率
提高约 66.7%。最高档使用 160 MHz 时钟三分频，协议中取整为 `53333333`；
吞吐按主机墙钟及实际收到的字节统计。该结果验证传输计数连续性和采集 FIFO
无溢出，射频位映射与相位连续性仍需已知信号源验证。

主要改动：AXI DMA 64 字节突发、三组 SRAM 帧缓冲、DMA 分散写入包载荷区、
MAC 每包一份描述符借用整帧、固定包头复用，以及每 10 ms 发布一次统计。
PARLIO 缓冲块与 EOF 长度统一为 4032 字节。MAC / PARLIO 适配在构建目录生成，
用源文件 SHA256 限定已验证的 IDF 版本。固件每次启动采集前验证 DMA 分散写入
和头部保护；普通 lwIP 流量仍共用 TX 互斥锁，缓冲复用前检查 MAC 所有权。

新增 `parlio_overflow` 粘滞标志，出现采集 FIFO 写溢出时，即使 UDP 序号完整，
主机也会判定传输不连续。Linux / Windows 原生 UDP、TCP、FFT、缺包及 FIFO
告警回归均通过；34 项单元测试和常规发送配置的固件构建通过。

最终报告、固件、源码哈希和日志位于 `artifacts/s31-rate-opt-20261004/`。

最终固件另通过 UDP 40 MS/s（10.009 秒、639.557 Mbit/s）和 TCP 4 MS/s
（20.004 秒、63.992 Mbit/s）实机回归，FIFO、DMA、缓冲和传输错误均为 0。
网页通过 8/53.333 MS/s 切换、调频、增益、停止和重启；最终 53.333 MS/s + FFT
网页接收复测 59.373 秒、6,327,635,328 字节，所有错误计数为 0，两端计数一致。
高档位 HTTP 校验已同步，采集恢复后会清除临时流停滞提示。
测试后服务保持 53.333 MS/s UDP 接收。

五分钟长测使用最终板端固件；随后主机仅修正网页高档位参数校验与错误提示。
这些主机修正已通过 Linux / Windows 模拟设备和上述实机网页回归。
