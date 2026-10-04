# ESP32-S31 千兆以太网连续接收

适用硬件：ESP32-S31 Function CoreBoard-1，板载 YT8531 千兆 PHY。
固件保留原有 USB/UART 短帧接收，并增加单客户端连续 IQ 接收。
连续接收期间禁止改频、改增益及短帧采集；主机应用自动停止、配置、重启。

Windows 和 Linux 可使用 [原生 C++ 接收器](../host/native/README.md)，具有独立高优先级
收包线程、C++ FFT 和本地浏览器界面。Linux 板载网卡优先使用默认 UDP 模式；
原生程序加 `--tcp` 使用可靠传输，
板上 PSRAM 队列吸收短时网络停顿；Python 程序目前使用 UDP。
DMA 和发送路径优化后，Linux + 板载网卡已通过 53.333333 MS/s UDP + C++ FFT、
300.008 秒零缺样测试，有效 IQ 吞吐 852.665 Mbit/s，详见
[原生接收器实测结果](../host/native/README.md#axi-dma-和整包发送优化2026-10-04)。以下保留 Python 用法。

## 连接与启动

1. 网线连接板子和主机千兆网口。板子固定地址为 `169.254.9.36/16`。
   主机对应网口使用 `169.254.9.35/16`，无需网关。不要改动主机的上网接口。
2. 安装主机依赖（Python 3.10 或更新版本）：

   ```sh
   python3 -m venv .venv
   .venv/bin/pip install -r host/requirements.txt
   ```

3. 在仓库根目录运行：

   ```sh
   .venv/bin/python host/run.py --rate 8000000
   ```

4. 浏览器访问 <http://localhost:8765>。提供中心频率、手动增益、采样率、
   FFT 点数、去直流、功率平均、频谱、瀑布图和 IQ 记录。
   `--no-start` 只启动界面，点击“开始 / 应用”后接收。
   主机地址不同时通过 `--bind` 指定，但需与板子处于同一二层网络和子网。

Windows PowerShell 原生运行：

```powershell
py -m venv .venv-win
.\.venv-win\Scripts\python -m pip install -r host/requirements.txt
.\.venv-win\Scripts\python host/run.py --rate 8000000
```

程序只监听本机 Web 端口。退出程序会停止流；主机意外退出后，固件在约 5 秒内
因心跳超时停止 DMA；TCP 随后结束发送并释放缓冲。其他客户端可以查询 `NET?`，
不能抢占活动流。

## 接收、处理和连续性

数据路径为：射频诊断 I/Q 总线 → GPIO 输出/输入矩阵 → PARLIO 循环 DMA →
独立 SRAM 副本 → UDP，或 PSRAM 环形队列 → TCP → 以太网 MAC DMA →
主机收包和连续性检查 → C++ / NumPy FFT。
PARLIO 只提交一次循环接收事务，不在每个数据块之间停机重启。
GPIO 矩阵使用芯片自身引脚输入回读，无需外接 16 根跳线。

**预留并保持空闲的引脚**：`35–40, 42–49, 0, 1`。
这些引脚在接收期间输出诊断信号，不应连接其他外设。
以太网使用 MDC 5、MDIO 6、PHY reset 7 和 RGMII 8–19。
这不是适用于任意 S31 开发板的通用引脚配置。

常规档位为 `0.25 / 1 / 2 / 4 / 8 / 16 MS/s`，每个复数样点两个字节：有符号 I8、Q8。
另开放 `20 / 32 / 40 / 53.333333 MS/s` 诊断档。最高档使用 160 MHz 时钟三分频，
协议速率取整为 `53333333`。各档位测试时长和结果见原生接收器实测记录。
16 MS/s 的有效 IQ 载荷为 32 MB/s；以太网线速另计包头和帧间隔。
采样率表示 PARLIO 对诊断总线的采样时钟，低于 ADC 原生速率时为直接抽取，
没有额外数字抗混叠滤波。因此应控制带外输入，不能将低速率简单理解为窄带 DDC。
当前连续模式使用接收机初始化带宽；USB `BANDWIDTH` 配置用于短帧模式。

每包携带会话号、序号、绝对样点位置和板端 DMA 丢样计数。
主机检查首包、中间缺包、重复/迟到包、错误会话和包头格式；停止后比较板端发包
总数，检测尾部缺包。缺口会显示在界面中，不补零或隐藏。UDP 不重传。

“传输连续”是 DMA 和主机传输计数的检查结果。它不单独证明诊断总线的位映射、
射频样点相位、杂散或增益精度；这些还需要已知射频信号源验证。

FFT 使用复数双边 Hann 窗、可选去直流、线性功率指数平均，显示单位为
`dBFS/bin`，未标定为 dBm。频谱显示取最新完整连续窗口，以约 15 帧/秒处理，
浏览器慢或关闭不会停止收包。缩小到 1024 个显示点时使用最大值保留窄峰。
IQ 记录另有独立写入线程和丢包计数。

## 记录与导出

默认写入 `artifacts/recordings/`，可通过 `--recording-dir` 修改。
`.s31iq` 保留原始元数据和 IQ；同名 `.json` 保存记录统计。
文件以 8 字节 `S31IQ\x02\x00\x00` 开头，之后每条记录是小端 uint16 长度加
完整 UDP IQ 数据包。

校验并导出供其他软件使用的连续交织 I8/Q8 文件：

```sh
PYTHONPATH=host python3 -m s31_receiver.recording input.s31iq output.ci8
```

导出器验证包头、参数和连续性。有缺口、混合会话或结构损坏时拒绝生成扁平
IQ 文件；原始记录可用于分析缺口。输出 `.ci8.json` 包含采样率、中心频率和
首样点位置。CI8 按 `int8` 读出，偶数位置为 I，奇数位置为 Q，除以 128 归一化。

## 无界面验证

```sh
PYTHONPATH=host python3 -m s31_receiver --rate 8000000 --seconds 300 \
  --report artifacts/s31-soak.json
```

进程保留 FFT 处理；加 `--record` 同时测试磁盘记录。
报告包含主机和板端计数；缺包、格式错误、采集 FIFO 溢出、设备错误或流停滞时返回非零。
主机的墙钟吞吐和板端采样时钟可能略有差异，应同时检查板端 `elapsed_ms`。

## 固件构建

使用 `firmware-targets.json` 中的 S31 SDK 提交：
`26dd2948925d44f41e3bb614fb044b7a0839a242`。
激活该 ESP-IDF 后，使用新的构建目录启用本次验证的 DMA 路径：

```sh
idf.py --preview -B build-s31-ethernet-dma -DIDF_TARGET=esp32s31 \
  -DSDKCONFIG="$PWD/build-s31-ethernet-dma/sdkconfig" \
  "-DSDKCONFIG_DEFAULTS=$PWD/sdkconfig.defaults.esp32s31;$PWD/sdkconfig.defaults.esp32s31_eth;$PWD/sdkconfig.defaults.esp32s31_dma" \
  -DS31_DMA_PROBE=OFF build
idf.py --preview -B build-s31-ethernet-dma -p PORT flash
```

Ethernet 配置启用 16 MB 八线 PSRAM，使用 250 MHz 时钟，以兼容共享 PLL 上的
125 MHz RGMII 时钟。TCP 环形队列约 8 MiB；队列耗尽会计入 `buffer_drops`，
不能保证无限时长网络停顿下不丢样。

单独的 Ethernet 配置启用较大应用分区，首次切换应一起刷入 bootloader、分区表
和应用（`idf.py flash` 自动完成）。该配置默认关闭，不改变普通 S31 固件配置。
已有构建目录的 `sdkconfig` 优先于 defaults；切换配置时使用新的构建目录。

## 网络协议

控制端口为 9875，回复及 IQ 数据发回发送控制命令的 IP/UDP 端口。
客户端保持同一个 UDP socket，并在每秒发送一次 `NETPING <session>`。

| 命令 | 回复 / 行为 |
| --- | --- |
| `NET?` | `NET {JSON}`：链路、会话、速率、DMA/发送统计和错误码 |
| `FREQ <MHz>` | 停止流时设置频率，沿用原接收机调谐范围 |
| `GAIN MANUAL <code>` | 停止流时设置手动增益码 |
| `NETSTART <rate_hz> <session>` | 要求千兆链路，`OK` 后开始发包；启动失败可由 `NET?` 读取 |
| `NETSTARTTCP <rate_hz> <session>` | 同一主机先连接 TCP 9876，再用 UDP 控制口启动可靠流 |
| `NETPING <session>` | `PONG <session>`，刷新心跳 |
| `NETSTOP` | 停止采集、释放 DMA，回复 `OK` |

IQ 数据包头格式是小端 `struct.Struct('<4sBBHIIQIIHHQ')`，共 44 字节：
`magic='S31Q'`, version=2, flags=0, header_bytes=44, session(uint32),
sequence(uint32), first_sample(uint64), rate_hz(uint32), frequency_mhz(uint32),
samples(uint16), format=1(uint16), dma_dropped_samples(uint64)。
应用层不计算或传输 CRC32；IPv4 UDP checksum 仍为 0，以太网硬件 FCS 保留。
固件与实时客户端必须同时升级到 v2，旧版本包头会被拒绝。
连续性检查不代表 IQ 内容经过端到端校验。旧 v1 录制文件仍可离线校验、导出。
每包 IQ 载荷最多 1344 字节，避免 IP 分片。

TCP 使用相同的 44 字节头，单帧载荷最多 4032 字节。它是字节流，接收端按
`header_bytes + samples * 2` 重组帧，不依赖一次 `recv` 对应一帧。停止时板端先停止
DMA，再发送队列余量并关闭 TCP；主机读取到 EOF 后查询最终计数。

## IQ 数据的 DMA 搬运路径

`CONFIG_ESP_SDR_S31_DIRECT_TX` 和 `CONFIG_ESP_SDR_S31_DMA_COPY` 是 Function CoreBoard-1
专用的实验选项，默认关闭。可在 Ethernet 配置之后叠加
`sdkconfig.defaults.esp32s31_dma` 启用；必须使用上述固定的 S31 ESP-IDF 版本。

UDP 数据路径为：

```text
PARLIO DMA 环形缓冲 → AXI GDMA 分散写入 → 三组 SRAM 帧缓冲 → Ethernet MAC DMA
                                       CPU 原位更新包头、管理描述符
```

稳态 UDP 路径没有 IQ 大块 CPU 拷贝。仍然存在一次 GDMA 内存搬运：AXI DMA
以 64 字节突发，将每个 4032 字节块分散写入三个 1344 字节的包载荷区。
每区预留 128 字节头部空间，CPU 原位更新紧邻载荷的 86 字节网络/协议头。
MAC 直接借用完整帧，每包只用一份 TX 描述符，无须再复制包头。
TCP 沿用原有的 PSRAM 队列和复制路径。

内存 DMA 的通道和描述符在启动时分配，高速路径不分配内存。采集下一块 IQ 的
搬运与当前块的封包/发送重叠。MAC 驱动借用 IQ 缓冲，复用任一暂存缓冲前会等待
其全部发送描述符归还，停止时也会完成回收。三缓冲使上一块的 MAC 发送、
本块的包头更新、下一块的 DMA 搬运重叠。普通 lwIP 报文与此路径共用 TX 互斥锁，
并在使用驱动缓冲前恢复已完成的借用描述符，防止普通流量写入 IQ 数据区。

MAC 和 PARLIO 适配代码在构建目录中生成，SDK 工作树不被修改；源文件 SHA256
不匹配时构建失败，需重新检查驱动内部接口。该路径针对单个 MAC 和内部无缓存 SRAM。
实验版若在 100 ms 内无法回收 DMA 缓冲，会复位开发板，避免 DMA 访问已释放内存。
实际拔线故障恢复尚未验证。

PARLIO 适配将 SRAM 描述符块长固定为 4032 字节，与采集 EOF 长度一致，
避免原驱动 4092 字节节点造成的短块。AXI 描述符使用 8 字节对齐类型；
通道在初始化时复位，之后仅在上一次完成后启动下一次搬运。每次采集启动前，
用不同已知数据检查完整块、短块、分段边界及头部区域是否保持不变。

`NET?` 新增 `cpu_stage_bytes`、`dma_stage_bytes`、`direct_tx` 和分段周期统计。
`parlio_overflow` 是采集 FIFO 写溢出的粘滞标志（0/1），不是丢样数量；
主机和界面将其纳入连续性判断。`capture_clock` 保存启动时的采集时钟寄存器值。
前两个字段只统计 IQ 暂存阶段；不能用于统计整个系统的 CPU 内存访问量。
周期计数包括区间内的中断/抢占，且搬运与计算有重叠，不能简单解释为独占 CPU 利用率。
本轮优化结果位于 `artifacts/s31-rate-opt-20261004/`。
移除 CRC 后的对照结果和固件来源记录在本地 `artifacts/s31-no-crc-20261004/`；
此前保留 CRC 的对照记录位于 `artifacts/s31-direct-dma-20261004/`。
