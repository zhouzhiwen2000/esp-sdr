# Transmit commands

The standard C5 and S3 builds support half-duplex transmit and receive over
native USB Serial/JTAG and UART0. `CAPS` advertises `TX REPLAY CW`. The current
C61/S31 burst receiver builds retain their receive protocol; their dedicated
USB/Ethernet transceiver is a [separate project](../legacy/transceiver/README.md).

## Payloads and rates

All counts are complex samples, at most 16,380 per waveform. TX rates are
**samples per second**, not the receive rate indices:

- C5: 250,000; 500,000; 1,000,000; 2,000,000; 3,000,000; 4,000,000;
  6,000,000; 40,000,000; 80,000,000.
- S3: 40,000,000 or 80,000,000.
- Continuous replay: 40,000,000 or 80,000,000 on both targets.

C5 uses CPU-paced register writes for rates below 40 MS/s. Both targets use
the modem SRAM replay engine at 40/80 MS/s. These are nominal clock settings;
serial throughput is lower and uploads introduce gaps between finite bursts.
Keep the default fixed 240 MHz CPU clock with power management disabled.

`16` payloads contain signed 8-bit I followed by signed 8-bit Q, two bytes per
sample. Each component is expanded into the upper eight bits of its 10-bit
hardware field. `20` payloads pack consecutive 20-bit words, least-significant
bits first: I occupies bits 0–9, Q bits 10–19. Two samples occupy five bytes;
an odd final sample occupies three bytes. The legacy `TX` command accepts
little-endian 32-bit words with I/Q in bits 0–19.

Compute CRC32 over the exact uploaded bytes, using the value returned by
Python's `zlib.crc32(payload)`, and encode it as hexadecimal in commands.

## Finite output

Set the frequency with `FREQ <MHz>`, then send one of:

```text
TX16 <samples> <rate> <crc32-hex>
TX20 <samples> <rate> <crc32-hex>
TX <samples> <rate> <crc32-hex>
LOOP16 <samples> <rate> <repeats> <crc32-hex>
LOOP20 <samples> <rate> <repeats> <crc32-hex>
```

Wait for `READY`, send exactly the payload bytes, then read
`SENT <total-samples> <cpu-cycles> <maximum-lateness-cycles>` or `ERR <reason>`.
`LOOP` uploads once and repeats the waveform. Total finite RF duration is
limited to 100 ms. Modem replay supports at most 255 repeats; CPU-paced C5
output supports at most 100,000, subject to the duration limit. DMA-mode
maximum lateness is zero because samples are not paced by the CPU.

For a sequence of separately uploaded frames:

```text
TXRUN <samples-per-frame> <rate> <frames> <16|20>
```

Read `RUN`. For each frame, send its CRC32 as **four little-endian binary
bytes**, wait for `READY`, upload the payload, and read `SENT`. The final
response is `END`. Frame count is 1–1000. An error terminates the run.

## Continuous output

```text
REPLAY16 <samples> <rate> <crc32-hex>
REPLAY20 <samples> <rate> <crc32-hex>
```

Read `READY`, upload one payload, then read `PLAYING`. The modem repeats that
waveform until `REPLAY STOP` or lease expiry. `REPLAY KEEP` renews the lease;
`REPLAY?` reports `REPLAY ON|OFF <samples> <rate>`.

`CW START` starts the PHY's continuous tone at the selected frequency.
`CW KEEP` renews its lease, `CW STOP` stops it, and `CW?` reports its state.
These commands reply `OK`, a state response, or `ERR <reason>`.

Both continuous modes require a keepalive within **five seconds**. Querying
`INFO`, `CAPS` or `GAIN?` does not renew the RF lease. A different radio
operation, including tuning, capture, or another upload, stops continuous
output first. `RELEASE` stops RF and releases the serial owner. Finite output,
explicit stops, and expired leases restore the receiver configuration. A
second serial client receives `ERR busy` while the owner holds the radio.

The restored implementation uses the original register settings. Gain,
output power, spectral quality and tuning outside characterized bands require
hardware measurement; host tests and a firmware build do not establish RF
performance.
