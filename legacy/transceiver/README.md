# C61/S31 USB and Ethernet transceiver

This directory is a standalone ESP-IDF project containing the original
transceiver application from `e60568437df0fac3a38764fb91eb2ed6d8db80c6`.
Its application sources, CMake files and SDK defaults are restored verbatim.
Build it from this directory, independently of the repository's current
burst firmware. For C5/S3, use the repository-root project, which integrates
transmit with the current receive controls and serial transport.

## Dependencies and build

The original PHY dependencies are pinned in `dependencies.json`:

| Component | Commit |
| --- | --- |
| `ESPARGOS/esp-open-phy-lib` | `2515d0891440cd63cde210dba1455d854f2698f4` |
| `ESPARGOS/esp32c61-phy-regs` | `406655303a2ac8e0528177c22c0b49890bff7cc1` |

From this directory, obtain them with:

```sh
python3 prepare_dependencies.py
```

The repositories returned `Repository not found` during restoration on
2026-09-30. If they are private or unavailable, supply authorized checkouts at
the exact paths and commits above, then run:

```sh
python3 prepare_dependencies.py --check
```

The script preserves existing checkouts and rejects a different commit.
Registry dependencies are declared in `main/idf_component.yml`; enable the
ESP-IDF component manager to download them. The custom TinyUSB component
expects `managed_components/espressif__tinyusb`.

For S31, activate its compatible preview SDK. The current repository pins
`26dd2948925d44f41e3bb614fb044b7a0839a242` for the S31 burst backend; the restored
streaming application still needs a complete build with its PHY dependencies
to establish SDK compatibility. The original S31 defaults require **16 MB
flash, octal PSRAM and the configured Ethernet/USB hardware**. They are not
the 2 MB burst-receiver defaults from the root project.

```sh
IDF_COMPONENT_MANAGER=1 idf.py --preview -B build-s31 \
  -DIDF_TARGET=esp32s31 -DSDKCONFIG=sdkconfig.s31 \
  -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32s31 build
idf.py --preview -B build-s31 -p /dev/ttyACM0 flash
```

C61 retains the original open-PHY development path and requires the matching
SDK and board configuration. The initial source provided no generic C61
transceiver defaults. Configure that project's transport and memory for the
actual board; the root project's C61 burst defaults do not describe this
streaming application. C61/S31 transceiver builds and powered RF operation
have not been verified as part of this source restoration.

## Entry points

- `main/main.c`: IQ upload ownership, replay, refill, scheduling and RF control.
- `main/modem.c`: tone generation, replay preparation and receive recovery.
- `main/iq_usb.c`, `main/iq_usb.h`: S31 vendor USB protocol, including
  `TX_ARM` (6), `TX_COMMIT` (7), `TX_ABORT` (8), and bulk OUT endpoint `0x02`.
- `main/iq_network.c`, `main/iq_network.h`: HTTP waveform upload at
  `PUT /api/v1/tx/waveform`, UDP arming at `POST /api/v1/tx/udp/arm`, and
  framed UDP IQ transport. See the headers for wire layouts and flags.
- `main/wifi_tx_rx.c`: Wi-Fi transmit and replay coordination.
- `main/s31_pie_copy.S`: accelerated S31 replay refill.
- `main/web/`: the embedded HTTP status and control interface.

The legacy USB/HTTP/UDP protocol is separate from the ASCII `TX16`/`REPLAY16`
commands of the current C5/S3 firmware. Flash the image matching the intended
transport and board configuration.
