# yeni_cihaz — SKAPP starter scaffold

Self-contained ESP-IDF project that boots the SKAPP Library baseline
(`sk_core` + `sk_api`). Use as the zero point for any new SmartKraft
device firmware. Drop this folder anywhere — no external dependencies
on the SKAPP repo.

## Layout

```
yeni_cihaz/
├── CMakeLists.txt          project-level (calls idf project.cmake)
├── sdkconfig.defaults      partition table, flash size, log level, etc.
├── main/
│   ├── CMakeLists.txt      registers main.c, requires sk_core + sk_api
│   └── main.c              Faz 0 starter — fill EDIT blocks at top
└── components/
    ├── sk_core/            mandatory — USB CLI, BLE, WiFi, mDNS, TCP,
    │                       auth, button/LED, OTA, identity, errors,
    │                       event bus, capabilities
    └── sk_api/             optional — outbound HTTP (IFTTT, generic,
                            webhook_post; POST/GET/PUT/DELETE)
```

## How to start a fresh device project

1. Copy this `yeni_cihaz/` folder to its final location and rename it
   to your device name (e.g. `my_device/`).
2. In the new folder, edit `CMakeLists.txt` → change `project(yeni_cihaz)`
   to `project(<your_device>)` (snake_case).
3. Open `main/main.c` and fill in the EDIT blocks at the top:
   - `SK_DEVICE_TYPE_PREFIX` — 2-char uppercase code (e.g. `"FC"`)
   - `SK_HW_REV` — `'A'` for first PCB
   - `SK_FW_VERSION` — semver string
   - `SK_BUTTON_GPIO` — control button pin (default 9 = ESP32-C6 boot)
   - `SK_TCP_PORT` — NDJSON TCP port (default 8080)
   - Toggle `SK_API_ENABLE` / `SK_OTA_ENABLE` as needed.
4. Add device-specific code below the `Device-specific code lives below`
   marker in `main.c` (sensors, timers, custom CLI commands, etc.).
5. If you don't need outbound HTTP: remove `sk_api` from
   `main/CMakeLists.txt` `PRIV_REQUIRES`, delete `components/sk_api/`,
   and drop the `#include "sk_api.h"` + `sk_api_init()` call in main.c.

## Build

```bash
cd yeni_cihaz
idf.py set-target esp32c6
idf.py build
idf.py -p <PORT> flash monitor
```

`<PORT>` is `COM3` (or similar) on Windows, `/dev/ttyUSB0` on Linux,
`/dev/cu.usbmodem*` on macOS.

## First-boot smoke test

Open the serial monitor at 115200 8N1. You should see:

```
sk> help
```

Type `help`, `device.capabilities`, `id` to verify sk_core is alive.
Press the control button briefly to start BLE advertising; scan from a
phone (e.g. nRF Connect) to confirm the device appears.

## Updating the SKAPP Library

The vendored `components/sk_core/` and `components/sk_api/` are a
point-in-time copy. To pull a newer version:

```bash
# from the source SKAPP repo
cp -r /path/to/SKAPP/esp32/skapp-library/sk_core components/sk_core
cp -r /path/to/SKAPP/esp32/skapp-library/sk_api  components/sk_api
```

Then re-build. Long-term: switch to git submodule or the Espressif
Component Registry once the library API is stable.
