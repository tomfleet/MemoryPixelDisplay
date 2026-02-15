| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- |

# SPI Host Driver Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example aims to show how to use SPI Host driver API, like `spi_transaction_t` and spi_device_queue.

If you are looking for code to drive LCDs in general, rather than code that uses the SPI master, that may be a better example to look at as it uses ESP-IDFs built-in LCD support rather than doing all the low-level work itself, which can be found at `examples/peripherals/lcd/tjpgd/`

## How to Use Example

### Hardware Required

* An ESP development board, with SPI LCD

**Connection** :

Depends on boards. The GPIO number used by this example can be changed in `spi_master_example_main.c` No wiring is required on ESP-WROVER-KIT

Especially, please pay attention to the level used to turn on the LCD backlight, some LCD module needs a low level to turn it on, while others take a high level. You can change the backlight level macro LCD_BK_LIGHT_ON_LEVEL in `spi_master_example_main.c`.

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

On ESP-WROVER-KIT there will be:

```
LCD ID: 00000000
ILI9341 detected.
LCD ILI9341 initialization.
```

At the meantime `ESP32` will be displayed on the connected LCD screen.

## Troubleshooting

For any technical queries, please open an [issue] (https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.

## Host-side Tigard test script (Python)

This repo now includes a direct SPI test tool for JDI MIP panels using FTDI/Tigard:

- Script: `tools/mip_tigard_demo.py`
- Protocol: `CMD_UPDATE (0x90)`, `CMD_ALL_CLEAR (0x20)`, periodic VCOM toggle (`0x40`) to keep panel drive balanced.

### Setup

Install dependency:

```bash
pip install pyftdi
```

Make sure panel control pins are wired correctly for host-side test:

- `SCLK`, `SI/MOSI`, `SCS` to Tigard SPI
- panel power rails connected (`VDD/VDDA` and `GND/VSS` as required by your module)
- `DISP` held HIGH
- optional: route `EXTCOMIN` to a spare Tigard/FTDI GPIO if you want external COM inversion drive
- `FRONTLIGHT` is optional (not used by this script)
- common ground between panel and Tigard

### Examples

Animated demo (LPM009M360A defaults 72x144):

```bash
python tools/mip_tigard_demo.py --url ftdi://ftdi:2232h/1 --cs 0 --pattern anim --fps 6
```

Animated demo with EXTCOMIN on FTDI GPIO bit 4 (toggle every 0.5 s):

```bash
python tools/mip_tigard_demo.py --url ftdi://ftdi:2232h/1 --cs 0 --pattern anim --extcomin-bit 4 --extcomin-period 0.5
```

Drive `DISP` from Tigard GPIO bit 5 and keep it on while running:

```bash
python tools/mip_tigard_demo.py --pattern anim --disp-bit 5 --extcomin-bit 4
```

GPIO generation verification only (no display update), with loopback readback checks:

```bash
python tools/mip_tigard_demo.py --verify-gpio --verify-seconds 8 --disp-bit 5 --extcomin-bit 4 --loopback-disp-bit 6 --loopback-extcomin-bit 7
```

In the loopback example above, wire the generated output pins to spare Tigard input bits (`5->6`, `4->7`) to verify toggling behavior numerically.

Single static bars pattern, then keep VCOM alive for 20 seconds:

```bash
python tools/mip_tigard_demo.py --pattern bars --hold-seconds 20 --clear-first
```

Checkerboard on a different FTDI URL / dimensions:

```bash
python tools/mip_tigard_demo.py --url ftdi://ftdi:2232h/2 --width 72 --height 144 --pattern checker
```
