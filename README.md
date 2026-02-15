## Host-side Tigard test script (Python)

This repo now includes a direct SPI test tool for JDI MIP panels using FTDI/Tigard:

- Script: `tools/mip_tigard_demo.py`
- Protocol: `CMD_UPDATE (0x90)`, `CMD_ALL_CLEAR (0x20)`, periodic VCOM toggle (`0x40`) to keep panel drive balanced.

### Setup

Install dependency:

```bash
pip install pyftdi
```

For GIF playback support:

```bash
pip install pillow
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

Play a GIF on the panel (auto-uses GIF frame timing):

```bash
python tools/mip_tigard_demo.py --url ftdi://ftdi:2232h/2 --gif reference/demo.gif --disp-bit 5
```

Play a GIF at fixed FPS and fixed loop count:

```bash
python tools/mip_tigard_demo.py --url ftdi://ftdi:2232h/2 --gif reference/demo.gif --gif-fps 8 --gif-loops 3 --disp-bit 5
```

