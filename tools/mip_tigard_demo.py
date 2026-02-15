#!/usr/bin/env python3
"""
Host-side JDI MIP panel test tool for FTDI/Tigard via PyFtdi.

Protocol matches the local JDI reference driver:
- CMD_UPDATE:     0x90
- CMD_ALL_CLEAR:  0x20
- CMD_VCOM bit:   0x40 (toggled periodically)
- CMD_NO_UPDATE:  0x00

Target defaults are for LPM009M360A (72x144, 4-bit packed pixels).
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable
from pathlib import Path

try:
    from pyftdi.spi import SpiController
except Exception as exc:  # pragma: no cover
    print(f"Failed to import pyftdi: {exc}")
    print("Install with: pip install pyftdi")
    sys.exit(2)

try:
    from PIL import Image, ImageSequence
except Exception:
    Image = None
    ImageSequence = None

CMD_NO_UPDATE = 0x00
CMD_ALL_CLEAR = 0x20
CMD_VCOM = 0x40
CMD_UPDATE = 0x90

COLOR_BLACK = 0x0
COLOR_BLUE = 0x2
COLOR_GREEN = 0x4
COLOR_CYAN = 0x6
COLOR_RED = 0x8
COLOR_MAGENTA = 0xA
COLOR_YELLOW = 0xC
COLOR_WHITE = 0xE

PALETTE_RGB_TO_MIP4 = (
    ((0, 0, 0), COLOR_BLACK),
    ((0, 0, 255), COLOR_BLUE),
    ((0, 255, 0), COLOR_GREEN),
    ((0, 255, 255), COLOR_CYAN),
    ((255, 0, 0), COLOR_RED),
    ((255, 0, 255), COLOR_MAGENTA),
    ((255, 255, 0), COLOR_YELLOW),
    ((255, 255, 255), COLOR_WHITE),
)


def clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(value, hi))


def rgb_to_mip4(r: int, g: int, b: int) -> int:
    best = COLOR_BLACK
    best_dist = 1 << 62
    for (pr, pg, pb), c in PALETTE_RGB_TO_MIP4:
        dist = (r - pr) * (r - pr) + (g - pg) * (g - pg) + (b - pb) * (b - pb)
        if dist < best_dist:
            best_dist = dist
            best = c
    return best


class FtdiGpioBus:
    def __init__(self, spi_ctrl: SpiController):
        self.gpio = spi_ctrl.get_gpio()
        self.direction = 0
        self.value = 0

    @staticmethod
    def _normalize_bit(bit: int) -> int:
        if 0 <= bit <= 3:
            return bit + 4
        if 4 <= bit <= 7:
            return bit
        raise ValueError("GPIO bit must be in 0..3 (alias for B4..B7) or 4..7 (FTDI B4..B7)")

    def _apply(self) -> None:
        self.gpio.set_direction(self.direction, self.direction)
        self.gpio.write(self.value)

    def claim_output(self, bit: int, initial_high: bool = False) -> None:
        bit = self._normalize_bit(bit)
        mask = 1 << bit
        self.direction |= mask
        if initial_high:
            self.value |= mask
        else:
            self.value &= ~mask
        self._apply()

    def set_bit(self, bit: int, high: bool) -> None:
        bit = self._normalize_bit(bit)
        mask = 1 << bit
        if high:
            self.value |= mask
        else:
            self.value &= ~mask
        self.gpio.write(self.value)

    def read_bit(self, bit: int) -> bool:
        bit = self._normalize_bit(bit)
        return bool((self.read() >> bit) & 0x1)

    def read(self) -> int:
        return int(self.gpio.read(with_output=True))


class DispDriver:
    def __init__(self, gpio_bus: FtdiGpioBus, bit: int, active_high: bool):
        self.bus = gpio_bus
        self.bit = bit
        self.active_high = active_high
        self.enabled = False
        self.bus.claim_output(bit, initial_high=False)

    def _drive(self, logical_on: bool) -> None:
        out_high = logical_on if self.active_high else (not logical_on)
        self.bus.set_bit(self.bit, out_high)

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        self._drive(enabled)


class ExtcominDriver:
    def __init__(self, gpio_bus: FtdiGpioBus, bit: int, period_s: float, active_high: bool):
        if bit < 0:
            raise ValueError("EXTCOMIN bit must be >= 0")
        self.bus = gpio_bus
        self.bit = bit
        self.period_s = max(0.05, period_s)
        self.active_high = active_high
        self.level = False
        self.next_toggle = time.monotonic() + self.period_s

        self.bus.claim_output(bit, initial_high=False)
        self._write_level(False)

    def _write_level(self, high: bool) -> None:
        drive_high = high if self.active_high else (not high)
        self.bus.set_bit(self.bit, drive_high)

    def service(self, now: float) -> None:
        if now < self.next_toggle:
            return
        self.level = not self.level
        self._write_level(self.level)
        self.next_toggle = now + self.period_s

    def shutdown(self) -> None:
        self._write_level(False)


class ManualCsDriver:
    def __init__(self, gpio_bus: FtdiGpioBus, bit: int, active_high: bool):
        self.bus = gpio_bus
        self.bit = bit
        self.active_high = active_high
        self.asserted = False
        self.bus.claim_output(bit, initial_high=False)
        self.deassert_cs()

    def _drive(self, asserted: bool) -> None:
        out_high = asserted if self.active_high else (not asserted)
        self.bus.set_bit(self.bit, out_high)

    def assert_cs(self) -> None:
        self.asserted = True
        self._drive(True)

    def deassert_cs(self) -> None:
        self.asserted = False
        self._drive(False)


class MipDisplay:
    def __init__(
        self,
        spi_port,
        width: int,
        height: int,
        trace_spi: bool = False,
        trace_packets: int = 0,
        manual_cs: ManualCsDriver | None = None,
        invert_colors: bool = False,
    ):
        if width % 2 != 0:
            raise ValueError("Width must be even for 4-bit packed pixels")
        self.spi = spi_port
        self.w = width
        self.h = height
        self.line_bytes = width // 2
        self.vcom = 0
        self.fb = bytearray(self.line_bytes * self.h)
        self.trace_spi = trace_spi
        self.trace_packets_remaining = max(0, trace_packets)
        self.tx_packets = 0
        self.tx_bytes = 0
        self.manual_cs = manual_cs
        self.invert_colors = invert_colors

    @staticmethod
    def _invert_nibble(color: int) -> int:
        return (color ^ 0x0F) & 0x0F

    def _xfer(self, payload: bytes | bytearray) -> None:
        self.tx_packets += 1
        self.tx_bytes += len(payload)
        if self.trace_spi and self.trace_packets_remaining > 0:
            self.trace_packets_remaining -= 1
            preview = bytes(payload[: min(24, len(payload))]).hex(" ")
            print(f"SPI TX[{self.tx_packets}] len={len(payload)}: {preview}")
        if self.manual_cs is not None:
            self.manual_cs.assert_cs()
            self.spi.write(payload, start=False, stop=False)
            self.manual_cs.deassert_cs()
        else:
            self.spi.write(payload)

    def _cmd2(self, cmd: int) -> None:
        self._xfer(bytes((cmd & 0xFF, 0x00)))

    def toggle_vcom_keepalive(self) -> None:
        self.vcom = CMD_VCOM if self.vcom == 0 else 0
        self._cmd2(CMD_NO_UPDATE | self.vcom)

    def clear_panel(self) -> None:
        self._cmd2(CMD_ALL_CLEAR | self.vcom)

    def fill(self, color: int) -> None:
        if self.invert_colors:
            color = self._invert_nibble(color)
        packed = ((color & 0x0F) << 4) | (color & 0x0F)
        self.fb[:] = bytes((packed,)) * len(self.fb)

    def set_framebuffer(self, packed_fb: bytes | bytearray) -> None:
        if len(packed_fb) != len(self.fb):
            raise ValueError("Packed framebuffer size mismatch")
        if self.invert_colors:
            for i, value in enumerate(packed_fb):
                hi = ((value >> 4) & 0x0F) ^ 0x0F
                lo = (value & 0x0F) ^ 0x0F
                self.fb[i] = (hi << 4) | lo
        else:
            self.fb[:] = packed_fb

    def set_pixel(self, x: int, y: int, color: int) -> None:
        if x < 0 or x >= self.w or y < 0 or y >= self.h:
            return
        idx = y * self.line_bytes + (x // 2)
        if self.invert_colors:
            color = self._invert_nibble(color)
        if (x & 1) == 0:
            self.fb[idx] = (self.fb[idx] & 0x0F) | ((color & 0x0F) << 4)
        else:
            self.fb[idx] = (self.fb[idx] & 0xF0) | (color & 0x0F)

    def fill_rect(self, x: int, y: int, w: int, h: int, color: int) -> None:
        x0 = clamp(x, 0, self.w)
        y0 = clamp(y, 0, self.h)
        x1 = clamp(x + w, 0, self.w)
        y1 = clamp(y + h, 0, self.h)
        for yy in range(y0, y1):
            for xx in range(x0, x1):
                self.set_pixel(xx, yy, color)

    def _line_bytes(self, y: int) -> memoryview:
        start = y * self.line_bytes
        end = start + self.line_bytes
        return memoryview(self.fb)[start:end]

    def refresh(self) -> None:
        before_packets = self.tx_packets
        before_bytes = self.tx_bytes
        hdr = CMD_UPDATE | self.vcom
        for y in range(self.h):
            row = bytearray(self.line_bytes + 4)
            row[0] = hdr
            row[1] = y + 1
            row[2 : 2 + self.line_bytes] = self._line_bytes(y)
            row[-2] = 0x00
            row[-1] = 0x00
            self._xfer(row)
        print(
            f"Refresh sent: packets={self.tx_packets - before_packets}, bytes={self.tx_bytes - before_bytes}, "
            f"hdr=0x{hdr:02x}"
        )


def draw_bars(display: MipDisplay) -> None:
    colors: Iterable[int] = (
        COLOR_BLACK,
        COLOR_BLUE,
        COLOR_GREEN,
        COLOR_CYAN,
        COLOR_RED,
        COLOR_MAGENTA,
        COLOR_YELLOW,
        COLOR_WHITE,
    )
    colors = tuple(colors)
    bar_h = max(1, display.h // len(colors))
    for i, c in enumerate(colors):
        display.fill_rect(0, i * bar_h, display.w, bar_h, c)


def draw_checker(display: MipDisplay, tile: int = 8) -> None:
    for y in range(display.h):
        for x in range(display.w):
            cell = ((x // tile) + (y // tile)) & 1
            display.set_pixel(x, y, COLOR_WHITE if cell else COLOR_BLACK)


def draw_anim(display: MipDisplay, frame: int) -> None:
    draw_bars(display)

    box = min(display.w // 2, display.h // 4)
    box = max(box, 10)
    xr = max(1, display.w - box)
    yr = max(1, display.h - box)
    period = xr * 2
    t = frame % period
    x = t if t < xr else (period - t)
    y = (frame * 2) % yr

    display.fill_rect(x, y, box, box, COLOR_WHITE)
    display.fill_rect(x + 2, y + 2, max(1, box - 4), max(1, box - 4), COLOR_BLACK)

    for i in range(0, display.h, 6):
        display.set_pixel((i + frame) % display.w, i, COLOR_RED)


def frame_image_to_packed(frame_image, width: int, height: int) -> bytearray:
    resized = frame_image.convert("RGB").resize((width, height), resample=Image.NEAREST)
    rgb_data = resized.tobytes()
    line_bytes = width // 2
    packed = bytearray(line_bytes * height)

    src = 0
    dst = 0
    for _y in range(height):
        for _x in range(0, width, 2):
            r0, g0, b0 = rgb_data[src], rgb_data[src + 1], rgb_data[src + 2]
            src += 3
            r1, g1, b1 = rgb_data[src], rgb_data[src + 1], rgb_data[src + 2]
            src += 3
            c0 = rgb_to_mip4(r0, g0, b0)
            c1 = rgb_to_mip4(r1, g1, b1)
            packed[dst] = ((c0 & 0x0F) << 4) | (c1 & 0x0F)
            dst += 1
    return packed


def load_gif_frames(path: str, width: int, height: int) -> tuple[list[bytearray], list[float]]:
    if Image is None or ImageSequence is None:
        raise SystemExit("GIF support requires Pillow. Install with: pip install pillow")
    gif_path = Path(path)
    if not gif_path.exists():
        raise SystemExit(f"GIF file not found: {gif_path}")

    packed_frames: list[bytearray] = []
    delays_s: list[float] = []

    with Image.open(gif_path) as image:
        for frame in ImageSequence.Iterator(image):
            packed_frames.append(frame_image_to_packed(frame, width, height))
            delay_ms = frame.info.get("duration", image.info.get("duration", 100))
            delays_s.append(max(0.02, float(delay_ms) / 1000.0))

    if not packed_frames:
        raise SystemExit(f"No frames found in GIF: {gif_path}")

    return packed_frames, delays_s


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Drive JDI MIP display over Tigard/FTDI SPI")
    parser.add_argument("--url", default="ftdi://ftdi:2232h/1", help="PyFtdi URL (default: %(default)s)")
    parser.add_argument("--cs", type=int, default=0, help="FTDI CS index on selected interface")
    parser.add_argument("--hz", type=int, default=4_000_000, help="SPI clock Hz")
    parser.add_argument("--mode", type=int, default=0, choices=[0, 1, 2, 3], help="SPI mode")
    parser.add_argument(
        "--cs-active-high",
        action="store_true",
        help="Use manual active-high CS via GPIO (hardware CS on B3 is bypassed)",
    )
    parser.add_argument(
        "--cs-gpio-bit",
        type=int,
        default=-1,
        help="GPIO bit for manual CS when --cs-active-high is set (4..7, or alias 0..3 => B4..B7)",
    )
    parser.add_argument("--width", type=int, default=72, help="Display width")
    parser.add_argument("--height", type=int, default=144, help="Display height")
    parser.add_argument("--fps", type=float, default=5.0, help="Animation FPS")
    parser.add_argument("--frames", type=int, default=0, help="Animation frame count, 0 = forever")
    parser.add_argument("--pattern", choices=["bars", "checker", "anim"], default="anim")
    parser.add_argument("--gif", type=str, default="", help="Path to GIF file to stream to display")
    parser.add_argument(
        "--gif-loops",
        type=int,
        default=0,
        help="Number of GIF loops (0 = forever)",
    )
    parser.add_argument(
        "--gif-fps",
        type=float,
        default=0.0,
        help="Override GIF timing with fixed FPS (<=0 uses GIF frame durations)",
    )
    parser.add_argument("--clear-first", action="store_true", help="Send panel clear command before drawing")
    parser.add_argument("--hold-seconds", type=float, default=10.0, help="Keep VCOM toggling after static draw")
    parser.add_argument(
        "--disp-bit",
        type=int,
        default=-1,
        help="FTDI GPIO bit for DISP (disable with -1). Accepts 4..7 (Tigard B4..B7), or alias 0..3 => B4..B7",
    )
    parser.add_argument("--disp-active-low", action="store_true", help="Drive DISP as active-low")
    parser.add_argument("--leave-disp-on", action="store_true", help="Do not force DISP off on exit")
    parser.add_argument(
        "--extcomin-bit",
        type=int,
        default=-1,
        help="FTDI GPIO bit for EXTCOMIN (disable with -1). Accepts 4..7 (Tigard B4..B7), or alias 0..3 => B4..B7",
    )
    parser.add_argument("--extcomin-period", type=float, default=0.5, help="EXTCOMIN toggle period in seconds")
    parser.add_argument("--extcomin-active-low", action="store_true", help="Drive EXTCOMIN as active-low")
    parser.add_argument("--trace-spi", action="store_true", help="Print SPI transfer summaries")
    parser.add_argument("--invert-colors", action="store_true", help="Invert 4-bit panel colors before transmit")
    parser.add_argument(
        "--trace-packets",
        type=int,
        default=6,
        help="When --trace-spi is enabled, print hex preview for the first N packets",
    )
    parser.add_argument(
        "--verify-gpio",
        action="store_true",
        help="Only verify DISP/EXTCOMIN pin generation by toggling and printing observed FTDI GPIO states",
    )
    parser.add_argument(
        "--print-gpio-capabilities",
        action="store_true",
        help="Print available FTDI SPI GPIO pins/bitfields for selected interface and exit",
    )
    parser.add_argument("--verify-seconds", type=float, default=5.0, help="Duration for --verify-gpio")
    parser.add_argument(
        "--loopback-extcomin-bit",
        type=int,
        default=-1,
        help="Optional FTDI GPIO input bit looped from EXTCOMIN for readback verification (4..7, or alias 0..3 => B4..B7)",
    )
    parser.add_argument(
        "--loopback-disp-bit",
        type=int,
        default=-1,
        help="Optional FTDI GPIO input bit looped from DISP for readback verification (4..7, or alias 0..3 => B4..B7)",
    )
    return parser.parse_args()


def run_gpio_verification(
    gpio_bus: FtdiGpioBus,
    extcomin: ExtcominDriver | None,
    disp: DispDriver | None,
    duration_s: float,
    loopback_extcomin_bit: int,
    loopback_disp_bit: int,
) -> int:
    stop_time = time.monotonic() + max(0.5, duration_s)
    next_log = time.monotonic()
    next_disp_toggle = time.monotonic() + 0.25

    disp_mismatch = 0
    ext_mismatch = 0
    samples = 0

    while time.monotonic() < stop_time:
        now = time.monotonic()
        if extcomin is not None:
            extcomin.service(now)
        if disp is not None and now >= next_disp_toggle:
            disp.set_enabled(not disp.enabled)
            next_disp_toggle = now + 0.25

        read_val = gpio_bus.read()
        samples += 1

        if loopback_extcomin_bit >= 0 and extcomin is not None:
            expected = extcomin.level
            observed = gpio_bus.read_bit(loopback_extcomin_bit)
            if observed != expected:
                ext_mismatch += 1

        if loopback_disp_bit >= 0 and disp is not None:
            expected = disp.enabled
            observed = gpio_bus.read_bit(loopback_disp_bit)
            if observed != expected:
                disp_mismatch += 1

        if now >= next_log:
            print(
                f"GPIO raw=0x{read_val:04x} "
                f"DISP={'NA' if disp is None else int(disp.enabled)} "
                f"EXTCOMIN={'NA' if extcomin is None else int(extcomin.level)}"
            )
            next_log = now + 0.5

        time.sleep(0.01)

    print(f"Verification complete: samples={samples}")
    if loopback_extcomin_bit >= 0 and extcomin is not None:
        print(f"EXTCOMIN loopback mismatches: {ext_mismatch}")
    if loopback_disp_bit >= 0 and disp is not None:
        print(f"DISP loopback mismatches: {disp_mismatch}")

    return 0


def main() -> int:
    args = parse_args()

    ctrl = SpiController()
    ctrl.configure(args.url)
    spi = ctrl.get_port(cs=args.cs, freq=args.hz, mode=args.mode)
    gpio_bus = FtdiGpioBus(ctrl)

    if args.print_gpio_capabilities:
        gpio = ctrl.get_gpio()
        print(f"Interface: {args.url}")
        print(f"GPIO width: {gpio.width}")
        print(f"Configured GPIO pins bitfield: 0x{gpio.pins:04x}")
        print(f"Addressable GPIO pins bitfield: 0x{gpio.all_pins:04x}")
        print("Note: SPI pins b0..b3 are reserved; free pins are typically b4..b7 on narrow ports.")
        ctrl.terminate()
        return 0

    manual_cs = None
    if args.cs_active_high:
        if args.cs_gpio_bit < 0:
            raise SystemExit("--cs-active-high requires --cs-gpio-bit")
        manual_cs = ManualCsDriver(gpio_bus, bit=args.cs_gpio_bit, active_high=True)

    disp = None
    if args.disp_bit >= 0:
        disp = DispDriver(gpio_bus, bit=args.disp_bit, active_high=not args.disp_active_low)
        disp.set_enabled(True)

    extcomin = None

    if args.extcomin_bit >= 0:
        extcomin = ExtcominDriver(
            gpio_bus,
            bit=args.extcomin_bit,
            period_s=args.extcomin_period,
            active_high=not args.extcomin_active_low,
        )

    display = MipDisplay(
        spi,
        width=args.width,
        height=args.height,
        trace_spi=args.trace_spi,
        trace_packets=args.trace_packets,
        manual_cs=manual_cs,
        invert_colors=args.invert_colors,
    )

    gif_frames = None
    gif_delays = None
    if args.gif:
        gif_frames, gif_delays = load_gif_frames(args.gif, args.width, args.height)
        print(f"Loaded GIF frames: {len(gif_frames)} from {args.gif}")

    print(f"Connected: {args.url}, CS{args.cs}, {args.hz}Hz, mode{args.mode}")
    if manual_cs is not None:
        print(f"Manual CS enabled on GPIO bit {args.cs_gpio_bit} (active-high), hardware CS bypassed")
    if disp is not None:
        print(f"DISP driven on FTDI GPIO bit {args.disp_bit} ({'active-low' if args.disp_active_low else 'active-high'})")
    else:
        print("DISP not driven by FTDI (external drive required)")
    if extcomin is not None:
        print(f"EXTCOMIN driven on FTDI GPIO bit {args.extcomin_bit}, period {args.extcomin_period:.3f}s")
    else:
        print("EXTCOMIN not enabled; using SPI VCOM keepalive/update commands")
    if args.invert_colors:
        print("Color inversion enabled")

    if args.verify_gpio:
        print("GPIO verify mode: SPI display data is NOT sent in this mode.")
        try:
            return run_gpio_verification(
                gpio_bus,
                extcomin,
                disp,
                duration_s=args.verify_seconds,
                loopback_extcomin_bit=args.loopback_extcomin_bit,
                loopback_disp_bit=args.loopback_disp_bit,
            )
        finally:
            if extcomin is not None:
                extcomin.shutdown()
            if manual_cs is not None:
                manual_cs.deassert_cs()
            if disp is not None and not args.leave_disp_on:
                disp.set_enabled(False)
            ctrl.terminate()

    if args.clear_first:
        display.clear_panel()

    frame_delay = 1.0 / max(args.fps, 0.5)

    if args.pattern in ("bars", "checker") and not gif_frames:
        if args.pattern == "bars":
            draw_bars(display)
        else:
            draw_checker(display)
        display.refresh()
        print(f"Drew static pattern: {args.pattern}")

        stop_time = time.monotonic() + max(0.0, args.hold_seconds)
        while time.monotonic() < stop_time:
            if extcomin is not None:
                extcomin.service(time.monotonic())
                time.sleep(0.01)
            else:
                display.toggle_vcom_keepalive()
                time.sleep(0.5)
        if extcomin is not None:
            extcomin.shutdown()
        if manual_cs is not None:
            manual_cs.deassert_cs()
        if disp is not None and not args.leave_disp_on:
            disp.set_enabled(False)
        ctrl.terminate()
        return 0

    if gif_frames:
        frame = 0
        gif_index = 0
        loop_index = 0
        max_frames = args.frames if args.frames > 0 else None
        next_spi_vcom = time.monotonic() + 0.5
        try:
            while True:
                if max_frames is not None and frame >= max_frames:
                    break
                if args.gif_loops > 0 and loop_index >= args.gif_loops:
                    break

                now = time.monotonic()
                if extcomin is not None:
                    display.vcom = 0
                    extcomin.service(now)
                else:
                    if now >= next_spi_vcom:
                        display.vcom = CMD_VCOM if display.vcom == 0 else 0
                        next_spi_vcom = now + 0.5

                display.set_framebuffer(gif_frames[gif_index])
                display.refresh()
                frame += 1

                if args.gif_fps > 0:
                    delay_s = 1.0 / max(args.gif_fps, 0.5)
                else:
                    delay_s = gif_delays[gif_index]

                gif_index += 1
                if gif_index >= len(gif_frames):
                    gif_index = 0
                    loop_index += 1

                deadline = time.monotonic() + delay_s
                while time.monotonic() < deadline:
                    if extcomin is not None:
                        extcomin.service(time.monotonic())
                        time.sleep(0.01)
                    else:
                        now = time.monotonic()
                        if now >= next_spi_vcom:
                            display.toggle_vcom_keepalive()
                            next_spi_vcom = now + 0.5
                        time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            if extcomin is not None:
                extcomin.shutdown()
            if manual_cs is not None:
                manual_cs.deassert_cs()
            if disp is not None and not args.leave_disp_on:
                disp.set_enabled(False)
            ctrl.terminate()

        print(f"Sent {frame} GIF frames")
        return 0

    frame = 0
    max_frames = args.frames if args.frames > 0 else None
    next_spi_vcom = time.monotonic() + 0.5
    try:
        while max_frames is None or frame < max_frames:
            if extcomin is not None:
                display.vcom = 0
                extcomin.service(time.monotonic())
            else:
                now = time.monotonic()
                if now >= next_spi_vcom:
                    display.vcom = CMD_VCOM if display.vcom == 0 else 0
                    next_spi_vcom = now + 0.5
            draw_anim(display, frame)
            display.refresh()
            frame += 1
            time.sleep(frame_delay)
    except KeyboardInterrupt:
        pass
    finally:
        if extcomin is not None:
            extcomin.shutdown()
        if manual_cs is not None:
            manual_cs.deassert_cs()
        if disp is not None and not args.leave_disp_on:
            disp.set_enabled(False)
        ctrl.terminate()

    print(f"Sent {frame} frames")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
