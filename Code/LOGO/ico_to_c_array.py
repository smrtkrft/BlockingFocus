"""ICO -> 1-bit C array converter for SH1107 OLED.

Reads BF/logo/B_LOGO_T.ico, picks the largest size inside, resizes to a
target square (default 64x64), thresholds to 1-bit, and writes a C
header file with the bitmap in SH1107 page-major layout (each byte = 8
vertical pixels, LSB at top).

Run from IDF env:
    python ico_to_c_array.py
"""

import os
import sys
from pathlib import Path

from PIL import Image

THIS_DIR = Path(__file__).parent
ICO_PATH = THIS_DIR / "B_LOGO_T.ico"
OUT_HEADER = THIS_DIR.parent / "components" / "bf_ui" / "include" / "bf_ui_logo.h"

TARGET_SIZE = 80    # 80x80 (~25% larger than 64) — fits with SmartKraft below on 128x128
THRESHOLD = 96      # luma >= this becomes "ON" (light pixels = lit OLED pixel)
INVERT = False      # set True if the rendered logo comes out as a negative


def load_largest_frame(ico_path: Path) -> Image.Image:
    """Pillow can iterate frames inside an ICO; pick the highest-res one."""
    img = Image.open(ico_path)
    if img.format != "ICO":
        # Pillow may still load other formats — accept whatever Image opens.
        return img
    sizes = img.ico.sizes()  # set of (w,h) tuples
    largest = max(sizes, key=lambda wh: wh[0] * wh[1])
    img.size = largest
    return img.copy()


def to_1bit(img: Image.Image, target: int, threshold: int, invert: bool) -> bytearray:
    # Composite onto BLACK so the foreground keeps its luminance:
    #   - white/colored logo on transparent → light pixels in luma image → ON
    #   - dark logo on transparent → dark pixels → use INVERT=True instead
    # Then convert to luma + resize.
    rgba = img.convert("RGBA")
    bg = Image.new("RGBA", rgba.size, (0, 0, 0, 255))
    flat = Image.alpha_composite(bg, rgba).convert("L")
    flat = flat.resize((target, target), Image.LANCZOS)

    pixels = flat.load()
    pages = target // 8
    fb = bytearray(target * pages)
    on_count = 0
    for page in range(pages):
        for x in range(target):
            byte = 0
            for bit in range(8):
                y = page * 8 + bit
                v = pixels[x, y]
                on = v >= threshold      # light source pixel → ON
                if invert:
                    on = not on
                if on:
                    byte |= (1 << bit)
                    on_count += 1
            fb[page * target + x] = byte

    # ASCII preview so we can verify visually before flashing.
    print()
    print(f"Bitmap preview ({target}×{target}, {on_count} ON pixels):")
    for y in range(target):
        line = []
        for x in range(target):
            page = y // 8
            bit = y % 8
            on = (fb[page * target + x] >> bit) & 1
            line.append("##" if on else "  ")
        print("  " + "".join(line))
    print()
    return fb


def emit_header(fb: bytearray, target: int, out_path: Path) -> None:
    cols = 16
    body = []
    for i, b in enumerate(fb):
        body.append(f"0x{b:02X}")
    rows = []
    for i in range(0, len(body), cols):
        rows.append("    " + ", ".join(body[i:i + cols]) + ",")
    body_str = "\n".join(rows).rstrip(",")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        "// Auto-generated from BF/logo/B_LOGO_T.ico by ico_to_c_array.py.\n"
        "// SH1107 page-major: each byte = 8 vertical pixels, LSB at top.\n"
        "// Layout: BF_UI_LOGO_W columns x (BF_UI_LOGO_H/8) pages.\n"
        "#pragma once\n"
        "#include <stdint.h>\n\n"
        f"#define BF_UI_LOGO_W {target}\n"
        f"#define BF_UI_LOGO_H {target}\n\n"
        f"static const uint8_t BF_UI_LOGO[{len(fb)}] = {{\n"
        f"{body_str}\n"
        "};\n",
        encoding="utf-8",
    )


def main() -> int:
    if not ICO_PATH.exists():
        print(f"ICO not found: {ICO_PATH}", file=sys.stderr)
        return 1

    print(f"Reading {ICO_PATH}")
    img = load_largest_frame(ICO_PATH)
    print(f"Source size: {img.size}, mode: {img.mode}")

    fb = to_1bit(img, TARGET_SIZE, THRESHOLD, INVERT)
    print(f"Converted to {TARGET_SIZE}x{TARGET_SIZE} 1-bit, {len(fb)} bytes")

    emit_header(fb, TARGET_SIZE, OUT_HEADER)
    print(f"Wrote {OUT_HEADER}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
