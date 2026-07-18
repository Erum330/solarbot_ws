"""
make_texture.py  –  Solar panel texture generator for SolarBot simulation
==========================================================================
Generates 10 unique 1024×1024 PNG textures, one per solar panel.
Each texture shows the photovoltaic cell grid with a procedural dust overlay.

Changes from original:
  1. Dust colour corrected for UAE dust: fine calcium-carbonate/silica,
     closer to pale off-white/beige than orange-brown.
  2. Added 'clean' density option (no dust, bare panel for reference).
  3. Panel 00 generates a clean reference texture.
  4. Panels 01-10 retain the original DENSITY_CYCLE.

Run from anywhere:
    python3 make_texture.py
Output: ../materials/textures/  (relative to this script)
"""

from PIL import Image, ImageDraw, ImageFilter
import random
import math
import os

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', 'materials', 'textures')
W, H = 1024, 1024
BASE_SEED = 7391

# Density applied to each panel (unchanged from original)
DENSITY_CYCLE = [
    "light",
    "patchy",
    "heavy",
    "streaked",
    "medium",
    "heavy",
    "light",
    "streaked",
    "patchy",
    "medium",
]


# ── Base panel grid ────────────────────────────────────────────────

def draw_base_panel(draw, w, h):
    """Dark blue photovoltaic cell grid — the clean panel appearance."""
    cols, rows = 6, 12
    # Background — deep navy photovoltaic colour
    draw.rectangle([0, 0, w - 1, h - 1], fill=(32, 58, 105))
    # Outer aluminium frame border
    draw.rectangle([0, 0, w - 1, h - 1], outline=(200, 200, 200), width=8)
    # Primary cell grid lines (silver-white)
    for i in range(1, cols):
        x = int(i * w / cols)
        draw.line([(x, 0), (x, h)], fill=(210, 210, 210), width=4)
    for j in range(1, rows):
        y = int(j * h / rows)
        draw.line([(0, y), (w, y)], fill=(210, 210, 210), width=4)
    # Sub-cell busbars (faint blue-grey)
    subcols, subrows = cols * 2, rows * 2
    for i in range(1, subcols):
        x = int(i * w / subcols)
        draw.line([(x, 0), (x, h)], fill=(60, 90, 140), width=1)
    for j in range(1, subrows):
        y = int(j * h / subrows)
        draw.line([(0, y), (w, y)], fill=(60, 90, 140), width=1)


# ── UAE dust colour ────────────────────────────────────────────────
#
# UAE dust is very fine calcium carbonate + silica (calcareous).
# It looks pale beige / off-white, NOT sandy orange.
# Original range: (155-200, 135-172, 82-128) — too warm/orange.
# Corrected range: (185-215, 175-200, 148-175) — pale beige/white.

def _dust_colour(rng, alpha):
    return (
        rng.randint(185, 215),   # R — pale
        rng.randint(175, 200),   # G — near-white
        rng.randint(148, 175),   # B — slight warm tint
        alpha,
    )


# ── Dust pattern primitives ────────────────────────────────────────

def _add_blobs(ddraw, rng, w, h, count_range, alpha_range):
    for _ in range(rng.randint(*count_range)):
        cx    = rng.randint(0, w)
        cy    = rng.randint(0, h)
        rx    = rng.randint(50, 190)
        ry    = rng.randint(35, 130)
        angle = rng.uniform(0, math.pi)
        alpha = rng.randint(*alpha_range)
        colour = _dust_colour(rng, alpha)
        pts = []
        for t in range(0, 360, 8):
            rad = math.radians(t)
            x = cx + rx * math.cos(rad) * math.cos(angle) - ry * math.sin(rad) * math.sin(angle)
            y = cy + rx * math.cos(rad) * math.sin(angle) + ry * math.sin(rad) * math.cos(angle)
            pts.append((x, y))
        ddraw.polygon(pts, fill=colour)


def _add_streaks(ddraw, rng, w, h, count_range, alpha_range):
    """Wind-driven vertical streaks (rain-washed dust trails)."""
    for _ in range(rng.randint(*count_range)):
        x0       = rng.randint(0, w)
        y0       = rng.randint(0, h // 3)
        length   = rng.randint(80, 380)
        width_px = rng.randint(3, 20)
        alpha    = rng.randint(*alpha_range)
        colour   = _dust_colour(rng, alpha)
        x1 = x0 + rng.randint(-40, 40)
        y1 = y0 + length
        ddraw.line([(x0, y0), (x1, y1)], fill=colour, width=width_px)


def _add_scatter(ddraw, rng, w, h, count_range, alpha_range):
    """Fine particle scatter."""
    for _ in range(rng.randint(*count_range)):
        x0     = rng.randint(0, w)
        y0     = rng.randint(0, h)
        r      = rng.randint(1, 7)
        alpha  = rng.randint(*alpha_range)
        colour = _dust_colour(rng, alpha)
        ddraw.ellipse([x0 - r, y0 - r, x0 + r, y0 + r], fill=colour)


def _add_edge_buildup(ddraw, rng, w, h, alpha_range):
    """
    Dust accumulates along cell grid edges and the outer frame.
    Mirrors real UAE panel behaviour: dust traps along any raised line.
    """
    cols, rows = 6, 12
    for i in range(cols + 1):
        x = int(i * w / cols)
        for _ in range(rng.randint(3, 8)):
            y0       = rng.randint(0, h)
            length   = rng.randint(20, 80)
            width_px = rng.randint(4, 14)
            alpha    = rng.randint(*alpha_range)
            colour   = _dust_colour(rng, alpha)
            ddraw.line([(x, y0), (x, y0 + length)], fill=colour, width=width_px)
    for j in range(rows + 1):
        y = int(j * h / rows)
        for _ in range(rng.randint(2, 6)):
            x0       = rng.randint(0, w)
            length   = rng.randint(20, 80)
            width_px = rng.randint(3, 10)
            alpha    = rng.randint(*alpha_range)
            colour   = _dust_colour(rng, alpha)
            ddraw.line([(x0, y), (x0 + length, y)], fill=colour, width=width_px)


# ── Dust layer builder ─────────────────────────────────────────────

def _build_dust_layer(rng, w, h, density):
    dust  = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    ddraw = ImageDraw.Draw(dust)

    if density == "clean":
        # No dust — return transparent layer
        return dust

    elif density == "light":
        _add_blobs(ddraw, rng, w, h, (3, 7),    (18, 50))
        _add_streaks(ddraw, rng, w, h, (2, 5),  (12, 40))
        _add_scatter(ddraw, rng, w, h, (100, 250), (18, 65))
        _add_edge_buildup(ddraw, rng, w, h,       (12, 38))
        blur_radius = 2

    elif density == "medium":
        _add_blobs(ddraw, rng, w, h, (6, 12),   (32, 75))
        _add_streaks(ddraw, rng, w, h, (4, 8),  (22, 60))
        _add_scatter(ddraw, rng, w, h, (250, 500), (28, 85))
        _add_edge_buildup(ddraw, rng, w, h,        (22, 60))
        blur_radius = 3

    elif density == "heavy":
        _add_blobs(ddraw, rng, w, h, (12, 20),  (52, 125))
        _add_streaks(ddraw, rng, w, h, (6, 14), (38, 88))
        _add_scatter(ddraw, rng, w, h, (500, 900), (38, 125))
        _add_edge_buildup(ddraw, rng, w, h,         (38, 88))
        blur_radius = 4

    elif density == "streaked":
        _add_blobs(ddraw, rng, w, h, (3, 7),    (22, 58))
        _add_streaks(ddraw, rng, w, h, (12, 22), (32, 90))
        _add_scatter(ddraw, rng, w, h, (150, 350), (22, 78))
        _add_edge_buildup(ddraw, rng, w, h,        (18, 52))
        blur_radius = 3

    elif density == "patchy":
        for _ in range(rng.randint(2, 4)):
            rx = rng.randint(0, w // 2)
            ry = rng.randint(0, h // 2)
            for _ in range(rng.randint(4, 9)):
                cx    = rx + rng.randint(0, w // 2)
                cy    = ry + rng.randint(0, h // 2)
                ex    = rng.randint(40, 150)
                ey    = rng.randint(30, 100)
                alpha = rng.randint(45, 115)
                colour = _dust_colour(rng, alpha)
                pts = []
                for t in range(0, 360, 10):
                    rad = math.radians(t)
                    pts.append((cx + ex * math.cos(rad), cy + ey * math.sin(rad)))
                ddraw.polygon(pts, fill=colour)
        _add_scatter(ddraw, rng, w, h, (200, 450), (22, 82))
        _add_edge_buildup(ddraw, rng, w, h,        (18, 58))
        blur_radius = 3

    else:
        raise ValueError(f"Unknown dust density: {density!r}")

    return dust.filter(ImageFilter.GaussianBlur(radius=blur_radius))


# ── Public API ─────────────────────────────────────────────────────

def generate_panel_texture(panel_index, density, seed, output_dir):
    """
    Generate one panel texture and save as PNG.
    Returns the filename.
    """
    rng  = random.Random(seed)
    base = Image.new("RGB", (W, H), (32, 58, 105))
    draw = ImageDraw.Draw(base)
    draw_base_panel(draw, W, H)
    dust   = _build_dust_layer(rng, W, H, density)
    result = Image.alpha_composite(base.convert("RGBA"), dust).convert("RGB")
    filename = f"solar_panel_{panel_index:02d}_dust.png"
    result.save(os.path.join(output_dir, filename))
    return filename


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Output → {os.path.abspath(OUTPUT_DIR)}\n")

    # Panel 00: clean reference texture (no dust)
    fname = generate_panel_texture(0, "clean", BASE_SEED, OUTPUT_DIR)
    print(f"  panel_00  density=clean       saved: {fname}")

    # Panels 01-10: dusty textures (original cycle)
    for i in range(1, len(DENSITY_CYCLE) + 1):
        density = DENSITY_CYCLE[i - 1]
        seed    = BASE_SEED + i * 137
        fname   = generate_panel_texture(i, density, seed, OUTPUT_DIR)
        print(f"  panel_{i:02d}  density={density:10s} saved: {fname}")

    print(f"\nDone — {len(DENSITY_CYCLE) + 1} textures written.")


if __name__ == "__main__":
    main()
