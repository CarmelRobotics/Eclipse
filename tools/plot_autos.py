"""Plot generated PathPlanner double-auto samples and geometric rear-intake coverage.

Run Gradle tests first. Requires Pillow. This is a field diagram, not a physics sim.
"""
import csv
import math
import sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT / "build" / "reports" / "auto-paths"
SINGLE = "--single" in sys.argv
W, H = 1800, 1990
image = Image.new("RGB", (W, H), "#f4f6f8")
draw = ImageDraw.Draw(image)


def font(size):
    for name in ("C:/Windows/Fonts/segoeui.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


draw.text((55, 22), "Eclipse singles: 90-degree start > rear-first sweep > hub pocket > score" if SINGLE
          else "Eclipse doubles: 90-degree start > pickup > 5 s dump > fresh pocket > score",
          fill="#142538", font=font(31))
draw.text((55, 70), "PathPlanner coordinates | outbound uses trench | rear intake leads pickup | loaded returns use bump",
          fill="#4b5e70", font=font(22))
rows = list(csv.DictReader((REPORTS / "samples.csv").open(newline="")))
cells = list(csv.DictReader((REPORTS / "pickup-cells.csv").open(newline="")))
colors = ("#087e9a", "#d65f18", "#7c49a8", "#188653")
legs = ("START_TO_FUEL", "SINGLE_COLLECT_TO_SCORE") if SINGLE else (
    "DOUBLE_START_TO_FUEL", "COLLECT1_TO_SCORE", "SCORE_TO_FUEL2", "COLLECT2_TO_SCORE")
labels = ("1 Sideways trench rush + long sweep", "2 Hub-pocket pickup + loaded bump return") if SINGLE else (
    "1 Sideways rush + broad pickup", "2 Straight bump return + 5 s dump",
    "3 Compact trench re-entry (roller off)", "4 Fresh pocket + C sweep + loaded bump return")

for panel, alliance in enumerate(("Blue", "Red")):
    ox, oy, scale = 90, 180 + panel * 815, 92

    def xy(x, y):
        return (ox + float(x) * scale, oy + (8.07 - float(y)) * scale)

    def rect(x1, y1, x2, y2, fill):
        a, b = xy(x1, y2), xy(x2, y1)
        draw.rectangle((*a, *b), fill=fill)

    draw.text((ox, oy - 45), alliance + " alliance (global blue-origin axes)",
              fill="#22384b", font=font(23))
    rect(0, 0, 16.54, 8.07, "#ffffff")
    rect(7.36, 1.72, 9.18, 6.35, "#fff4c7")
    # Team-observed accumulation pocket, not an assumed initial fuel placement.
    if alliance == "Blue":
        rect(5.55, 3.05, 7.11, 5.01, "#f2e9e2")
    else:
        rect(16.54 - 7.11, 8.07 - 5.01, 16.54 - 5.55, 8.07 - 3.05, "#f2e9e2")
    for x in (4.02, 16.54 - 5.24):
        rect(x, 1.67, x + 1.22, 3.43, "#edd5b4")
        rect(x, 4.64, x + 1.22, 6.40, "#edd5b4")
        rect(x, 3.43, x + 1.22, 4.64, "#63788b")
        rect(x, 1.278, x + 1.22, 1.67, "#c85353")
        rect(x, 6.40, x + 1.22, 8.07 - 1.278, "#c85353")
    for x in range(17):
        draw.line((xy(x, 0), xy(x, 8.07)), fill="#e4e9ed", width=1)
        draw.text((xy(x, 0)[0] - 6, xy(x, 0)[1] + 6), str(x),
                  font=font(15), fill="#516274")
    for y in range(9):
        if y <= 8.07:
            draw.line((xy(0, y), xy(16.54, y)), fill="#e4e9ed", width=1)
            draw.text((ox - 30, xy(0, y)[1] - 10), str(y), font=font(15), fill="#516274")
    draw.line((xy(8.27, 0), xy(8.27, 8.07)), fill="#9d8c4b", width=2)
    for cell in cells:
        if not SINGLE and cell["alliance"] == alliance:
            x, y = xy(cell["x"], cell["y"])
            shade = "#b7e7e9" if cell["cycle"] == "1" else "#d7c5ed"
            draw.rectangle((x - 2, y - 2, x + 2, y + 2), fill=shade)

    for side in ("LEFT", "RIGHT"):
        groups = [[r for r in rows if r["alliance"] == alliance
                   and r["path"] == f"ECLIPSE_{side}_{leg}"] for leg in legs]
        # Width layering keeps coincident outbound/return centerlines visible.
        for i in ((1, 0) if SINGLE else (1, 2, 3, 0)):
            pts, color = groups[i], colors[i]
            draw.line([xy(r["x"], r["y"]) for r in pts], fill=color, width=(5, 11, 7, 3)[i])
            j = int(len(pts) * (0.55 if i == 0 else 0.70 if i == 2 else 0.42))
            a = xy(pts[max(0, j - 3)]["x"], pts[max(0, j - 3)]["y"])
            b = xy(pts[j]["x"], pts[j]["y"])
            angle = math.atan2(b[1] - a[1], b[0] - a[0])
            draw.polygon((b,
                          (b[0] - 16 * math.cos(angle - .5), b[1] - 16 * math.sin(angle - .5)),
                          (b[0] - 16 * math.cos(angle + .5), b[1] - 16 * math.sin(angle + .5))),
                         fill=color)

        # Black whiskers point from the center to the REAR intake, not the front.
        for i, fraction in (((0, .55), (1, .25)) if SINGLE else ((0, .65), (3, .4))):
            r = groups[i][int(len(groups[i]) * fraction)]
            x, y, theta = float(r["x"]), float(r["y"]), math.radians(float(r["headingDeg"]))
            rear = xy(x - .6786 * math.cos(theta), y - .6786 * math.sin(theta))
            draw.line((xy(x, y), rear), fill="#243747", width=3)
            draw.ellipse((rear[0] - 4, rear[1] - 4, rear[0] + 4, rear[1] + 4), fill="#243747")

        start, score = groups[0][0], groups[1][-1]
        # Starting footprint: 0.90 m bumper plus REAR-only 9-inch extension.
        sx, sy, theta = float(start["x"]), float(start["y"]), math.radians(float(start["headingDeg"]))
        footprint = [
            xy(sx + a * math.cos(theta) - b * math.sin(theta),
               sy + a * math.sin(theta) + b * math.cos(theta))
            for a, b in ((-.6786, -.45), (.45, -.45), (.45, .45), (-.6786, .45))
        ]
        draw.line(footprint + [footprint[0]], fill="#243747", width=2)
        rear = xy(sx - .6786 * math.cos(theta), sy - .6786 * math.sin(theta))
        draw.line((xy(sx, sy), rear), fill="#243747", width=4)
        x, y = xy(start["x"], start["y"])
        draw.ellipse((x - 7, y - 7, x + 7, y + 7), fill=colors[0], outline="white", width=2)
        x, y = xy(score["x"], score["y"])
        draw.rectangle((x - 7, y - 7, x + 7, y + 7), fill="#172f40")
        label_x = x - 100 if alliance == "Blue" else x + 15
        draw.text((label_x, y + 10), "SCORE" if SINGLE else "SCORE x2*", fill="#172f40", font=font(16))

for i, (color, label) in enumerate(zip(colors, labels)):
    x, y = 90 + (i % 2) * 770, 1800 + (i // 2) * 38
    draw.line((x, y + 12, x + 45, y + 12), fill=color, width=5)
    draw.text((x + 58, y), label, fill="#233c50", font=font(20))
draw.text((90, 1890), "Black outlines: sideways starting footprint. Whiskers: rear intake. Pickup cells: roller ON only.",
          fill="#586a79", font=font(19))
draw.text((90, 1925), "Axes: meters. Tan: bumps. Red: supports. Dark: hubs. Singles also collect the behind-hub pocket.",
          fill="#586a79", font=font(18))
if not SINGLE:
    draw.text((90, 1960), "*Second trip requires a time check after stow/alignment; late preparation stays at the first score.",
              fill="#586a79", font=font(19))
target = REPORTS / ("eclipse-single-auto-routes.png" if SINGLE else "eclipse-auto-routes.png")
image.save(target)
print(target)
