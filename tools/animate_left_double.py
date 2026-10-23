"""Animate the exported nominal left-double command run. Requires Pillow.

Run Gradle tests first. This depicts the delayed planar model, not real hardware
or fuel physics. Neither robot behavior nor deployment files are modified.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT / "build" / "reports" / "auto-paths"
WIDTH, HEIGHT = 960, 1000
SCALE, OX, OY, XMIN, XMAX = 85, 100, 170, 1.8, 9.5
FIELD_Y = 8.07
COLORS = ["#087e9a", "#cf641f", "#8255a6", "#16835a"]
INK, MUTED, BG = "#1d3042", "#516274", "#f4f6f8"
GREEN, OFF, SHOT = "#20a36d", "#82909a", "#df9e26"
LEGS = ["DOUBLE_START_TO_FUEL", "COLLECT1_TO_SCORE", "SCORE_TO_FUEL2", "COLLECT2_TO_SCORE"]
FONT_CACHE = {}


def font(size):
    if size not in FONT_CACHE:
        for name in ("C:/Windows/Fonts/segoeui.ttf", "DejaVuSans.ttf"):
            try:
                FONT_CACHE[size] = ImageFont.truetype(name, size)
                break
            except OSError:
                pass
        else:
            FONT_CACHE[size] = ImageFont.load_default()
    return FONT_CACHE[size]


def xy(x, y):
    return (OX + (float(x) - XMIN) * SCALE, OY + (FIELD_Y - float(y)) * SCALE)


def load_trace():
    source = REPORTS / "left-double-command-trace.csv"
    if not source.exists():
        raise RuntimeError("Run the competition auto tests to export the command trace first.")
    assets = [ROOT / "src/main/deploy/pathplanner/autos/2910_trench_left_double.auto"]
    assets += list((ROOT / "src/main/deploy/pathplanner/paths").glob("ECLIPSE_LEFT_*.path"))
    if any(p.stat().st_mtime > source.stat().st_mtime for p in assets):
        raise RuntimeError("Command trace is older than a deployed asset. Rerun Gradle tests.")
    rows = list(csv.DictReader(source.open(newline="")))
    for row in rows:
        for key, value in row.items():
            row[key] = value == "true" if value in ("true", "false") else float(value)
    assert rows[-1]["stopped"], "Animation requires an exported cleanup/end frame."
    assert rows[-1]["shots"] == 2 and rows[-1]["secondAdmitted"], "Not a completed double run."
    assert 4.95 <= rows[-1]["feed1Seconds"] <= 5.05
    return rows


def phase(row):
    if row["stopped"]:
        return ("AUTO END - outputs stopped", INK)
    if row["shots"] >= 2:
        return ("Final shooting volley", SHOT)
    if row["secondPreparation"] and not row["secondAdmitted"]:
        return ("Stow + align for trip two", MUTED)
    if row["secondAdmitted"]:
        if row["intakeStarts"] < 2:
            return ("Second trench trip - intake OFF", COLORS[2])
        if not row["roller"]:
            return ("Aim + spin up for the final shot", SHOT)
        if row["x"] < 6.5 and row["y"] > 5.1:
            return ("Loaded bump return - trip two", COLORS[3])
        return ("Collect behind the hub + midfield sweep", COLORS[3])
    if row["shots"] == 1:
        return ("First shooting volley - 5-second feed", SHOT)
    if row["intakeStarts"] == 0:
        return ("Fast trench rush - hold 90 degrees", COLORS[0]) if row["speedMps"] > .08 else (
            "Initial shooter stow + hold 90 degrees", MUTED)
    if not row["roller"]:
        return ("Aim + spin up for the first shot", SHOT)
    if row["x"] < 7.1 and row["y"] < 5.7:
        return ("Loaded bump return - trip one", COLORS[1])
    return ("Rear-first starting-fuel pickup", COLORS[0])


def draw_background():
    canvas = Image.new("RGB", (WIDTH, HEIGHT), BG)
    d = ImageDraw.Draw(canvas)
    d.text((38, 20), "Eclipse - left double auto", fill=INK, font=font(30))
    d.text((38, 65), "Blue alliance | PathPlanner coordinates | 1x modeled playback",
           fill=MUTED, font=font(18))

    def rect(x0, y0, x1, y1, color):
        d.rectangle((*xy(x0, y1), *xy(x1, y0)), fill=color)

    rect(XMIN, 0, XMAX, FIELD_Y, "#ffffff")
    rect(7.36, 1.72, 9.20, 6.35, "#fff3cf")
    rect(5.55, 3.05, 7.11, 5.01, "#eee7df")
    rect(4.02, 1.67, 5.24, 3.43, "#ecd4b4")
    rect(4.02, 4.64, 5.24, 6.40, "#ecd4b4")
    rect(4.02, 3.43, 5.24, 4.64, "#61788d")
    rect(4.02, 1.278, 5.24, 1.67, "#c85656")
    rect(4.02, 6.40, 5.24, FIELD_Y - 1.278, "#c85656")
    for x in range(2, 10):
        d.line((xy(x, 0), xy(x, FIELD_Y)), fill="#e6eaee", width=1)
        px, py = xy(x, 0)
        d.text((px - 5, py + 6), str(x), fill=MUTED, font=font(16))
    for y in range(9):
        if y <= FIELD_Y:
            d.line((xy(XMIN, y), xy(XMAX, y)), fill="#e6eaee", width=1)
            px, py = xy(XMIN, y)
            d.text((px - 24, py - 10), str(y), fill=MUTED, font=font(16))
    d.rectangle((*xy(XMIN, FIELD_Y), *xy(XMAX, 0)), outline="#a6b2be", width=2)
    for y in range(int(OY), int(xy(XMIN, 0)[1]), 17):
        d.line((xy(8.27, 0)[0], y, xy(8.27, 0)[0], y + 9), fill="#ba9b57", width=2)
    d.text((xy(8.27, 0)[0] - 42, OY - 25), "Centerline", fill=MUTED, font=font(16))
    d.text((OX - 35, OY - 25), "Y (m)", fill=MUTED, font=font(16))
    d.text((xy(XMAX, 0)[0] - 35, xy(XMAX, 0)[1] + 26), "X (m)", fill=MUTED, font=font(16))
    d.text((xy(4.63, 4.03)[0] - 19, xy(4.63, 4.03)[1] - 12), "HUB",
           fill="#ffffff", font=font(18))
    d.text((xy(4.63, 6.02)[0] - 22, xy(4.63, 6.02)[1] - 10), "Bump",
           fill="#785839", font=font(16))
    d.text((xy(4.63, 2.30)[0] - 22, xy(4.63, 2.30)[1] - 10), "Bump",
           fill="#785839", font=font(16))
    d.text((xy(5.72, 4.1)[0], xy(5.72, 4.1)[1]), "Hub pocket",
           fill="#867263", font=font(15))
    planned = list(csv.DictReader((REPORTS / "samples.csv").open(newline="")))
    for leg in LEGS:
        pts = [xy(r["x"], r["y"]) for r in planned
               if r["alliance"] == "Blue" and r["path"] == "ECLIPSE_LEFT_" + leg]
        assert pts, "Missing deployed route samples: " + leg
        d.line(pts, fill="#c6cfd7", width=3)
    x, y = xy(3, 5.55)
    d.rectangle((x - 5, y - 5, x + 5, y + 5), fill=INK)
    d.text((x - 64, y + 15), "SCORE", fill=MUTED, font=font(16))
    x = 800
    d.text((x, 155), "ROUTE", fill=MUTED, font=font(17))
    labels = [("1  Rush +", "    collect fuel"), ("2  Loaded", "    bump return"),
              ("3  Trench", "    trip two"), ("4  Hub pocket", "    + return")]
    for i, lines in enumerate(labels):
        yy = 192 + i * 74
        d.line((x, yy - 7, x + 114, yy - 7), fill=COLORS[i], width=4)
        d.text((x, yy), lines[0], fill=INK, font=font(17))
        d.text((x, yy + 23), lines[1], fill=INK, font=font(17))
    d.text((x, 515), "ROBOT", fill=MUTED, font=font(17))
    d.polygon(((x + 9, 550), (x + 3, 561), (x + 15, 561)), fill=INK)
    d.text((x + 25, 545), "Front arrow", fill=INK, font=font(16))
    d.rectangle((x, 580, x + 17, 588), fill=GREEN)
    d.text((x + 25, 573), "Rear intake", fill=INK, font=font(16))
    d.text((x, 607), "Green = ON", fill=GREEN, font=font(17))
    d.text((x, 634), "Gray = OFF", fill=MUTED, font=font(17))
    d.text((38, 975), "Headless drive model; mechanism delays assumed. Not hardware footage or fuel physics.",
           fill=MUTED, font=font(17))
    return canvas


def draw_robot(d, row):
    c, s = math.cos(math.radians(row["headingDeg"])), math.sin(math.radians(row["headingDeg"]))

    def local(x, y):
        return xy(row["x"] + x * c - y * s, row["y"] + x * s + y * c)

    d.polygon([local(x, y) for x, y in [(-.45, -.45), (.45, -.45), (.45, .45), (-.45, .45)]],
              fill="#245a89", outline=INK, width=2)
    for x in (-.28, .28):
        for y in (-.28, .28):
            points = [local(x + a, y + b) for a, b in [(-.07, -.07), (.07, -.07), (.07, .07), (-.07, .07)]]
            d.polygon(points, fill="#bdc9d2")
    d.polygon([local(.35, 0), local(.12, -.13), local(.12, .13)], fill="#ffffff")
    d.polygon([local(x, y) for x, y in [(-.6786, -.4), (-.45, -.4), (-.45, .4), (-.6786, .4)]],
              fill=GREEN if row["roller"] else OFF, outline=INK, width=2)
    for y in (-.26, 0, .26):
        d.line((local(-.66, y), local(-.48, y)), fill="#e8f4ee", width=2)


def render(row, index, trace, background, duration):
    frame = background.copy()
    d = ImageDraw.Draw(frame)
    title, color = phase(row)
    d.rounded_rectangle((38, 100, 922, 134), radius=6, fill="#e2e8ed")
    d.rectangle((38, 100, 44, 134), fill=color)
    d.text((54, 102), title, fill=INK, font=font(21))
    d.text((775, 27), f"{row['timeSeconds']:04.1f} / {duration:04.1f} s", fill=INK, font=font(21))
    d.text((800, 700), f"{row['speedMps']:.2f} m/s", fill=INK, font=font(21))
    d.text((800, 730), f"Yaw {row['headingDeg']:.0f} deg", fill=MUTED, font=font(17))
    # The trail comes from executed model poses, not a constant-speed path replay.
    previous = trace[0]
    for past in trace[2:index + 1:2]:
        _, trail_color = phase(past)
        if math.hypot(past["x"] - previous["x"], past["y"] - previous["y"]) > .003:
            d.line((xy(previous["x"], previous["y"]), xy(past["x"], past["y"])),
                   fill=trail_color, width=4)
        previous = past
    if "spin up" in title or "shooting volley" in title:
        a, b = xy(row["x"], row["y"]), xy(4.625, 4.035)
        for k in range(0, 20, 2):
            p = (a[0] + (b[0] - a[0]) * k / 20, a[1] + (b[1] - a[1]) * k / 20)
            q = (a[0] + (b[0] - a[0]) * (k + 1) / 20, a[1] + (b[1] - a[1]) * (k + 1) / 20)
            d.line((p, q), fill=SHOT, width=3)
    draw_robot(d, row)
    d.text((38, 907), f"First feed: {row['feed1Seconds']:.2f} / 5.00 s", fill=INK, font=font(19))
    d.text((490, 907), f"Final feed: {row['feed2Seconds']:.2f} s", fill=INK, font=font(19))
    d.rounded_rectangle((38, 944, 922, 955), radius=4, fill="#dce3e8")
    width = 884 * min(1, row["timeSeconds"] / duration)
    if width > 1:
        d.rounded_rectangle((38, 944, 38 + width, 955), radius=4, fill=color)
    return frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=REPORTS / "eclipse-left-double-auto.gif")
    args = parser.parse_args()
    trace = load_trace()
    times = [r["timeSeconds"] for r in trace]
    duration = times[-1]
    background = draw_background()
    # Fixed global palette keeps the field/background from changing colors by frame.
    rgb_colors = [(r, g, b) for r in range(0, 256, 51)
                  for g in range(0, 256, 51) for b in range(0, 256, 51)]
    exact = [INK, MUTED, BG, GREEN, OFF, SHOT] + COLORS + [
        "#ffffff", "#e6eaee", "#245a89", "#c6cfd7", "#61788d", "#ecd4b4",
        "#c85656", "#fff3cf", "#eee7df", "#e2e8ed", "#bdc9d2", "#dce3e8"]
    rgb_colors += [tuple(bytes.fromhex(c[1:])) for c in exact]
    rgb_colors += [(i, i, i) for i in (30, 60, 90, 120, 150, 180, 210, 240)]
    palette = Image.new("P", (1, 1))
    entries = [v for color in rgb_colors[:256] for v in color]
    palette.putpalette(entries + [0] * (768 - len(entries)))
    frame_times = [i * .08 for i in range(math.ceil(duration / .08))] + [duration]
    frames, delays = [], []
    for i, t in enumerate(frame_times):
        index = bisect.bisect_right(times, t + 1e-7) - 1
        row = trace[max(0, index)]
        frame = render(row, index, trace, background, duration)
        frames.append(frame.quantize(palette=palette, dither=Image.Dither.NONE))
        delays.append(max(10, round((frame_times[i + 1] - t) * 100) * 10)
                      if i + 1 < len(frame_times) else 1200)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.output, save_all=True, append_images=frames[1:], duration=delays,
                   loop=0, optimize=False, disposal=1,
                   comment=b"Executed headless left-double model; 1x; not hardware/fuel physics.")
    tile_width, tile_height = WIDTH // 3, round(HEIGHT / 3)
    previews = Image.new("RGB", (WIDTH, tile_height * 2), BG)
    for tile, t in enumerate((0, 1.8, 6.5, 10.8, 14.4, 18.5)):
        index = max(0, bisect.bisect_right(times, t) - 1)
        shot = render(trace[index], index, trace, background, duration)
        shot.thumbnail((tile_width, tile_height))
        previews.paste(shot, ((tile % 3) * tile_width, (tile // 3) * tile_height))
    previews.save(args.output.with_name(args.output.stem + "-contact-sheet.png"))
    mid = max(0, bisect.bisect_right(times, 14.4) - 1)
    render(trace[mid], mid, trace, background, duration).save(
        args.output.with_name(args.output.stem + "-preview.png"))
    with Image.open(args.output) as check:
        assert check.n_frames >= 200 and check.size == (WIDTH, HEIGHT)
        gif_ms = 0
        for i in range(check.n_frames):
            check.seek(i)
            gif_ms += check.info["duration"]
    print(json.dumps({"gif": str(args.output), "frames": len(frames),
                      "modelSeconds": duration, "gifSecondsIncludingEndHold": gif_ms / 1000,
                      "bytes": args.output.stat().st_size,
                      "firstFeedSeconds": trace[-1]["feed1Seconds"],
                      "finalFeedSeconds": trace[-1]["feed2Seconds"]}, indent=2))


if __name__ == "__main__":
    main()
