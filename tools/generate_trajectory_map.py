#!/usr/bin/env python3
"""Generate a robust shot map for Eclipse's calibrated hub shooter.

The map is indexed by distance and robot radial velocity.  For every cell the
script searches launch angle and ball speed candidates, rejects trajectories
that do not descend through the hub opening, and selects the candidate with
the largest tolerance to speed, angle, and velocity error.

The output is intentionally CSV so it can be inspected in a spreadsheet,
plotted, or converted into a Java lookup structure after field calibration.
This is an offline tool; it is not imported by robot code.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


G = 9.80665


@dataclass(frozen=True)
class Candidate:
    angle_deg: float
    ball_speed_mps: float
    tof_s: float
    margin_m: float


def values(start: float, stop: float, step: float) -> list[float]:
    count = int(round((stop - start) / step))
    return [start + i * step for i in range(count + 1)]


def impact_height(
    distance_m: float,
    angle_deg: float,
    ball_speed_mps: float,
    radial_velocity_mps: float,
    release_height_m: float,
) -> tuple[float, float] | None:
    """Return (height at the target plane, time of flight), or None if invalid."""
    angle = math.radians(angle_deg)
    ground_speed = ball_speed_mps * math.cos(angle) + radial_velocity_mps
    if ground_speed <= 0.05:
        return None
    tof = distance_m / ground_speed
    height = release_height_m + ball_speed_mps * math.sin(angle) * tof - 0.5 * G * tof * tof
    vertical_speed = ball_speed_mps * math.sin(angle) - G * tof
    if vertical_speed >= 0:
        return None  # The ball is still rising at the hub; it cannot score cleanly.
    return height, tof


def robust_candidate(
    distance_m: float,
    radial_velocity_mps: float,
    *,
    release_height_m: float,
    hub_height_m: float,
    opening_height_m: float,
    angle_error_deg: float,
    speed_error_fraction: float,
    radial_velocity_error_mps: float,
    angle_step_deg: float,
    speed_step_mps: float,
    min_angle_deg: float,
    max_angle_deg: float,
    min_speed_mps: float,
    max_speed_mps: float,
) -> Candidate | None:
    lower = hub_height_m - opening_height_m
    upper = hub_height_m
    best: Candidate | None = None

    for angle_deg in values(min_angle_deg, max_angle_deg, angle_step_deg):
        for speed in values(min_speed_mps, max_speed_mps, speed_step_mps):
            nominal = impact_height(distance_m, angle_deg, speed, radial_velocity_mps, release_height_m)
            if nominal is None:
                continue

            # Robustness is the smallest distance to either opening boundary over
            # all modeled error corners. A negative margin means this candidate
            # can miss under the specified uncertainty.
            margin = float("inf")
            for da in (-angle_error_deg, angle_error_deg):
                for ds in (-speed_error_fraction, speed_error_fraction):
                    for dv in (-radial_velocity_error_mps, radial_velocity_error_mps):
                        result = impact_height(
                            distance_m,
                            angle_deg + da,
                            speed * (1.0 + ds),
                            radial_velocity_mps + dv,
                            release_height_m,
                        )
                        if result is None:
                            # A non-descending or non-forward perturbed shot is
                            # outside the modeled robust envelope. Use a finite
                            # penalty so CSV consumers can still sort and plot it.
                            margin = min(margin, -opening_height_m)
                            continue
                        height, _ = result
                        margin = min(margin, height - lower, upper - height)

            candidate = Candidate(angle_deg, speed, nominal[1], margin)
            # Keep negative-margin candidates in the output. They are still the
            # best available solution for this cell, and the negative margin
            # tells the team that the requested uncertainty budget cannot be
            # met with the current angle/speed limits.
            if best is None or (candidate.margin_m, -candidate.ball_speed_mps) > (
                best.margin_m,
                -best.ball_speed_mps,
            ):
                best = candidate
    return best


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-o", "--output", type=Path, default=Path("trajectory_map.csv"))
    parser.add_argument("--min-distance", type=float, default=1.5)
    parser.add_argument("--max-distance", type=float, default=5.0)
    parser.add_argument("--distance-step", type=float, default=0.25)
    parser.add_argument("--min-radial-velocity", type=float, default=-3.0,
                        help="m/s; positive means the robot is moving toward the hub")
    parser.add_argument("--max-radial-velocity", type=float, default=3.0)
    parser.add_argument("--radial-velocity-step", type=float, default=0.5)
    parser.add_argument("--release-height", type=float, default=0.305)
    parser.add_argument("--hub-height", type=float, default=1.829)
    parser.add_argument("--opening-height", type=float, default=0.30)
    parser.add_argument("--angle-error", type=float, default=1.0)
    parser.add_argument("--speed-error-fraction", type=float, default=0.05)
    parser.add_argument("--radial-velocity-error", type=float, default=0.25)
    parser.add_argument("--min-angle", type=float, default=30.0)
    parser.add_argument("--max-angle", type=float, default=60.0)
    parser.add_argument("--angle-step", type=float, default=0.5)
    parser.add_argument("--min-speed", type=float, default=8.0)
    parser.add_argument("--max-speed", type=float, default=20.0)
    parser.add_argument("--speed-step", type=float, default=0.1)
    parser.add_argument("--wheel-efficiency", type=float, default=0.75)
    parser.add_argument("--wheel-speed-scale", type=float, default=1.30,
                        help="field calibration multiplier applied to computed wheel RPS")
    parser.add_argument("--pivot-offset", type=float, default=0.018,
                        help="field calibration offset in pivot output rotations")
    parser.add_argument("--wheel-diameter", type=float, default=0.1016,
                        help="m; Eclipse's current 4-inch flywheel")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    circumference = math.pi * args.wheel_diameter

    rows: list[dict[str, float | str]] = []
    for distance in values(args.min_distance, args.max_distance, args.distance_step):
        for radial_velocity in values(
            args.min_radial_velocity, args.max_radial_velocity, args.radial_velocity_step
        ):
            result = robust_candidate(
                distance,
                radial_velocity,
                release_height_m=args.release_height,
                hub_height_m=args.hub_height,
                opening_height_m=args.opening_height,
                angle_error_deg=args.angle_error,
                speed_error_fraction=args.speed_error_fraction,
                radial_velocity_error_mps=args.radial_velocity_error,
                angle_step_deg=args.angle_step,
                speed_step_mps=args.speed_step,
                min_angle_deg=args.min_angle,
                max_angle_deg=args.max_angle,
                min_speed_mps=args.min_speed,
                max_speed_mps=args.max_speed,
            )
            if result is None:
                continue
            rows.append({
                "distance_m": distance,
                "radial_velocity_mps": radial_velocity,
                "launch_angle_deg": result.angle_deg,
                # Eclipse's calibrated mechanism maps launch angle to pivot
                # output rotations: 70 degrees at stow, flatter as pivot rises.
                "pivot_output_rotations": (70.0 - result.angle_deg) / 360.0 + args.pivot_offset,
                "ball_speed_mps": result.ball_speed_mps,
                "wheel_rps": args.wheel_speed_scale * result.ball_speed_mps
                    / (args.wheel_efficiency * circumference),
                "tof_s": result.tof_s,
                "robust_margin_m": result.margin_m,
            })

    with args.output.open("w", newline="", encoding="utf-8") as stream:
        fieldnames = list(rows[0]) if rows else [
            "distance_m", "radial_velocity_mps", "launch_angle_deg",
            "pivot_output_rotations", "ball_speed_mps", "wheel_rps", "tof_s",
            "robust_margin_m",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"wrote {len(rows)} trajectory cells to {args.output}")


if __name__ == "__main__":
    main()
