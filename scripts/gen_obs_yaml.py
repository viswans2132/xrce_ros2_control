#!/usr/bin/env python3
"""
Generate a YAML file with N random vertical cylindrical obstacles.

Constraints:
1) radius r ~ Uniform[0.1, 1.0] meters
2) No cylinder may touch the origin: closest distance from origin to cylinder surface >= 1.0 m
   => center distance from origin >= (r + 1.0)
3) Minimum clearance between any two cylinders' surfaces >= 1.0 m
   => center distance between i,j >= (r_i + r_j + 1.0)

Obstacles are "mostly 2D": cylinders are vertical and specified by (x,y,r,z_min,z_max).
"""

import argparse
import math
import random
import yaml
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class Cylinder:
    x: float
    y: float
    r: float
    z_min: float
    z_max: float


def _dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def generate_obstacles(
    n: int,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
    r_min: float = 0.5,
    r_max: float = 1.0,
    origin_clearance: float = 1.0,
    inter_clearance: float = 1.0,
    max_tries: int = 200000,
    seed: Optional[int] = None,
) -> List[Cylinder]:
    if seed is not None:
        random.seed(seed)

    if x_max <= x_min or y_max <= y_min:
        raise ValueError("Invalid bounds: require x_max>x_min and y_max>y_min")

    obs: List[Cylinder] = []
    tries = 0

    while len(obs) < n and tries < max_tries:
        tries += 1

        r = random.uniform(r_min, r_max)
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)

        # (2) keep cylinder surface at least origin_clearance away from origin
        # => center distance >= r + origin_clearance
        if (x * x + y * y) < (r + origin_clearance) ** 2:
            continue

        # (3) keep cylinder surfaces at least inter_clearance apart
        ok = True
        for o in obs:
            min_center_dist = o.r + r + inter_clearance
            if _dist2((x, y), (o.x, o.y)) < (min_center_dist ** 2):
                ok = False
                break
        if not ok:
            continue

        obs.append(Cylinder(x=x, y=y, r=r, z_min=z_min, z_max=z_max))

    if len(obs) < n:
        raise RuntimeError(
            f"Could only place {len(obs)}/{n} obstacles after {tries} tries. "
            f"Try expanding bounds, reducing N, or reducing clearances."
        )

    return obs


def write_yaml(obstacles: List[Cylinder], inflate_radius: float, out_path: str):
    data = {
        "obstacles": [
            {
                "type": "cylinder",
                "x": float(o.x),
                "y": float(o.y),
                "radius": float(o.r),
                "z_min": float(o.z_min),
                "z_max": float(o.z_max),
            }
            for o in obstacles
        ],
        "inflate_radius": float(inflate_radius),
    }
    with open(out_path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--num", type=int, required=True, help="Number of obstacles N")
    ap.add_argument("--out", type=str, default="obstacles.yaml", help="Output YAML path")

    ap.add_argument("--x_min", type=float, default=-10.0)
    ap.add_argument("--x_max", type=float, default=10.0)
    ap.add_argument("--y_min", type=float, default=-10.0)
    ap.add_argument("--y_max", type=float, default=10.0)

    ap.add_argument("--z_min", type=float, default=0.0)
    ap.add_argument("--z_max", type=float, default=5.0)

    ap.add_argument("--r_min", type=float, default=0.5)
    ap.add_argument("--r_max", type=float, default=1.0)

    ap.add_argument("--origin_clearance", type=float, default=1.0)
    ap.add_argument("--inter_clearance", type=float, default=1.0)

    ap.add_argument("--inflate_radius", type=float, default=0.15)

    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--max_tries", type=int, default=200000)

    args = ap.parse_args()

    obstacles = generate_obstacles(
        n=args.num,
        x_min=args.x_min,
        x_max=args.x_max,
        y_min=args.y_min,
        y_max=args.y_max,
        z_min=args.z_min,
        z_max=args.z_max,
        r_min=args.r_min,
        r_max=args.r_max,
        origin_clearance=args.origin_clearance,
        inter_clearance=args.inter_clearance,
        max_tries=args.max_tries,
        seed=args.seed,
    )

    write_yaml(obstacles, inflate_radius=args.inflate_radius, out_path=args.out)
    print(f"Wrote {len(obstacles)} obstacles to {args.out}")


if __name__ == "__main__":
    main()