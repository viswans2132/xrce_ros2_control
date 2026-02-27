#!/usr/bin/env python3
"""
Read a YAML obstacle file (as generated earlier) and produce a Gazebo / SDFormat
world snippet (.sdf) containing vertical cylinder obstacles.

- Each obstacle becomes a static model with a cylinder collision + visual.
- z_min / z_max define the vertical extent. Cylinder height = z_max - z_min.
- Cylinder is placed so its center is at z = (z_min + z_max)/2.

Usage:
  python3 yaml_to_sdf_cylinders.py --yaml obstacles.yaml --out obstacles.sdf

Optional:
  --world_name my_world
  --ground_plane (include a simple ground plane model)
"""

import argparse
import math
import yaml
from typing import Any, Dict, List


def _fmt(x: float) -> str:
    # SDFormat likes compact floats; keep enough precision
    return f"{x:.6f}".rstrip("0").rstrip(".")


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("YAML root must be a mapping/dict.")
    if "obstacles" not in data or not isinstance(data["obstacles"], list):
        raise ValueError("YAML must contain a list under key 'obstacles'.")
    return data


def make_cylinder_model(model_name: str, x: float, y: float, r: float, z_min: float, z_max: float) -> str:
    if z_max <= z_min:
        raise ValueError(f"Invalid z range for {model_name}: z_max must be > z_min")

    height = z_max - z_min
    z_center = 0.5 * (z_min + z_max)

    # pose: x y z roll pitch yaw
    pose = f"{_fmt(x)} {_fmt(y)} {_fmt(z_center)} 0 0 0"

    # cylinder length is height
    model = f"""
    <model name="{model_name}">
      <static>true</static>
      <pose>{pose}</pose>

      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{_fmt(r)}</radius>
              <length>{_fmt(height)}</length>
            </cylinder>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{_fmt(r)}</radius>
              <length>{_fmt(height)}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.7 0.2 0.2 1</ambient>
            <diffuse>0.8 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """.rstrip()
    return model


def make_ground_plane() -> str:
    # A very simple infinite-ish ground plane
    return """
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """.rstrip()


def write_sdf(obstacles: List[Dict[str, Any]], out_path: str, world_name: str, include_ground: bool):
    models = []

    if include_ground:
        models.append(make_ground_plane())

    for i, o in enumerate(obstacles):
        if o.get("type", "cylinder") != "cylinder":
            # ignore unknown obstacles
            continue

        try:
            x = float(o["x"])
            y = float(o["y"])
            r = float(o["radius"])
            z_min = float(o.get("z_min", 0.0))
            z_max = float(o.get("z_max", 5.0))
        except Exception as e:
            raise ValueError(f"Malformed obstacle at index {i}: {o}") from e

        model_name = f"cylinder_{i:03d}"
        models.append(make_cylinder_model(model_name, x, y, r, z_min, z_max))

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="{world_name}">
{chr(10).join(models)}
  </world>
</sdf>
"""
    with open(out_path, "w") as f:
        f.write(sdf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True, help="Input obstacles YAML file")
    ap.add_argument("--out", default="obstacles.sdf", help="Output SDF file path")
    ap.add_argument("--world_name", default="cylinder_world", help="SDF world name")
    ap.add_argument("--ground_plane", action="store_true", help="Include a simple ground plane model")
    args = ap.parse_args()

    data = load_yaml(args.yaml)
    obstacles = data.get("obstacles", [])
    write_sdf(obstacles, args.out, args.world_name, include_ground=args.ground_plane)

    print(f"Wrote SDF with {len(obstacles)} obstacles to {args.out}")


if __name__ == "__main__":
    main()