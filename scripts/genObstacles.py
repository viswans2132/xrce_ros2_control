#!/usr/bin/env python3
import random
import math

# Parameters
num_boxes = 10
num_cylinders = 10
min_distance = 3.5
area_size = 20.0  # world area [-10, 10] x [-10, 10]
box_size = (1, 1, 3)
cyl_radius = 1.0
cyl_height = 3.0

def too_close(p1, p2, min_dist):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1]) < min_dist

def generate_positions(n, min_dist, area):
    positions = []
    while len(positions) < n:
        x = random.uniform(-area/2, area/2)
        y = random.uniform(-area/2, area/2)
        # ensure no overlap and avoid the origin
        if math.hypot(x, y) > 1.5 and all(not too_close((x, y), p, min_dist) for p in positions):
            positions.append((x, y))
    return positions

# Generate random positions
positions = generate_positions(num_boxes + num_cylinders, min_distance, area_size)
box_positions = positions[:num_boxes]
cyl_positions = positions[num_boxes:]

# Build SDF XML
sdf = '<?xml version="1.7"?>\n'
sdf += '<sdf version="1.10">\n'
sdf += '  <model name="random_obstacles">\n'
sdf += '    <static>true</static>\n'

# Boxes with random yaw and random colors
for i, (x, y) in enumerate(box_positions):
    yaw = random.uniform(0, 2 * math.pi)
    r, g, b = random.random(), random.random(), random.random()
    sdf += f'''
    <link name="box_{i}">
      <pose>{x:.2f} {y:.2f} {box_size[2]/2:.2f} 0 0 {yaw:.3f}</pose>
      <collision name="collision">
        <geometry>
          <box><size>{box_size[0]} {box_size[1]} {box_size[2]}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{box_size[0]} {box_size[1]} {box_size[2]}</size></box>
        </geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
        </material>
      </visual>
    </link>
    '''

# Cylinders with random colors
for i, (x, y) in enumerate(cyl_positions):
    r, g, b = random.random(), random.random(), random.random()
    sdf += f'''
    <link name="cyl_{i}">
      <pose>{x:.2f} {y:.2f} {cyl_height/2:.2f} 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{cyl_radius}</radius>
            <length>{cyl_height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{cyl_radius}</radius>
            <length>{cyl_height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
        </material>
      </visual>
    </link>
    '''

sdf += '  </model>\n</sdf>\n'

# Write to file
with open("model.sdf", "w") as f:
    f.write(sdf)

print("✅ Generated model.sdf with randomized boxes (yaw + positions) and cylinders (colors randomized).")
