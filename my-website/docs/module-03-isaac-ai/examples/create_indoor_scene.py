#!/usr/bin/env python3
"""
Create Indoor Scene in Isaac Sim

Generates a photorealistic office scene with:
- Floor, walls, ceiling
- Furniture (desk, chair, shelves)
- RGB-D camera sensor

Usage:
  1. Launch Isaac Sim
  2. Run this script: python create_indoor_scene.py
  3. Scene will appear in viewport
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.sensor import Camera
import numpy as np

# Create world
world = World(stage_units_in_meters=1.0)

# Floor (10x10 meters)
floor = VisualCuboid(
    prim_path="/World/Floor",
    position=np.array([0, 0, -0.05]),
    size=np.array([10, 10, 0.1]),
    color=np.array([0.8, 0.8, 0.8])
)

# Walls
wall_thickness = 0.2
wall_height = 3.0

walls = [
    ("/World/WallNorth", [0, 5, wall_height/2], [10, wall_thickness, wall_height]),
    ("/World/WallSouth", [0, -5, wall_height/2], [10, wall_thickness, wall_height]),
    ("/World/WallEast", [5, 0, wall_height/2], [wall_thickness, 10, wall_height]),
    ("/World/WallWest", [-5, 0, wall_height/2], [wall_thickness, 10, wall_height])
]

for path, pos, size in walls:
    VisualCuboid(prim_path=path, position=np.array(pos), size=np.array(size), color=np.array([0.9, 0.9, 0.85]))

# Ceiling
ceiling = VisualCuboid(
    prim_path="/World/Ceiling",
    position=np.array([0, 0, wall_height + 0.05]),
    size=np.array([10, 10, 0.1]),
    color=np.array([1.0, 1.0, 1.0])
)

# Furniture - Desk
desk = DynamicCuboid(
    prim_path="/World/Desk",
    position=np.array([2, 0, 0.4]),
    size=np.array([1.5, 0.8, 0.8]),
    color=np.array([0.4, 0.3, 0.2])
)

# Chair
chair = DynamicCuboid(
    prim_path="/World/Chair",
    position=np.array([1, 0, 0.25]),
    size=np.array([0.5, 0.5, 0.5]),
    color=np.array([0.2, 0.2, 0.2])
)

# Bookshelf
shelf = VisualCuboid(
    prim_path="/World/Shelf",
    position=np.array([-3, 2, 1.0]),
    size=np.array([0.4, 1.5, 2.0]),
    color=np.array([0.6, 0.4, 0.3])
)

# RGB-D Camera
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([0, -3, 1.5]),
    frequency=30,
    resolution=(640, 480)
)

# Point camera at desk
camera.set_focal_length(24.0)
camera.set_focus_distance(3.0)

# Add lighting
from omni.isaac.core.utils.prims import create_prim
light_path = "/World/DistantLight"
create_prim(light_path, "DistantLight", attributes={"intensity": 3000, "angle": 0.5})

print("Scene created successfully!")
print(f"Camera: {camera.prim_path}")
print("Run simulation to capture images")

# Reset world to apply changes
world.reset()

# Keep simulation running
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
