#!/usr/bin/env python3
"""
Export RGB-D Dataset from Isaac Sim

Captures and saves:
- RGB images (PNG)
- Depth maps (NPY format)
- Camera intrinsics (JSON)

Usage:
  1. Launch scene with create_indoor_scene.py
  2. Run: python export_rgbd_data.py --frames 100 --output dataset/
"""

import argparse
import os
import json
import numpy as np
from PIL import Image
from omni.isaac.sensor import Camera

def export_rgbd_dataset(camera_path, output_dir, num_frames):
    """Export RGB-D frames from Isaac Sim camera."""

    # Create output directories
    rgb_dir = os.path.join(output_dir, "rgb")
    depth_dir = os.path.join(output_dir, "depth")
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    # Get camera
    camera = Camera(camera_path)
    camera.initialize()
    camera.add_depth_data_to_frame()

    # Export camera intrinsics
    intrinsics = {
        "width": camera.get_resolution()[0],
        "height": camera.get_resolution()[1],
        "fx": camera.get_focal_length(),
        "fy": camera.get_focal_length(),
        "cx": camera.get_resolution()[0] / 2,
        "cy": camera.get_resolution()[1] / 2
    }

    with open(os.path.join(output_dir, "camera_intrinsics.json"), "w") as f:
        json.dump(intrinsics, f, indent=2)

    print(f"Exporting {num_frames} frames to {output_dir}")

    # Capture frames
    for i in range(num_frames):
        # Get RGB image
        frame = camera.get_rgba()
        rgb = frame[:, :, :3]  # Drop alpha channel

        # Save RGB
        rgb_img = Image.fromarray(rgb.astype(np.uint8))
        rgb_img.save(os.path.join(rgb_dir, f"{i:06d}.png"))

        # Get depth
        depth = camera.get_depth()

        # Save depth (in meters)
        np.save(os.path.join(depth_dir, f"{i:06d}.npy"), depth)

        if i % 10 == 0:
            print(f"  Frame {i}/{num_frames}")

        # Step simulation
        from omni.isaac.core import World
        World.instance().step(render=True)

    print(f"Dataset export complete!")
    print(f"  RGB: {rgb_dir}")
    print(f"  Depth: {depth_dir}")
    print(f"  Intrinsics: {output_dir}/camera_intrinsics.json")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="/World/Camera", help="Camera prim path")
    parser.add_argument("--frames", type=int, default=100, help="Number of frames")
    parser.add_argument("--output", default="dataset/", help="Output directory")
    args = parser.parse_args()

    export_rgbd_dataset(args.camera, args.output, args.frames)


if __name__ == "__main__":
    main()
