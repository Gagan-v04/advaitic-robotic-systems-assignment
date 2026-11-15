"""Capture perception views: RGB, semantic segmentation masks, depth maps, and localized LiDAR/wrist ray hits."""

from __future__ import annotations

import json
from pathlib import Path
import numpy as np
from PIL import Image

import mujoco
from advaitic_cell.scene import load_cell, base_pose
from advaitic_cell.perception import _ray


def main() -> None:
    out_dir = Path("solution/perception")
    out_dir.mkdir(parents=True, exist_ok=True)

    model, data, index = load_cell()
    mujoco.mj_forward(model, data)

    # Step simulation forward to settle contact physics
    for _ in range(500):
        mujoco.mj_step(model, data)

    # 1. Cast 64-ray LiDAR scan around AMR base
    x, y, yaw = base_pose(data, index)
    angles_64 = np.linspace(-np.pi, np.pi, 64, endpoint=False)
    lidar_hits = []
    for angle in angles_64:
        direction = np.array([np.cos(yaw + angle), np.sin(yaw + angle), 0.0])
        dist, gid = _ray(model, data, (x, y, 0.6), direction, (0, 1))
        hit_xyz = (np.array([x, y, 0.6]) + dist * direction) if dist >= 0 else None
        lidar_hits.append({
            "angle_deg": round(float(np.degrees(angle)), 1),
            "distance_m": round(float(dist), 3) if dist >= 0 else None,
            "geom_id": int(gid),
            "hit_point": [round(float(c), 3) for c in hit_xyz] if hit_xyz is not None else None,
        })

    # 2. Cast 5x5 Wrist Depth Sensor Rays directly over Kit 0 bottle rim
    bottle_pos = data.xpos[index.slots[0].bottle_body].copy()
    target_bottle = np.array([bottle_pos[0], bottle_pos[1], 0.785])
    wrist_hits = []
    contact_points = []
    for dx in (-0.03, -0.015, 0.0, 0.015, 0.03):
        for dy in (-0.03, -0.015, 0.0, 0.015, 0.03):
            origin = target_bottle + np.array([dx, dy, 0.12])
            dist, gid = _ray(model, data, origin, (0.0, 0.0, -1.0), (0, 1, 2, 3, 4, 5))
            hit_xyz = (origin + dist * np.array([0.0, 0.0, -1.0])) if dist >= 0 else None
            if dist >= 0 and hit_xyz is not None:
                contact_points.append(hit_xyz)
            wrist_hits.append({
                "origin": [round(float(c), 4) for c in origin],
                "direction": [0.0, 0.0, -1.0],
                "distance_m": round(float(dist), 4) if dist >= 0 else None,
                "geom_id": int(gid),
                "hit_point": [round(float(c), 4) for c in hit_xyz] if hit_xyz is not None else None,
            })

    centroid = np.mean(contact_points, axis=0).tolist() if contact_points else None

    # Save localized points to JSON
    evidence = {
        "amr_pose": {"x": round(float(x), 3), "y": round(float(y), 3), "yaw_rad": round(float(yaw), 3)},
        "lidar_rays_64": lidar_hits,
        "wrist_depth_grid_5x5": wrist_hits,
        "localized_bottle_centroid": [round(c, 4) for c in centroid] if centroid else None,
    }
    with open(out_dir / "localized_points.json", "w") as f:
        json.dump(evidence, f, indent=2)

    # 3. Offscreen Rendering: RGB, Depth, Segmentation Mask
    width, height = 640, 480
    renderer = mujoco.Renderer(model, height=height, width=width)

    cameras = ["station_a", "top_a", "lidar", "overview"]
    for cam in cameras:
        # Standard RGB Frame
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render()
        Image.fromarray(rgb).save(out_dir / f"{cam}_rgb.png")

        # Metric Depth Map
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=cam)
        depth = renderer.render()
        renderer.disable_depth_rendering()
        
        finite_depth = depth[np.isfinite(depth) & (depth > 0)]
        d_min = float(np.percentile(finite_depth, 2)) if len(finite_depth) > 0 else 0.5
        d_max = float(np.percentile(finite_depth, 98)) if len(finite_depth) > 0 else 5.0
        depth_scaled = np.clip((depth - d_min) / max(d_max - d_min, 1e-4), 0, 1)
        depth_img = (255 * (1.0 - depth_scaled)).astype(np.uint8)
        Image.fromarray(depth_img).save(out_dir / f"{cam}_depth.png")

        # Semantic Segmentation Mask
        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera=cam)
        seg = renderer.render()
        renderer.disable_segmentation_rendering()
        
        geom_ids = seg[:, :, 0]
        mask_rgb = np.zeros((height, width, 3), dtype=np.uint8)
        unique_ids = np.unique(geom_ids)
        for gid in unique_ids:
            if gid == -1:
                mask_rgb[geom_ids == gid] = [25, 25, 30]
            else:
                color = [(gid * 67 + 35) % 256, (gid * 131 + 80) % 256, (gid * 197 + 120) % 256]
                mask_rgb[geom_ids == gid] = color
        Image.fromarray(mask_rgb).save(out_dir / f"{cam}_segmentation.png")

    print(f"Perception views and localized points successfully written to {out_dir}")


if __name__ == "__main__":
    main()
