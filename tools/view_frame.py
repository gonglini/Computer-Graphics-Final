"""Look at one frame of a dataset recorded by LidarDrive.

    python tools/view_frame.py <dataset dir> [frame=0] [out.png]

Left : bird's-eye view of the LiDAR frame (x forward = up), colored by semantic class, with the
       ground-truth 3D boxes.
Right: the front camera image (if it was recorded) or the intensity view.
Needs numpy + matplotlib only.
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

CLASS_COLORS = {
    0: (0.6, 0.6, 0.6), 1: (0.35, 0.35, 0.40), 2: (1.0, 1.0, 1.0), 3: (0.75, 0.60, 0.75),
    4: (0.45, 0.70, 0.30), 5: (0.85, 0.55, 0.25), 6: (0.10, 0.60, 0.15), 7: (0.95, 0.85, 0.20),
    8: (1.00, 0.20, 0.20), 9: (0.20, 0.55, 1.00), 10: (1.00, 0.45, 0.00), 11: (0.80, 0.10, 0.50),
}


def load(root, frame):
    name = f"{frame:06d}"
    pts = np.fromfile(os.path.join(root, "velodyne", name + ".bin"), dtype=np.float32).reshape(-1, 4)
    lab = np.fromfile(os.path.join(root, "labels", name + ".label"), dtype=np.uint32)
    boxes = []
    path = os.path.join(root, "boxes", name + ".txt")
    if os.path.exists(path):
        for line in open(path):
            if line.startswith("#") or not line.strip():
                continue
            t, i, x, y, z, l, w, h, yaw, n = line.split()
            boxes.append((t, int(i), *map(float, (x, y, z, l, w, h, yaw)), int(n)))
    return pts, lab & 0xFFFF, lab >> 16, boxes


def main():
    root = sys.argv[1]
    frame = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(root, f"view_{frame:06d}.png")
    meta = json.load(open(os.path.join(root, "meta.json"), encoding="utf-8"))
    names = {int(k): v for k, v in meta["semantic_classes"].items()}

    pts, sem, inst, boxes = load(root, frame)
    img_path = os.path.join(root, "image_2", f"{frame:06d}.png")
    has_img = os.path.exists(img_path)

    fig, axes = plt.subplots(1, 2, figsize=(16, 8), facecolor="black")
    ax = axes[0]
    ax.set_facecolor("black")
    colors = np.array([CLASS_COLORS.get(int(c), (0.6, 0.6, 0.6)) for c in sem])
    # BEV: forward (x) up, left (y) to the left
    ax.scatter(-pts[:, 1], pts[:, 0], s=0.3, c=colors, linewidths=0)
    for t, i, x, y, z, l, w, h, yaw, n in boxes:
        if n == 0:
            continue
        c, s = np.cos(yaw), np.sin(yaw)
        corners = np.array([[l / 2, w / 2], [l / 2, -w / 2], [-l / 2, -w / 2], [-l / 2, w / 2], [l / 2, w / 2]])
        rot = corners @ np.array([[c, s], [-s, c]])
        ax.plot(-(y + rot[:, 1]), x + rot[:, 0], color="lime" if t == "Car" else "orange", lw=1.2)
        ax.text(-y, x + l / 2 + 0.8, f"{t} {n}", color="white", fontsize=7, ha="center")
    ax.set_xlim(-45, 45)
    ax.set_ylim(-30, 70)
    ax.set_aspect("equal")
    ax.set_title(f"frame {frame}: {len(pts)} points, {sum(1 for b in boxes if b[-1] > 0)} boxes with points",
                 color="white")
    ax.tick_params(colors="gray")
    handles = [plt.Line2D([], [], marker="o", ls="", color=CLASS_COLORS[k], label=names.get(k, str(k)))
               for k in sorted(set(int(c) for c in np.unique(sem)))]
    ax.legend(handles=handles, loc="lower left", fontsize=8, facecolor="black", labelcolor="white")

    ax = axes[1]
    ax.set_facecolor("black")
    if has_img:
        ax.imshow(plt.imread(img_path))
        ax.set_title("front camera (image_2)", color="white")
        ax.axis("off")
    else:
        sc = ax.scatter(-pts[:, 1], pts[:, 0], s=0.3, c=pts[:, 3], cmap="jet", vmin=0, vmax=0.9, linewidths=0)
        ax.set_xlim(-45, 45)
        ax.set_ylim(-30, 70)
        ax.set_aspect("equal")
        ax.set_title("intensity", color="white")
        ax.tick_params(colors="gray")
        fig.colorbar(sc, ax=ax)

    fig.tight_layout()
    fig.savefig(out, dpi=110, facecolor="black")
    print("saved", out)
    counts = np.bincount(sem.astype(np.int64), minlength=12)
    for k, n in enumerate(counts):
        if n:
            print(f"  {names.get(k, k):14s} {n:7d} points")


if __name__ == "__main__":
    main()
