#!/usr/bin/env python3
"""Visualize ground-truth trajectory from poses.csv in 3D."""
import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection

ROOT = Path(__file__).parent.parent
POSES = ROOT / "_in/isaac-sim-20260623/poses.csv"
OUT   = ROOT / "_in/isaac-sim-20260623/gt_trajectory_3d.png"

rows = list(csv.DictReader(POSES.open()))
xs = np.array([float(r["x"]) for r in rows])
ys = np.array([float(r["y"]) for r in rows])
zs = np.array([float(r["z"]) for r in rows])
t  = np.arange(len(xs)) / 50.0  # seconds at 50 Hz

# --- colour by time ---
norm = plt.Normalize(t.min(), t.max())
cmap = plt.cm.plasma

fig = plt.figure(figsize=(14, 7))

# --- 3-D trajectory ---
ax3 = fig.add_subplot(121, projection="3d")
points = np.array([xs, ys, zs]).T.reshape(-1, 1, 3)
segs   = np.concatenate([points[:-1], points[1:]], axis=1)
lc = Line3DCollection(segs, cmap=cmap, norm=norm, linewidth=0.8, alpha=0.9)
lc.set_array(t[:-1])
ax3.add_collection3d(lc)
ax3.scatter(*[xs[0]], *[ys[0]], *[zs[0]], c="lime",   s=60, zorder=5, label="start")
ax3.scatter(*[xs[-1]], *[ys[-1]], *[zs[-1]], c="red", s=60, zorder=5, label="end")
ax3.set_xlim(xs.min(), xs.max())
ax3.set_ylim(ys.min(), ys.max())
ax3.set_zlim(zs.min(), zs.max())
ax3.set_xlabel("East (m)"); ax3.set_ylabel("North (m)"); ax3.set_zlabel("Up (m)")
ax3.set_title("GT trajectory — 3-D view")
ax3.legend(fontsize=8)

# colour bar
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])
fig.colorbar(sm, ax=ax3, shrink=0.5, pad=0.1, label="Time (s)")

# --- altitude over time ---
ax2 = fig.add_subplot(122)
ax2.plot(t, zs, linewidth=0.8, color="steelblue")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Altitude above takeoff (m)")
ax2.set_title("Altitude profile")
ax2.grid(True, alpha=0.3)
ax2.annotate(f"max {zs.max():.1f} m", xy=(t[zs.argmax()], zs.max()),
             xytext=(10, -15), textcoords="offset points",
             arrowprops=dict(arrowstyle="->", color="gray"), fontsize=8)

fig.suptitle(
    f"Isaac Sim GT  —  {len(rows):,} poses @ 50 Hz  |  "
    f"duration {t[-1]:.0f} s  |  path {np.sum(np.sqrt(np.diff(xs)**2+np.diff(ys)**2+np.diff(zs)**2)):.0f} m",
    fontsize=10,
)
fig.tight_layout()
fig.savefig(OUT, dpi=150, bbox_inches="tight")
print(f"Saved: {OUT}")
plt.show()
