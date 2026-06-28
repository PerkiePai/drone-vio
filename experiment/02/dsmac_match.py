#!/usr/bin/env python3
"""DSMAC-style vision-only geo-localization for the nadir drone camera.

Absolute (drift-free) position by matching each nadir frame against a pre-stored
satellite ortho — the complement to flow-odom (which is locally metric but drifts).
Per frame: de-rotate by -yaw, scale by AGL/fx/GSD to the ortho's m/px, match with
ALIKED+LightGlue, RANSAC-homography the frame centre onto the ortho, convert the
hit pixel back to ENU and score against geo.csv. NO scale integration → the
monocular-scale problem is bypassed entirely; the cost is needing a reference map.

This is the "global fix" layer of a GPS-denied / lidar-free / rangefinder-free
stack: flow-odom propagates between fixes, DSMAC re-anchors drift where the ground
has texture (TERCOM/DSMAC + the SPRIN-D particle-filter pattern). See frontend/CLAUDE.md.

Reference map: Esri World Imagery tiles (no key) stitched over the flight footprint
from geo.csv; cached under <out>/ortho_tiles. Run in the `cv` conda env (torch +
lightglue). Prototype caveats: uses GT for the per-frame search-window centre and
heading (validates the matching core; a deployed loop drives those from flow-odom +
AHRS), and AGL from agl_cache.npz (a rough baro/flatness estimate suffices — RANSAC
tolerates scale error far better than flow-odom integration does).
"""
import argparse, csv, json, math, os, time, urllib.request
import sys

import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))  # experiment/02 -> repo root
sys.path.insert(0, HERE)  # all deps co-located
import flow_odometry as fo
from compare_tracking import LG

TILE_URL = ("https://services.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}")


def deg2tile(lat, lon, z):
    n = 2 ** z
    x = (lon + 180) / 360 * n
    y = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n
    return x, y


def build_ortho(geo, z, margin, tiles_dir):
    """Stitch Esri tiles covering the geo.csv footprint. Cached per tile."""
    os.makedirs(tiles_dir, exist_ok=True)
    lat = np.array([float(r["lat_deg"]) for r in geo])
    lon = np.array([float(r["lon_deg"]) for r in geo])
    la0, la1 = lat.min() - margin, lat.max() + margin
    lo0, lo1 = lon.min() - margin, lon.max() + margin
    x0, _ = deg2tile(la0, lo0, z); x1, _ = deg2tile(la1, lo1, z)
    _, y0 = deg2tile(la1, lo0, z); _, y1 = deg2tile(la0, lo1, z)
    xa, xb = int(math.floor(min(x0, x1))), int(math.floor(max(x0, x1)))
    ya, yb = int(math.floor(min(y0, y1))), int(math.floor(max(y0, y1)))
    W, H = (xb - xa + 1) * 256, (yb - ya + 1) * 256
    ortho = np.zeros((H, W, 3), np.uint8)
    placed = 0
    for ty in range(ya, yb + 1):
        for tx in range(xa, xb + 1):
            fp = os.path.join(tiles_dir, f"{z}_{ty}_{tx}.jpg")
            if not os.path.exists(fp):
                url = TILE_URL.format(z=z, y=ty, x=tx)
                for attempt in range(3):
                    try:
                        req = urllib.request.Request(url, headers={"User-Agent": "research-prototype"})
                        open(fp, "wb").write(urllib.request.urlopen(req, timeout=20).read())
                        break
                    except Exception:
                        if attempt == 2:
                            print(f"  tile fail {z}/{ty}/{tx}")
                        time.sleep(0.5)
            im = cv2.imread(fp) if os.path.exists(fp) else None
            if im is not None and im.shape[:2] == (256, 256):
                ortho[(ty - ya) * 256:(ty - ya + 1) * 256, (tx - xa) * 256:(tx - xa + 1) * 256] = im
                placed += 1
    meta = dict(z=z, xa=xa, ya=ya, W=W, H=H)
    print(f"ortho {xb-xa+1}x{yb-ya+1} tiles = {W}x{H}px (placed {placed}/{(xb-xa+1)*(yb-ya+1)})")
    return ortho, meta


def warp_north_up(im, yaw_deg, f):
    """De-rotate by -yaw and scale by f, expanding the canvas. Returns warped image
    and the location of the original frame centre (the camera nadir point) in it."""
    h, w = im.shape
    M = cv2.getRotationMatrix2D((w / 2, h / 2), -yaw_deg, f)
    cos, sin = abs(M[0, 0]), abs(M[0, 1])
    nw, nh = int(w * cos + h * sin), int(w * sin + h * cos)
    M[0, 2] += nw / 2 - w / 2
    M[1, 2] += nh / 2 - h / 2
    return cv2.warpAffine(im, M, (nw, nh)), (M @ np.array([w / 2, h / 2, 1.0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(ROOT, "_in/isaac-sim-20260624_2337"))
    ap.add_argument("--z", type=int, default=19, help="web-mercator tile zoom (19 ~ 0.29 m/px)")
    ap.add_argument("--margin", type=float, default=0.0016, help="ortho padding in degrees (~175 m)")
    ap.add_argument("--n", type=int, default=60, help="frames sampled across the flight")
    ap.add_argument("--win", type=int, default=420, help="ortho search half-window (px) around the prior")
    ap.add_argument("--min_inliers", type=int, default=15)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    D = args.dir
    out = args.out or os.path.join(D, "dsmac_geoloc.png")

    geo = list(csv.DictReader(open(os.path.join(D, "geo.csv"))))
    tiles_dir = os.path.join(D, "ortho_tiles")
    ortho, meta = build_ortho(geo, args.z, args.margin, tiles_dir)
    orthog = cv2.cvtColor(ortho, cv2.COLOR_BGR2GRAY)
    z = meta["z"]; nN = 2 ** z

    K, R_CtoI, recs = fo.load_dataset(D)
    fx = K[0, 0]
    agl = np.load(os.path.join(D, "agl_cache.npz"))["agl"]
    g = json.load(open(os.path.join(D, "georef.json")))
    lat0, lon0 = g["origin"]["latitude"], g["origin"]["longitude"]
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    GSD = 156543.03392 * math.cos(math.radians(lat0)) / nN

    def enu_to_px(dE, dN):
        lat, lon = lat0 + dN / mlat, lon0 + dE / mlon
        gx = (lon + 180) / 360 * nN
        gy = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * nN
        return (gx - meta["xa"]) * 256, (gy - meta["ya"]) * 256

    def px_to_enu(px, py):
        gx, gy = meta["xa"] + px / 256, meta["ya"] + py / 256
        lon = gx / nN * 360 - 180
        lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * gy / nN))))
        return (lon - lon0) * mlon, (lat - lat0) * mlat

    lg = LG()
    idxs = np.linspace(50, len(recs) - 50, args.n).astype(int)
    res = []  # i, gE, gN, eE, eN, inliers, err_m
    for fi in idxs:
        r = recs[fi]
        im = cv2.imread(r["img"], cv2.IMREAD_GRAYSCALE)
        yaw = math.degrees(math.atan2(r["R_wb"][1, 0], r["R_wb"][0, 0]))
        f = (agl[fi] / fx) / GSD
        gE, gN = r["gt"][0], r["gt"][1]
        cx, cy = enu_to_px(gE, gN)
        x0, y0 = max(0, int(cx - args.win)), max(0, int(cy - args.win))
        win = orthog[y0:y0 + 2 * args.win, x0:x0 + 2 * args.win]
        q, cpt = warp_north_up(im, yaw, f)
        inl, err, eE, eN = 0, np.nan, np.nan, np.nan
        try:
            k0, k1, _ = lg.match(lg.extract(q), lg.extract(win))
            if len(k0) >= 8:
                Hm, mask = cv2.findHomography(k0, k1, cv2.RANSAC, 5.0)
                if Hm is not None and mask is not None:
                    inl = int(mask.sum())
                    if inl >= args.min_inliers:
                        p = Hm @ np.array([cpt[0], cpt[1], 1.0]); p /= p[2]
                        eE, eN = px_to_enu(x0 + p[0], y0 + p[1])
                        err = math.hypot(eE - gE, eN - gN)
        except Exception:
            pass
        res.append((fi, gE, gN, eE, eN, inl, err))
    res = np.array(res, float)

    acc = res[(res[:, 5] >= args.min_inliers) & np.isfinite(res[:, 6])]
    print(f"\nframes tried {len(res)}  matched {len(acc)} = {100*len(acc)/len(res):.0f}%")
    if len(acc):
        e = acc[:, 6]
        print(f"abs-position error: median {np.median(e):.1f} m  mean {e.mean():.1f}  "
              f"p90 {np.percentile(e,90):.1f}  max {e.max():.1f}")
        print(f"success <30m {100*np.mean(e<30):.0f}%  <50m {100*np.mean(e<50):.0f}%  "
              f"<100m {100*np.mean(e<100):.0f}%")

    gt = np.array([r["gt"][:2] for r in recs])
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
    a = ax[0]
    a.plot(gt[:, 0], gt[:, 1], "b-", lw=1, label="GT track")
    for row in res:
        if row[5] >= args.min_inliers and np.isfinite(row[6]):
            a.plot([row[1], row[3]], [row[2], row[4]], "r-", lw=0.5, alpha=0.5)
            a.plot(row[3], row[4], "r.", ms=6)
    a.plot([], [], "r.", label="DSMAC fix")
    a.set_title("DSMAC absolute fixes vs GT (ENU)")
    a.set_xlabel("E (m)"); a.set_ylabel("N (m)")
    a.axis("equal")
    a.set_xlim(gt[:, 0].min() - 150, gt[:, 0].max() + 150)
    a.set_ylim(gt[:, 1].min() - 150, gt[:, 1].max() + 150)
    a.invert_yaxis(); a.grid(alpha=0.3); a.legend()
    a = ax[1]
    if len(acc):
        a.hist(acc[:, 6], bins=15, color="steelblue", edgecolor="k")
        a.set_title(f"Abs-position error ({len(acc)}/{len(res)} matched, "
                    f"median {np.median(acc[:,6]):.0f} m)")
    a.set_xlabel("error (m)"); a.set_ylabel("count"); a.grid(alpha=0.3)
    fig.suptitle("DSMAC vision-only geo-localization — nadir frames vs satellite ortho", fontsize=12)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
