#!/usr/bin/env python3
# Extract raw mono cam0 frames from a TUM-VI bag (sensor_msgs/Image, mono8 fisheye).
# Run inside the openvins:noetic container. cf. extract_frames.py (compressed/AMvalley).
import rosbag, cv2, numpy as np, os, sys
bag   = sys.argv[1] if len(sys.argv) > 1 else "/data/tumvi/dataset-room1_512_16.bag"
out   = sys.argv[2] if len(sys.argv) > 2 else "/work/_frames_tumvi"
START = int(sys.argv[3]) if len(sys.argv) > 3 else 100
N     = int(sys.argv[4]) if len(sys.argv) > 4 else 200
TOPIC = "/cam0/image_raw"
os.makedirs(out, exist_ok=True)
i = s = 0
b = rosbag.Bag(bag, "r")
for _, m, _ in b.read_messages(topics=[TOPIC]):
    if i >= START:
        bpp = len(m.data) // (m.height * m.width)
        if bpp == 2:  # mono16 (TUM-VI 512_16): view as uint16, scale to 8-bit
            img16 = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
            img = (img16 >> 8).astype(np.uint8) if img16.max() > 255 else img16.astype(np.uint8)
        else:
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
        cv2.imwrite(os.path.join(out, f"frame_{s:04d}.png"), img); s += 1
        if s >= N: break
    i += 1
b.close()
print(f"saved {s} frames ({m.width}x{m.height}, {m.encoding}) to {out}")
