#!/usr/bin/env python3
# Extract a contiguous segment of camera frames from a MARS-LVIG bag for the
# front-end tracking comparison. Runs inside the openvins:noetic container
# (needs rosbag + cv2). CompressedImage.data is JPEG -> decode directly.
import rosbag, cv2, numpy as np, os, sys

bag_path = sys.argv[1] if len(sys.argv) > 1 else "/data/mars-lvig/AMvalley01.bag"
out_dir  = sys.argv[2] if len(sys.argv) > 2 else "/work/_frames"
START    = int(sys.argv[3]) if len(sys.argv) > 3 else 200   # skip takeoff
N        = int(sys.argv[4]) if len(sys.argv) > 4 else 200   # consecutive frames
TOPIC    = "/left_camera/image/compressed"
os.makedirs(out_dir, exist_ok=True)

i = saved = 0
b = rosbag.Bag(bag_path, "r")
for _, m, _ in b.read_messages(topics=[TOPIC]):
    if i >= START:
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        cv2.imwrite(os.path.join(out_dir, f"frame_{saved:04d}.png"), img)
        saved += 1
        if saved >= N:
            break
    i += 1
b.close()
print(f"saved {saved} frames ({img.shape[1]}x{img.shape[0]}) to {out_dir} starting at bag frame {START}")
