"""Capture one frame every 1 second from each video in _in, saving frames back to _in."""
import os
import glob
import cv2

# _in lives at the project root, two levels up (frontend/superglue/ -> root).
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
IN_DIR = os.path.join(ROOT, "_in")
INTERVAL_SEC = 1.0

def capture(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Could not open {video_path}")
        return
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    if fps <= 0:
        fps = 30.0  # fallback
    step = max(1, int(round(fps * INTERVAL_SEC)))
    name = os.path.splitext(os.path.basename(video_path))[0]

    frame_idx = 0
    saved = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame_idx % step == 0:
            sec = int(frame_idx / fps)
            out = os.path.join(IN_DIR, f"{name}_{sec:04d}s.jpg")
            cv2.imwrite(out, frame)
            saved += 1
        frame_idx += 1
    cap.release()
    print(f"{os.path.basename(video_path)}: {saved} frames saved (fps={fps:.2f}, every {step} frames)")

def main():
    videos = []
    for ext in ("*.mp4", "*.avi", "*.mov", "*.mkv"):
        videos.extend(glob.glob(os.path.join(IN_DIR, ext)))
    if not videos:
        print(f"No videos found in {IN_DIR}")
        return
    for v in videos:
        capture(v)

if __name__ == "__main__":
    main()
