# ============================================================================
# Stop / cancel the VIO recorder started by vio-recorder-pai.py.
#
# Run this in the Isaac Sim Script Editor to cleanly tear down a running (or
# armed-but-waiting) recording: removes the physics callback, stops the
# background image writer, and flushes+closes every open CSV.
#
# Safe to run even if nothing is recording (it just reports "nothing to stop").
# After stopping you can re-run vio-recorder-pai.py to start a fresh run
# (it writes a NEW ~/vio_dataset/<run_ts>/ dir, so old data is never clobbered).
# ============================================================================
from isaacsim.core.api.world import World

rec = globals().get("_VIO_REC")
if not rec:
    print("[REC] nothing to stop — no _VIO_REC found (recorder not running).")
else:
    world = World.instance()

    # 1. detach the physics callback so no more frames are sampled
    if world is not None:
        try:
            world.remove_physics_callback(rec["cb"])
            print(f"[REC] removed physics callback '{rec['cb']}'.")
        except Exception as e:
            print(f"[REC] callback already gone: {e}")

    # 2. signal the background image-writer thread to exit
    try:
        rec["q"].put_nowait(None)
        print("[REC] image writer signalled to stop.")
    except Exception as e:
        print(f"[REC] could not signal image writer: {e}")

    # 3. flush + close all CSV file handles
    closed = 0
    for fh in rec.get("files", []):
        try:
            fh.flush()
            fh.close()
            closed += 1
        except Exception:
            pass
    print(f"[REC] closed {closed} file(s).")

    print(f">>> Recording stopped. Output dir: {rec.get('dir')}")
    print(">>> Re-run vio-recorder-pai.py before takeoff to start a fresh run.")

    # forget it so a second run of this script is a no-op
    globals().pop("_VIO_REC", None)
