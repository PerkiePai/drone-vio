#!/usr/bin/env bash
# Unattended retry: download AMvalley01.bag the moment Google's quota frees.
# Exits 0 on success (file present and >100MB), 1 if it gives up.
set -u
ID="1NTecR3tb2-NYZDPH_p94bFy3lYmsQ53b"
DEST="/home/innovation/pai/drone-vio/_in/mars-lvig/AMvalley01.bag"
LOG="/tmp/amvalley_retry.log"
MAX=48          # attempts
SLEEP=1800      # 30 min between tries

for i in $(seq 1 $MAX); do
  echo "[$(date '+%F %T')] attempt $i/$MAX" >> "$LOG"
  conda run -n cv gdown "https://drive.google.com/uc?id=$ID" -O "$DEST" >> "$LOG" 2>&1
  if [ -f "$DEST" ] && [ "$(stat -c%s "$DEST")" -gt 104857600 ]; then
    echo "[$(date '+%F %T')] SUCCESS: $(du -h "$DEST" | cut -f1)" >> "$LOG"
    exit 0
  fi
  rm -f "$DEST"
  echo "[$(date '+%F %T')] still blocked; sleeping ${SLEEP}s" >> "$LOG"
  sleep "$SLEEP"
done
echo "[$(date '+%F %T')] gave up after $MAX attempts" >> "$LOG"
exit 1
