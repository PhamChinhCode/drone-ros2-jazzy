#!/bin/bash
# Giu tong dung luong ~/drone_logs <= DRONE_LOG_CAP_GB (mac dinh 10 GB): xoa file bag cu nhat truoc.
#
# Bag ghi lien tuc moi khi stack chay (ke ca de qua dem, 09-17 tung lam day the SD 229 GB) va duoc
# cat file moi 10 phut (full_system.launch.py) nen xoa theo TUNG FILE .mcap, khong can xoa ca thu
# muc dang ghi. File moi nhat (dang ghi) KHONG BAO GIO bi xoa. Thu muc bag rong thi xoa luon.
# Nhat ky nghiep vu (<mission_id>/events.jsonl + anh) nho, khong dong vao.
#
# drone_startup.sh goi luc khoi dong va moi 10 phut. Chay tay: scripts/prune_logs.sh
set -u
LOG_DIR="${DRONE_LOG_DIR:-$HOME/drone_logs}"
CAP_BYTES=$(( ${DRONE_LOG_CAP_GB:-10} * 1024 * 1024 * 1024 ))
[ -d "$LOG_DIR" ] || exit 0

total=$(du -sb "$LOG_DIR" | cut -f1)
(( total <= CAP_BYTES )) && exit 0

# Cu nhat truoc (theo mtime); bo dong cuoi = file moi nhat (dang ghi).
mapfile -t files < <(find "$LOG_DIR" -path "$LOG_DIR/bag_*" -type f \( -name '*.mcap' -o -name '*.db3' \) \
                       -printf '%T@ %s %p\n' | sort -n | head -n -1)
removed=0
for line in "${files[@]}"; do
  (( total <= CAP_BYTES )) && break
  size=$(cut -d' ' -f2 <<<"$line")
  path=$(cut -d' ' -f3- <<<"$line")
  rm -f -- "$path" && total=$(( total - size )) && removed=$(( removed + 1 ))
done
# Thu muc bag khong con file du lieu (chi con metadata.yaml) -> xoa.
for d in "$LOG_DIR"/bag_*/; do
  [ -d "$d" ] || continue
  compgen -G "$d*.mcap" >/dev/null || compgen -G "$d*.db3" >/dev/null || rm -rf -- "$d"
done
(( removed )) && echo "[prune_logs] da xoa $removed file bag cu, con $(( total / 1024 / 1024 )) MB"
exit 0
