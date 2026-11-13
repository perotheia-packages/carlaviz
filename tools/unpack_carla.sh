#!/usr/bin/env bash
# Unpack carla_ws's vendored LibCarla 0.9.15 client (carla_ws third_party/carla/libcarla-client.tar.zst, the
# client the stack itself links) into third_party/carla/install (gitignored). carla_gt never vendors a second copy.
#
#   CARLA_WS=/home/axadmin/repo/perotheia-packages/carla_ws tools/unpack_carla.sh [-f]
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
CARLA_WS="${CARLA_WS:-$HERE/../carla_ws}"
TAR="$CARLA_WS/third_party/carla/libcarla-client.tar.zst"
DST="$HERE/third_party/carla/install"
if [ -e "$DST/include/carla" ] && [ "${1:-}" != "-f" ]; then
    echo "carla client already at $DST (use -f to re-extract)"; exit 0
fi
[ -f "$TAR" ] || { echo "missing $TAR (set CARLA_WS)" >&2; exit 1; }
mkdir -p "$DST"
zstd -dq -c "$TAR" | tar -C "$DST" -xf -
echo "unpacked libcarla client ($TAR) -> $DST"
