#!/usr/bin/env bash
# Discover camera nodes on the Docker host instead of assuming fixed indices.
set -euo pipefail

compose_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
override="$(mktemp "${TMPDIR:-/tmp}/irl-camera-compose.XXXXXX.yml")"
trap 'rm -f -- "$override"' EXIT

shopt -s nullglob
camera_devices=(/dev/video[0-9]* /dev/media[0-9]*)
{
    printf 'services:\n  perception:\n    devices: !override'
    if ((${#camera_devices[@]} == 0)); then
        printf ' []\n'
    else
        printf '\n'
        for device in "${camera_devices[@]}"; do
            printf '      - "%s:%s"\n' "$device" "$device"
        done
    fi
} > "$override"

if ((${#camera_devices[@]} == 0)); then
    printf 'No /dev/video* or /dev/media* devices found; check the camera connection if using RealSense.\n' >&2
fi

# Preserve the base file's project name and relative volume/build paths.
docker compose -f "$compose_dir/docker-compose.yml" -f "$override" "$@"
