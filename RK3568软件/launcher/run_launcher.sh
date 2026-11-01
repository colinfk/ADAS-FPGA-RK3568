#!/bin/sh
set -eu
src=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
mkdir -p "$src/_tmp"
export TMPDIR="$src/_tmp" XDG_CACHE_HOME="$src/_tmp" PYTHONDONTWRITEBYTECODE=1
exec python3 "$src/launcher.py" --config "$src/launcher.ini" "$@"
