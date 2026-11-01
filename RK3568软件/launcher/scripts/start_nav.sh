#!/bin/sh
set -eu
export ADAS_HUD_MODE=nav
exec sh "$(dirname -- "$0")/_start_runtime.sh" --nav-composite "$@"
