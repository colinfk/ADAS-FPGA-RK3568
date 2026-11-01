#!/bin/sh
set -eu
export ADAS_HUD_MODE=cruise
export ADAS_ADJACENT_POLICY=cruise
exec sh "$(dirname -- "$0")/_start_runtime.sh" "$@"
