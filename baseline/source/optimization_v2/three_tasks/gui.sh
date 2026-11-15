#!/usr/bin/env bash
set -Eeuo pipefail
monitor_base="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$monitor_base/../env.sh"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "$monitor_base/../venvs/scene/bin/python" "$monitor_base/monitor/server.py" "$@"
