#!/usr/bin/env bash
# Run TensorBoard against monitor_value_logger event files (host or same machine as logs).
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
TB_VENV="${WS}/.venv_tb"
LOGDIR="${1:-${WS}/log/monitor_value/tb}"
HOST="${TB_HOST:-0.0.0.0}"
PORT="${TB_PORT:-6006}"

if [[ ! -x "${TB_VENV}/bin/tensorboard" ]]; then
  echo "TensorBoard venv missing. Create with:" >&2
  echo "  python3 -m venv ${TB_VENV} && ${TB_VENV}/bin/pip install tensorboard" >&2
  exit 1
fi

mkdir -p "${LOGDIR}"
echo "logdir=${LOGDIR}"
echo "open http://<this-host-ip>:${PORT}  (or http://127.0.0.1:${PORT} locally)"
exec "${TB_VENV}/bin/tensorboard" --logdir "${LOGDIR}" --host "${HOST}" --port "${PORT}"
