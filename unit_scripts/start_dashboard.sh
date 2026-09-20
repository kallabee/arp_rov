#!/usr/bin/env bash
# Start processes the Kankai dashboard needs.
# If they are already running, ask whether to restart (unless -y / -n).
#
#   unit_scripts/start_dashboard.sh
#   unit_scripts/start_dashboard.sh -y      # restart without asking
#   unit_scripts/start_dashboard.sh -n      # keep running processes; start missing only
#   unit_scripts/start_dashboard.sh --status
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="${WS}/log/dashboard"
MEDIAMTX_BIN="${MEDIAMTX_BIN:-/home/arp/mediamtx/mediamtx}"
MEDIAMTX_DIR="$(dirname "${MEDIAMTX_BIN}")"
WEB_PORT="${WEB_PORT:-8080}"

YES=""
NO=""
STATUS_ONLY=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [-y|-n|--status]

Start the ROS stack, thruster listener, and MediaMTX used by the dashboard.
If something is already running, ask whether to restart it.

  -y, --restart     Restart running processes without asking
  -n, --no-restart  Do not restart; start only processes that are down
  --status          Print what is running and exit
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -y|--restart|--yes) YES=1 ;;
    -n|--no-restart) NO=1 ;;
    --status) STATUS_ONLY=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ -n "$YES" && -n "$NO" ]]; then
  echo "Use only one of -y and -n" >&2
  exit 2
fi

mkdir -p "${LOGDIR}"

# cmdline pattern, display name
# Keep patterns specific so this script itself is not matched.
STACK_SPECS=(
  "rov_monitor_imu_and_logging\\.launch\\.py|launch (imu+pub+logger+camera+web)"
  "imu_navigation/imu_nav_node|imu_navigation"
  "monitor_value_pub/monitor_value_pub|monitor_value_pub"
  "monitor_value_logger/monitor_value_logger|monitor_value_logger"
  "rpi_camera_ctrl/rpi_camera_ctrl|rpi_camera_ctrl"
  "lib/monitor_value_web/monitor_value_web --|monitor_value_web"
)
THRUSTER_SPEC="lib/thruster_controller/listener|thruster_controller"
MEDIAMTX_SPEC="${MEDIAMTX_BIN}|mediamtx"

pids_for() {
  local pat="$1"
  local pid cmd
  while read -r pid cmd; do
    [[ -z "${pid:-}" ]] && continue
    [[ "$pid" == "$$" || "$pid" == "$PPID" ]] && continue
    case "$cmd" in
      *start_dashboard.sh*|*CURSOR_SANDBOX*|*pgrep*) continue ;;
    esac
    printf '%s\n' "$pid"
  done < <(pgrep -af "$pat" 2>/dev/null || true)
}

count_pids() {
  local n=0
  local _p
  for _p in $(pids_for "$1"); do
    n=$((n + 1))
  done
  echo "$n"
}

print_status() {
  local spec pat name n
  echo "Dashboard processes:"
  for spec in "${STACK_SPECS[@]}" "$THRUSTER_SPEC" "$MEDIAMTX_SPEC"; do
    pat="${spec%%|*}"
    name="${spec##*|}"
    n="$(count_pids "$pat")"
    if [[ "$n" -eq 0 ]]; then
      printf '  %-36s down\n' "$name"
    elif [[ "$n" -eq 1 ]]; then
      printf '  %-36s running\n' "$name"
    else
      printf '  %-36s running (%s copies)\n' "$name" "$n"
    fi
  done
}

stack_running() {
  local spec pat n
  for spec in "${STACK_SPECS[@]}"; do
    pat="${spec%%|*}"
    n="$(count_pids "$pat")"
    [[ "$n" -gt 0 ]] && return 0
  done
  return 1
}

stack_complete() {
  local spec pat n
  for spec in "${STACK_SPECS[@]}"; do
    [[ "$spec" == *launch* ]] && continue
    pat="${spec%%|*}"
    n="$(count_pids "$pat")"
    [[ "$n" -eq 1 ]] || return 1
  done
  return 0
}

stack_has_dupes() {
  local spec pat n
  for spec in "${STACK_SPECS[@]}"; do
    pat="${spec%%|*}"
    n="$(count_pids "$pat")"
    [[ "$n" -gt 1 ]] && return 0
  done
  return 1
}

unique_pids() {
  local seen=" "
  local p
  for p in "$@"; do
    [[ -z "$p" ]] && continue
    case "$seen" in
      *" $p "*) continue ;;
    esac
    seen+="$p "
    printf '%s\n' "$p"
  done
}

stop_pids() {
  local pids=()
  local p
  while read -r p; do
    [[ -n "$p" ]] && pids+=("$p")
  done < <(unique_pids "$@")
  [[ ${#pids[@]} -eq 0 ]] && return 0

  echo "Stopping PIDs: ${pids[*]}"
  kill -INT "${pids[@]}" 2>/dev/null || true

  local i alive
  for i in $(seq 1 25); do
    alive=()
    for p in "${pids[@]}"; do
      kill -0 "$p" 2>/dev/null && alive+=("$p")
    done
    [[ ${#alive[@]} -eq 0 ]] && return 0
    sleep 0.2
  done

  kill -TERM "${alive[@]}" 2>/dev/null || true
  sleep 1
  alive=()
  for p in "${pids[@]}"; do
    kill -0 "$p" 2>/dev/null && alive+=("$p")
  done
  if [[ ${#alive[@]} -gt 0 ]]; then
    kill -KILL "${alive[@]}" 2>/dev/null || true
  fi
}

collect_spec_pids() {
  local spec pat
  for spec in "$@"; do
    pat="${spec%%|*}"
    pids_for "$pat"
  done
}

ask_restart() {
  local prompt="$1"
  local default="${2:-n}"
  if [[ -n "$YES" ]]; then
    return 0
  fi
  if [[ -n "$NO" ]]; then
    return 1
  fi
  if [[ ! -t 0 ]]; then
    echo "Non-interactive and already running (pass -y to restart, -n to skip)." >&2
    return 1
  fi
  local hint="[y/N]"
  [[ "$default" == y ]] && hint="[Y/n]"
  local ans=""
  read -r -p "${prompt} ${hint} " ans || true
  ans="${ans:-$default}"
  [[ "$ans" =~ ^[yY] ]]
}

source_ws() {
  export PYTHONPATH="${PYTHONPATH:-}:/usr/lib/aarch64-linux-gnu/python3.12/site-packages"
  export LIBCAMERA_DRM_FORMATS="${LIBCAMERA_DRM_FORMATS:-0}"
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
  cd "${WS}"
}

wait_port() {
  local port="$1"
  local label="$2"
  local i
  for i in $(seq 1 40); do
    if python3 - "$port" <<'PY' 2>/dev/null
import socket, sys
s = socket.socket()
s.settimeout(0.3)
try:
    s.connect(("127.0.0.1", int(sys.argv[1])))
except OSError:
    sys.exit(1)
finally:
    s.close()
PY
    then
      echo "${label} is listening on :${port}"
      return 0
    fi
    sleep 0.25
  done
  echo "Warning: ${label} did not open :${port} yet (see ${LOGDIR}/)" >&2
  return 1
}

start_stack() {
  echo "Starting ROS launch: rov_monitor_imu_and_logging.launch.py"
  nohup ros2 launch monitor_value_pub rov_monitor_imu_and_logging.launch.py \
    >>"${LOGDIR}/stack.log" 2>&1 &
  echo $! >"${LOGDIR}/stack.pid"
}

start_thruster() {
  echo "Starting thruster_controller listener"
  nohup ros2 run thruster_controller listener \
    >>"${LOGDIR}/thruster.log" 2>&1 &
  echo $! >"${LOGDIR}/thruster.pid"
}

start_mediamtx() {
  if [[ ! -x "${MEDIAMTX_BIN}" ]]; then
    echo "MediaMTX not found at ${MEDIAMTX_BIN} (camera stream skipped)" >&2
    return 0
  fi
  echo "Starting MediaMTX"
  (
    cd "${MEDIAMTX_DIR}"
    nohup "${MEDIAMTX_BIN}" >>"${LOGDIR}/mediamtx.log" 2>&1 &
    echo $! >"${LOGDIR}/mediamtx.pid"
  )
}

host_hint() {
  hostname -I 2>/dev/null | awk '{print $1}'
}

if [[ "$STATUS_ONLY" -eq 1 ]]; then
  print_status
  exit 0
fi

print_status
echo

need_stack_start=1
need_thruster_start=1
need_mtx_start=1
restart_default=n

if stack_running; then
  if stack_has_dupes || ! stack_complete; then
    echo "Partial stack or duplicate nodes detected."
    restart_default=y
  else
    echo "ROS dashboard stack is already running."
  fi
  if ask_restart "Restart the ROS dashboard stack?" "$restart_default"; then
    stop_pids $(collect_spec_pids "${STACK_SPECS[@]}")
    need_stack_start=1
  else
    echo "Leaving ROS dashboard stack as-is."
    need_stack_start=0
  fi
fi

if [[ "$(count_pids "${THRUSTER_SPEC%%|*}")" -gt 0 ]]; then
  if ask_restart "Restart thruster_controller?" n; then
    stop_pids $(collect_spec_pids "$THRUSTER_SPEC")
    need_thruster_start=1
  else
    echo "Leaving thruster_controller as-is."
    need_thruster_start=0
  fi
fi

if [[ "$(count_pids "${MEDIAMTX_SPEC%%|*}")" -gt 0 ]]; then
  if ask_restart "Restart MediaMTX (camera stream)?" n; then
    stop_pids $(collect_spec_pids "$MEDIAMTX_SPEC")
    need_mtx_start=1
  else
    echo "Leaving MediaMTX as-is."
    need_mtx_start=0
  fi
fi

source_ws

if [[ "$need_stack_start" -eq 1 ]]; then
  start_stack
fi
if [[ "$need_thruster_start" -eq 1 ]]; then
  start_thruster
fi
if [[ "$need_mtx_start" -eq 1 ]]; then
  start_mediamtx
fi

if [[ "$need_stack_start" -eq 1 ]]; then
  wait_port "$WEB_PORT" "Dashboard" || true
fi

ip="$(host_hint)"
echo
echo "Logs: ${LOGDIR}/"
if [[ -n "$ip" ]]; then
  echo "Dashboard: http://${ip}:${WEB_PORT}/"
else
  echo "Dashboard: http://127.0.0.1:${WEB_PORT}/"
fi
echo
print_status
