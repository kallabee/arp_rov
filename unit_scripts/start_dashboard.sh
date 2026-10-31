#!/usr/bin/env bash
# Start processes the Kankai dashboard needs.
# If they are already running, ask whether to restart (unless -y / -n).
#
#   unit_scripts/start_dashboard.sh
#   unit_scripts/start_dashboard.sh -y      # restart without asking
#   unit_scripts/start_dashboard.sh -n      # keep running processes; start missing only
#   unit_scripts/start_dashboard.sh --status
set -euo pipefail

ORIG_ARGS=("$@")
WS="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="${WS}/log/dashboard"
MEDIAMTX_BIN="${MEDIAMTX_BIN:-/home/arp/mediamtx/mediamtx}"
MEDIAMTX_DIR="$(dirname "${MEDIAMTX_BIN}")"
WEB_PORT="${WEB_PORT:-8080}"
ROS_DOCKER="${ROS_DOCKER:-jazzy}"

YES=""
NO=""
STATUS_ONLY=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [-y|-n|--status]

Start the ROS stack, thruster listener, and MediaMTX used by the dashboard.
If the ROS stack or thruster_controller is already running, ask whether to restart.
MediaMTX (camera stream) is never restarted: leave it if up, start it only if down.

  -y, --restart     Restart running ROS/thruster processes without asking
  -n, --no-restart  Do not restart; start only processes that are down
  --status          Print what is running and exit
  ROS_SETUP         Optional path to ROS setup.bash (default: /opt/ros/jazzy/setup.bash)
  ROS_DOCKER        Docker container with ROS (default: jazzy). Used when this
                    script is run on the host without /opt/ros.
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

port_open() {
  local port="$1"
  python3 - "$port" <<'PY' 2>/dev/null
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
}

# MediaMTX may run on the host while ROS runs in docker (host network).
# Detect by process or by the usual stream ports.
mtx_up() {
  [[ "$(count_pids "${MEDIAMTX_SPEC%%|*}")" -gt 0 ]] && return 0
  local p
  for p in 8889 8554 8888; do
    port_open "$p" && return 0
  done
  return 1
}

print_status() {
  local spec pat name n
  echo "Dashboard processes:"
  for spec in "${STACK_SPECS[@]}" "$THRUSTER_SPEC"; do
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
  if mtx_up; then
    printf '  %-36s running\n' "mediamtx"
  else
    printf '  %-36s down\n' "mediamtx"
  fi
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

find_ros_setup() {
  local cand
  if [[ -n "${ROS_SETUP:-}" ]]; then
    printf '%s\n' "${ROS_SETUP}"
    return 0
  fi
  for cand in \
    "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" \
    /opt/ros/jazzy/setup.bash \
    /opt/ros/humble/setup.bash \
    /opt/ros/iron/setup.bash
  do
    if [[ -r "$cand" ]]; then
      printf '%s\n' "$cand"
      return 0
    fi
  done
  return 1
}

ros_available() {
  find_ros_setup >/dev/null || command -v ros2 >/dev/null 2>&1
}

reexec_in_ros_docker() {
  [[ -n "${INSIDE_ROS_DOCKER:-}" ]] && return 0
  ros_available && return 0
  if ! command -v docker >/dev/null 2>&1; then
    echo "ROS setup.bash not found (tried /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash)." >&2
    echo "This host has no ROS install. Enter the jazzy container, or set ROS_SETUP." >&2
    echo "  docker exec -it ${ROS_DOCKER} /bin/bash" >&2
    echo "  cd ${WS} && ./start_dashboard.sh" >&2
    exit 1
  fi
  local running=""
  running="$(docker inspect -f '{{.State.Running}}' "${ROS_DOCKER}" 2>/dev/null || true)"
  if [[ -z "$running" ]]; then
    echo "ROS is not on this host, and docker container '${ROS_DOCKER}' does not exist." >&2
    echo "Create it with --name ${ROS_DOCKER}, or set ROS_DOCKER to the running container name." >&2
    exit 1
  fi
  if [[ "$running" != "true" ]]; then
    echo "Starting docker container '${ROS_DOCKER}'..."
    docker start "${ROS_DOCKER}" >/dev/null
  fi
  echo "ROS is in docker container '${ROS_DOCKER}'; continuing there (MediaMTX stays on the host)."
  local flags=(
    -u "$(id -u):$(id -g)"
    -w "${WS}"
    -e INSIDE_ROS_DOCKER=1
    -e HOME="${HOME:-/home/arp}"
    -e USER="${USER:-arp}"
    -e WEB_PORT="${WEB_PORT}"
    -e ROS_DOCKER="${ROS_DOCKER}"
  )
  [[ -t 0 ]] && flags+=(-i)
  [[ -t 1 ]] && flags+=(-t)
  exec docker exec "${flags[@]}" "${ROS_DOCKER}" /bin/bash "$(cd "$(dirname "$0")" && pwd)/$(basename "$0")" "${ORIG_ARGS[@]}"
}

source_ws() {
  export PYTHONPATH="${PYTHONPATH:-}:/usr/lib/aarch64-linux-gnu/python3.12/site-packages"
  export LIBCAMERA_DRM_FORMATS="${LIBCAMERA_DRM_FORMATS:-0}"
  local ros_setup=""
  ros_setup="$(find_ros_setup || true)"
  # ROS setup.bash probes optional vars like AMENT_TRACE_SETUP_FILES; with
  # `set -u` that aborts. Disable nounset only while sourcing overlays.
  set +u
  if [[ -n "$ros_setup" ]]; then
    # shellcheck disable=SC1090
    source "$ros_setup"
  elif command -v ros2 >/dev/null 2>&1; then
    echo "ROS underlay not found; using ros2 already on PATH."
  else
    set -u
    echo "ROS setup.bash not found inside this environment." >&2
    exit 1
  fi
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
  if [[ -f /home/arp/work/docker/fastdds.xml ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/home/arp/work/docker/fastdds.xml}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  fi
  if [[ -r "${WS}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WS}/install/setup.bash"
  else
    set -u
    echo "Workspace overlay missing: ${WS}/install/setup.bash" >&2
    echo "Build the workspace (colcon build) first." >&2
    exit 1
  fi
  set -u
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
  : >"${LOGDIR}/thruster.log"
  nohup ros2 run thruster_controller listener \
    >>"${LOGDIR}/thruster.log" 2>&1 &
  echo $! >"${LOGDIR}/thruster.pid"
  local i
  for i in $(seq 1 50); do
    if grep -Eq 'Started\.|Light PWM ready|minimal_subscriber' "${LOGDIR}/thruster.log" 2>/dev/null; then
      echo "thruster_controller is up"
      return 0
    fi
    if ! kill -0 "$(cat "${LOGDIR}/thruster.pid")" 2>/dev/null; then
      echo "thruster_controller exited during startup (lights/thrusters will not run):" >&2
      tail -n 25 "${LOGDIR}/thruster.log" >&2 || true
      return 0
    fi
    sleep 0.4
  done
  echo "Warning: thruster_controller still starting (ESC arm delay); see ${LOGDIR}/thruster.log" >&2
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

lan_ipv4s() {
  python3 - <<'PY'
import fcntl, socket, struct, os
skip = {"lo", "docker0"}
names = sorted(os.listdir("/sys/class/net"))
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for name in names:
    if name in skip or name.startswith(("veth", "br-", "cni")):
        continue
    try:
        ip = socket.inet_ntoa(fcntl.ioctl(
            s.fileno(), 0x8915, struct.pack("256s", name.encode()[:15])
        )[20:24])
    except OSError:
        continue
    if ip.startswith("127.") or ip.startswith("172.17."):
        continue
    print(f"{name} {ip}")
PY
}

reexec_in_ros_docker

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

if mtx_up; then
  echo "MediaMTX already running — leaving camera stream as-is."
  need_mtx_start=0
fi

if [[ "$need_stack_start" -eq 1 || "$need_thruster_start" -eq 1 ]]; then
  source_ws
fi

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

echo
echo "Logs: ${LOGDIR}/"
echo "Dashboard: http://127.0.0.1:${WEB_PORT}/"
lan_ipv4s | while read -r iface ip; do
  [[ -z "${ip:-}" ]] && continue
  echo "Dashboard: http://${ip}:${WEB_PORT}/  (${iface})"
done
echo
print_status
