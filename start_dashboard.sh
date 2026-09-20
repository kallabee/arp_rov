#!/usr/bin/env bash
exec "$(cd "$(dirname "$0")" && pwd)/unit_scripts/start_dashboard.sh" "$@"
