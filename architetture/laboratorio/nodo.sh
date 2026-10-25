#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
set -e
PROG="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$PROG"
export DOC_MISSIONS_DIR="${DOC_MISSIONS_DIR:-$PROG/output/missioni_laboratorio}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
if [ -f .env ]; then set -a; . ./.env; set +a; fi
if [ ! -f architetture/laboratorio/ros2_ws/install/local_setup.bash ]; then
  echo "Compila prima il laboratorio: pixi run build" >&2
  exit 1
fi
. architetture/laboratorio/ros2_ws/install/local_setup.bash
exec "$@"
