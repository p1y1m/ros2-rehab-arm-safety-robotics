#!/usr/bin/env bash

set -eo pipefail

readonly CONTAINER_NAME="medical-robotics"
readonly HOST_PROJECT="/home/pedroeyanezmel531/medical robotics"
readonly CONTAINER_PROJECT="/workspace"

if [[ ! -d "${HOST_PROJECT}" ]]; then
  echo "Project directory not found: ${HOST_PROJECT}" >&2
  exit 1
fi

if ! docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  echo "Docker container not found: ${CONTAINER_NAME}" >&2
  exit 1
fi

if [[ "$(docker inspect --format '{{.State.Running}}' "${CONTAINER_NAME}")" != "true" ]]; then
  docker start "${CONTAINER_NAME}"
fi

docker exec "${CONTAINER_NAME}" mkdir -p "${CONTAINER_PROJECT}"
docker cp "${HOST_PROJECT}/." "${CONTAINER_NAME}:${CONTAINER_PROJECT}/"

docker exec -it "${CONTAINER_NAME}" bash -lc '
  set -eo pipefail
  cd /workspace
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select rehab_arm_sim --symlink-install
  source install/setup.bash
  exec ros2 launch rehab_arm_sim session.launch.py
'
