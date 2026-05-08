#!/usr/bin/env bash
# Convenience wrapper around the Quatro ROS Noetic Docker image.
#
#   ./docker/run.sh build-image  # build the Docker image once
#   ./docker/run.sh build-pkg    # catkin build quatro inside the container
#   ./docker/run.sh test         # build + run the headless example, print transform
#   ./docker/run.sh shell        # open an interactive shell inside the container
#
# Repo source is bind-mounted at /ws/src/quatro. catkin's build/devel/logs
# directories live in named volumes so they survive across runs.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

IMAGE_TAG="${QUATRO_IMAGE:-quatro-dev:noetic}"
BUILD_VOL="${QUATRO_BUILD_VOL:-quatro_build}"
DEVEL_VOL="${QUATRO_DEVEL_VOL:-quatro_devel}"
LOGS_VOL="${QUATRO_LOGS_VOL:-quatro_logs}"
TEST_TIMEOUT="${QUATRO_TEST_TIMEOUT:-60}"

run_docker() {
    docker run --rm "$@" \
        -v "$REPO_ROOT:/ws/src/quatro:rw" \
        -v "$BUILD_VOL:/ws/build" \
        -v "$DEVEL_VOL:/ws/devel" \
        -v "$LOGS_VOL:/ws/logs" \
        "$IMAGE_TAG"
}

cmd="${1:-help}"
shift || true

case "$cmd" in
    build-image)
        docker build -t "$IMAGE_TAG" "$SCRIPT_DIR"
        ;;
    shell)
        run_docker -it bash
        ;;
    build-pkg)
        run_docker bash -lc "
            source /opt/ros/noetic/setup.bash
            cd /ws
            catkin build quatro $*
        "
        ;;
    test)
        run_docker bash -lc "
            set -e
            source /opt/ros/noetic/setup.bash
            cd /ws
            catkin build quatro
            source devel/setup.bash
            timeout --signal=INT --preserve-status ${TEST_TIMEOUT} \
                roslaunch quatro quatro_headless.launch || true
        "
        ;;
    clean)
        docker volume rm "$BUILD_VOL" "$DEVEL_VOL" "$LOGS_VOL" 2>/dev/null || true
        ;;
    help|*)
        sed -n '2,12p' "$0"
        ;;
esac
