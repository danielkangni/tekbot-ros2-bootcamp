#!/usr/bin/env bash
# Usage:
#   ./docker/run.sh build        # construit l'image (une seule fois)
#   ./docker/run.sh colcon       # compile les packages du test dans le conteneur
#   ./docker/run.sh [commande]   # shell (par defaut) ou commande, ex: ros2 launch maze_solving tekbot_maze.launch.py
#   ./docker/run.sh exec         # ouvre un 2e terminal dans le conteneur deja lance
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$(dirname "$HERE")"
OUT="$HOME/tekbot_ws_humble"
ENGINE=${ENGINE:-podman}   # podman (disque principal) ou docker
IMG=tekbot-humble
NAME=tekbot-humble-run
mkdir -p "$OUT"

case "$1" in
  build)
    $ENGINE build -t $IMG "$HERE"; exit ;;
  exec)
    exec $ENGINE exec -it -e LIBGL_ALWAYS_SOFTWARE=${SOFTWARE_GL:-0} $NAME /entrypoint.sh bash ;;
  colcon)
    if $ENGINE ps --format '{{.Names}}' | grep -qx $NAME; then   # conteneur deja lance : on compile dedans
      exec $ENGINE exec $NAME /entrypoint.sh bash -c 'cd /ws && colcon --log-base /ws/out/log build --symlink-install --build-base /ws/out/build --install-base /ws/out/install --packages-select tekbot_description tekbot_control maze_solving tekbot_pathfinding'
    fi
    set -- bash -c 'cd /ws && colcon --log-base /ws/out/log build --symlink-install \
        --build-base /ws/out/build --install-base /ws/out/install \
        --packages-select tekbot_description tekbot_control maze_solving tekbot_pathfinding' ;;
  "") set -- bash ;;
esac

TTY=-i; [ -t 0 ] && TTY=-it
xhost +local:docker >/dev/null 2>&1 || true
GPU_ARGS=(--device /dev/dri)
# Rendu materiel (GPU Intel) par defaut ; SOFTWARE_GL=1 force le rendu CPU (plante Gazebo : 700% CPU)
RENDER_ENV=(-e LIBGL_ALWAYS_SOFTWARE=${SOFTWARE_GL:-0})

USER_ARGS=(-u "$(id -u):$(id -g)")
[ "$ENGINE" = podman ] && USER_ARGS=(--userns=keep-id)

exec $ENGINE run --rm $TTY --name $NAME --net host \
    "${USER_ARGS[@]}" -e HOME=/tmp/home \
    -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix --security-opt label=disable \
    "${GPU_ARGS[@]}" "${RENDER_ENV[@]}" \
    -v "$SRC":/ws/src:Z -v "$OUT":/ws/out:Z -w /ws \
    $IMG "$@"
