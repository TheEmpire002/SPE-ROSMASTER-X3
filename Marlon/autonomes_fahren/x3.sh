#!/usr/bin/env bash
# EIN Befehl fuer alles - auf dem Pi (NICHT im Container) im Repo-Ordner:
#
#   bash Marlon/autonomes_fahren/x3.sh                    # Container + Treiber + Lidar + losfahren
#   bash Marlon/autonomes_fahren/x3.sh --calibrate        # Lidar-Ausrichtung pruefen (faehrt nicht)
#   bash Marlon/autonomes_fahren/x3.sh --max-v 0.2        # Parameter gehen an x3_explorer.py
#   bash Marlon/autonomes_fahren/x3.sh stop               # Roboter anhalten (z.B. aus 2. Terminal)
#   bash Marlon/autonomes_fahren/x3.sh neustart [...]     # ROS-Container frisch starten, dann losfahren
#   bash Marlon/autonomes_fahren/x3.sh hochladen          # Protokolle committen und pushen
#   bash Marlon/autonomes_fahren/x3.sh shell              # Terminal im Container (ros2 topic echo ...)
#
# Stoppen beim Fahren: Ctrl+C
#
# Was passiert:
#   1. Yahboom-Werks-App (rosmaster_main.py) beenden - sie belegt die seriellen Ports
#   2. ROS-Container "x3" starten (falls er nicht schon laeuft), mit
#      -v /dev:/dev (Lidar-Port live), --ipc=host (ROS-Daten zwischen Terminals)
#      und diesem Repo unter demselben Pfad
#   3. darin start_autonomy.sh ausfuehren
# Protokolle: Marlon/autonomes_fahren/protokoll/*.txt

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(git -C "$HERE" rev-parse --show-toplevel 2>/dev/null || (cd "$HERE/../.." && pwd))"
NAME="x3"
IMAGE="${X3_IMAGE:-yahboomtechnology/ros-foxy:4.0.8}"
PIDFILE="/tmp/x3_start.pid"            # im Container, schreibt start_autonomy.sh

if [ -f /.dockerenv ]; then
    echo "FEHLER: x3.sh auf dem Pi starten, nicht im Container." >&2
    echo "        Im Container direkt:  bash $HERE/start_autonomy.sh" >&2
    exit 1
fi

# ---- Protokolle hochladen (braucht kein Docker) ------------------------------
if [ "${1:-}" = "hochladen" ] || [ "${1:-}" = "upload" ]; then
    cd "$REPO" || exit 1
    if [ -z "$(ls -A "$HERE/protokoll" 2>/dev/null)" ]; then
        echo "Keine Protokolle in $HERE/protokoll - erst einmal fahren." >&2
        exit 1
    fi
    git add -- Marlon/autonomes_fahren/protokoll || exit 1
    if git diff --cached --quiet -- Marlon/autonomes_fahren/protokoll; then
        echo "[x3] Protokolle unveraendert, nichts zu committen"
    else
        git commit -m "Protokoll vom Roboter $(date '+%Y-%m-%d %H:%M')" -- Marlon/autonomes_fahren/protokoll || exit 1
    fi
    git pull --rebase --autostash || { echo "FEHLER: git pull fehlgeschlagen" >&2; exit 1; }
    git push && echo "[x3] Protokolle hochgeladen"
    exit $?
fi

# ---- Docker ------------------------------------------------------------------
DOCKER="docker"
if ! docker ps >/dev/null 2>&1; then
    if sudo -n docker ps >/dev/null 2>&1 || sudo docker ps >/dev/null; then
        DOCKER="sudo docker"
    else
        echo "FEHLER: Docker nicht erreichbar." >&2
        exit 1
    fi
fi
running() { [ "$($DOCKER inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" = "true" ]; }

stop_autonomy() {
    running || return 0
    $DOCKER exec "$NAME" bash -c '
        p=$(cat '"$PIDFILE"' 2>/dev/null) || exit 0
        kill -0 "$p" 2>/dev/null || exit 0
        kill -INT "$p"
        for i in $(seq 1 40); do kill -0 "$p" 2>/dev/null || exit 0; sleep 0.5; done
        echo "[x3] start_autonomy.sh reagiert nicht - beende hart" >&2
        kill -KILL "$p"; pkill -KILL -f x3_explorer.py; pkill -INT -f sllidar_node
    '
}

case "${1:-}" in
    stop)
        stop_autonomy
        echo "[x3] gestoppt"
        exit 0 ;;
    shell)
        running || { echo "Container $NAME laeuft nicht. Erst: bash Marlon/autonomes_fahren/x3.sh" >&2; exit 1; }
        exec $DOCKER exec -it "$NAME" bash ;;
    neustart|restart)
        shift
        stop_autonomy
        echo "[x3] beende ROS-Container ..."
        $DOCKER rm -f "$NAME" >/dev/null 2>&1
        others=$($DOCKER ps -q --filter "ancestor=$IMAGE")
        [ -n "$others" ] && $DOCKER stop $others >/dev/null ;;
esac

# ---- 1. Yahboom-Werks-App beenden ---------------------------------------------
# Startet beim Booten per ~/.config/autostart und haelt Motorplatine + Lidar offen.
for p in rosmaster_main.py start_app.sh; do
    if pgrep -f "$p" >/dev/null; then
        echo "[x3] beende Yahboom-App ($p) - belegt die seriellen Ports"
        pkill -f "$p"
        sudo -n pkill -f "$p" 2>/dev/null
    fi
done

# ---- 2. Container -----------------------------------------------------------
# Andere ROS-Container (z.B. ~/run_docker.sh) ohne --ipc=host: Topics sind
# sichtbar, aber es kommen keine Daten an -> Roboter wuerde nicht fahren.
others=$($DOCKER ps -q --filter "ancestor=$IMAGE" | while read -r id; do
    [ "$($DOCKER inspect -f '{{.Name}}' "$id")" = "/$NAME" ] || echo "$id"
done)
if [ -n "$others" ]; then
    echo "FEHLER: Es laufen noch andere ROS-Container: $(echo $others)" >&2
    echo "        (z.B. aus ~/run_docker.sh). Sie stoeren Treiber und Lidar." >&2
    echo "        Alles beenden und neu starten:  bash Marlon/autonomes_fahren/x3.sh neustart" >&2
    exit 1
fi

if running && ! $DOCKER exec "$NAME" test -f "$HERE/start_autonomy.sh"; then
    echo "FEHLER: Container $NAME sieht dieses Repo nicht ($REPO)." >&2
    echo "        bash Marlon/autonomes_fahren/x3.sh neustart" >&2
    exit 1
fi

if ! running; then
    echo "[x3] starte ROS-Container $NAME ($IMAGE)"
    $DOCKER rm -f "$NAME" >/dev/null 2>&1          # gestoppter Rest vom letzten Mal
    MOUNTS=(-v /dev:/dev -v "$REPO:$REPO")
    [ -d /tmp/.X11-unix ] && MOUNTS+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
    [ -d "$HOME/temp" ] && MOUNTS+=(-v "$HOME/temp:/root/temp")
    # Nur einhaengen, wenn es die Datei gibt - sonst legt Docker einen Ordner an
    [ -f "$HOME/ros_bashrc" ] && MOUNTS+=(-v "$HOME/ros_bashrc:/root/.bashrc")
    xhost + >/dev/null 2>&1
    # --init: "sleep infinity" als PID 1 raeumt beendete Prozesse nicht ab. Die
    # Zombies (sllidar_node, Mcnamu_driver_X) findet pgrep sonst noch als "laufend".
    if ! $DOCKER run -d --rm --init --name "$NAME" \
            -m 4g --memory-swap 4g \
            --net=host --ipc=host --privileged \
            --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
            --security-opt apparmor:unconfined \
            "${MOUNTS[@]}" \
            "$IMAGE" sleep infinity >/dev/null; then
        echo "FEHLER: Container startet nicht." >&2
        exit 1
    fi
    sleep 2
    running || { echo "FEHLER: Container $NAME hat sich sofort beendet." >&2; exit 1; }
fi

# ---- 3. Autonomie -----------------------------------------------------------
# Ohne Terminal (oder wenn die SSH-Verbindung abreisst) bekommt der Prozess im
# Container das Signal nicht mit -> hier abfangen und den Stopp weiterreichen.
trap 'echo; stop_autonomy; exit 130' INT TERM HUP
TTY=""
[ -t 0 ] && [ -t 1 ] && TTY="-t"
$DOCKER exec -i $TTY "$NAME" bash "$HERE/start_autonomy.sh" "$@"
rc=$?
echo
echo "[x3] Protokolle: $HERE/protokoll"
exit "$rc"
