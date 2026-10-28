#!/usr/bin/env bash
# Startet alles fuer autonomes Fahren auf dem ROSMASTER X3 - IM Docker-Container.
# Normalerweise nicht direkt aufrufen, sondern auf dem Pi:  bash Marlon/x3.sh
#
#   ./start_autonomy.sh                  # losfahren
#   ./start_autonomy.sh --calibrate      # Lidar-Ausrichtung pruefen (faehrt nicht)
#   ./start_autonomy.sh --max-v 0.2      # alle Parameter gehen an x3_explorer.py
#
# Stoppen: Ctrl+C  (Roboter wird angehalten, alle Prozesse beendet)
#
# Ablauf wie beim Kalibrieren (da lief der Treiber schon, der Lidar kam danach):
#   1. Treiber starten (nur falls noch keiner laeuft), 8 s warten. Stirbt der
#      Treiberknoten beim Start (USB-Hub meldet sich neu an, /dev/myserial fehlt
#      kurz), wird er einzeln neu gestartet - und danach bei jedem Absturz wieder.
#   2. Lidar per "ros2 run sllidar_ros2 sllidar_node" an /dev/rplidar starten
#   3. x3_explorer.py starten
# Liefert der Lidar waehrend der Fahrt nichts mehr, wird er automatisch neu gestartet
# (der Roboter steht so lange still).
#
# Protokolle landen in Marlon/protokoll/ (im Git, damit man sie hochladen kann).

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Nur eine Instanz. x3.sh stop / Ctrl+C ohne Terminal schicken SIGINT an diese PID.
PIDFILE="/tmp/x3_start.pid"
old=$(cat "$PIDFILE" 2>/dev/null)
if [ -n "$old" ] && [ "$old" != "$$" ] && grep -q start_autonomy "/proc/$old/cmdline" 2>/dev/null; then
    echo "FEHLER: start_autonomy.sh laeuft schon (PID $old). Stoppen:  bash Marlon/x3.sh stop" >&2
    exit 1
fi
echo $$ > "$PIDFILE"

LOGDIR="$HERE/protokoll"
mkdir -p "$LOGDIR"
START_LOG="$LOGDIR/start.txt"
BRINGUP_LOG="$LOGDIR/bringup.txt"
LIDAR_LOG="$LOGDIR/lidar.txt"
DRIVER_LOG="$LOGDIR/treiber.txt"
DIAG_LOG="$LOGDIR/diagnose.txt"
: > "$BRINGUP_LOG"; : > "$LIDAR_LOG"; : > "$DRIVER_LOG"; : > "$DIAG_LOG"

# Alle Ausgaben zusaetzlich nach start.txt. tee ignoriert Ctrl+C, sonst bricht
# die Ausgabe (und mit ihr das Skript) mitten im Anhalten ab.
exec > >(trap '' INT TERM HUP; exec tee "$START_LOG") 2>&1
echo "[start] $(date '+%Y-%m-%d %H:%M:%S')  Argumente: $*"
export PYTHONDONTWRITEBYTECODE=1       # kein root-eigenes __pycache__ im Repo
# Treiber-Fehler sofort ins Log schreiben (sonst erst beim Beenden), damit der
# Treiber-Waechter einen haengenden Treiberknoten bemerkt
export PYTHONUNBUFFERED=1

# ---- ROS-Umgebung -----------------------------------------------------------
if [ -z "${ROS_DISTRO:-}" ]; then
    for d in humble foxy jazzy galactic noetic melodic; do
        [ -f "/opt/ros/$d/setup.bash" ] && { source "/opt/ros/$d/setup.bash"; break; }
    done
fi
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "FEHLER: Keine ROS-Installation gefunden (/opt/ros/...)." >&2
    echo "        Laeuft ROS bei euch in Docker? Dann auf dem Pi  bash Marlon/x3.sh  benutzen." >&2
    exit 1
fi
for ws in "$HOME/yahboomcar_ros2_ws/software/library_ws/install" \
          "$HOME/yahboomcar_ros2_ws/yahboomcar_ws/install" "$HOME/yahboomcar_ws/install" \
          "$HOME/yahboomcar_ws/devel" "$HOME/catkin_ws/devel"; do
    [ -f "$ws/setup.bash" ] && source "$ws/setup.bash"
done
# "docker exec" liest ~/.bashrc nicht. Setzt die Yahboom-bashrc eine
# ROS_DOMAIN_ID, dieselbe nehmen - sonst sehen andere Terminals nichts.
if [ -z "${ROS_DOMAIN_ID:-}" ] && [ -f "$HOME/.bashrc" ]; then
    id=$(sed -n 's/^[[:space:]]*export[[:space:]]\+ROS_DOMAIN_ID=["'\'']\?\([0-9]\+\).*/\1/p' "$HOME/.bashrc" | tail -n 1)
    [ -n "$id" ] && export ROS_DOMAIN_ID="$id"
fi
echo "[start] ROS $ROS_VERSION ($ROS_DISTRO)  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

if [ "$ROS_VERSION" = "2" ]; then
    BRINGUP_LAUNCH="${BRINGUP_LAUNCH:-yahboomcar_bringup yahboomcar_bringup_X3_launch.py}"
    # derselbe Befehl, mit dem das Kalibrieren funktioniert hat
    LIDAR_CMD="${LIDAR_CMD:-ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/rplidar -p serial_baudrate:=115200 -p frame_id:=laser -p angle_compensate:=true}"
    # nur der Motor-Knoten aus dem bringup, fuer den Neustart nach USB-Aussetzern
    DRIVER_CMD="${DRIVER_CMD:-ros2 run yahboomcar_bringup Mcnamu_driver_X3}"
    LAUNCH="ros2 launch"
    topics() { timeout 10 ros2 topic list 2>/dev/null; }
    # ein paar Sekunden lang Stopp senden statt einmal - eine einzelne Nachricht geht beim
    # Verbindungsaufbau gern verloren.
    stop_robot() { timeout 4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1; }
else
    BRINGUP_LAUNCH="${BRINGUP_LAUNCH:-yahboomcar_bringup bringup.launch}"
    LIDAR_CMD="${LIDAR_CMD:-roslaunch rplidar_ros rplidar.launch}"
    LAUNCH="roslaunch"
    topics() { rostopic list 2>/dev/null; }
    stop_robot() { timeout 3 rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}" >/dev/null 2>&1; }
    if ! rostopic list >/dev/null 2>&1; then
        echo "[start] starte roscore"
        roscore >"$LOGDIR/roscore.txt" 2>&1 &
        PIDS="$!"
        sleep 3
    fi
fi

[ -n "${LIDAR_LAUNCH:-}" ] && LIDAR_CMD="$LAUNCH $LIDAR_LAUNCH"
PIDS="${PIDS:-}"
EXPLORER=""
LIDAR_OWN=""
DRIVER_OWN=""
STOPFILE="/tmp/x3_stop.$$"
# Vom Treiber-Waechter (Hintergrund-Subshell) gesetzt, wenn er den Treiberknoten
# uebernommen hat - eine Variable kaeme in cleanup() nicht an.
DRIVER_OWN_FILE="/tmp/x3_driver_own.$$"
rm -f "$STOPFILE" "$DRIVER_OWN_FILE"

# Momentaufnahme fuer die Fehlersuche (wird mit hochgeladen)
diagnose() {
    {
        echo "==== $(date '+%Y-%m-%d %H:%M:%S')  $1"
        echo "-- Geraete"; ls -l /dev/rplidar /dev/myserial /dev/ttyUSB* /dev/ttyACM* 2>&1
        echo "-- Prozesse"; ps -eo pid,ppid,etime,args 2>/dev/null | grep -E "ros2|sllidar|yahboom|Mcnamu|x3_explorer|python" | grep -v grep
        if [ "$ROS_VERSION" = "2" ]; then
            echo "-- Topics"; topics
            for t in /scan /cmd_vel /RGBLight; do echo "-- ros2 topic info $t"; timeout 5 ros2 topic info "$t" 2>&1; done
        fi
        echo
    } >>"$DIAG_LOG" 2>&1
}

# Dateien gehoeren sonst root und der Pi-Benutzer kann sie nicht committen/loeschen
fix_owner() {
    chown -R --reference="$HERE" "$LOGDIR" 2>/dev/null
}

cleanup() {
    rc="${1:-0}"
    trap '' INT TERM HUP       # Anhalten nicht durch zweites Ctrl+C abbrechen
    echo; echo "[start] stoppe Roboter ..."
    touch "$STOPFILE"          # Lidar-Schleife nicht mehr neu starten
    # Zuerst den Explorer beenden, sonst ueberschreibt er den Stopp-Befehl
    if [ -n "$EXPLORER" ] && kill -0 "$EXPLORER" 2>/dev/null; then
        kill -TERM "$EXPLORER" 2>/dev/null
        for i in 1 2 3 4 5 6 7 8 9 10; do kill -0 "$EXPLORER" 2>/dev/null || break; sleep 0.3; done
        kill -KILL "$EXPLORER" 2>/dev/null
    fi
    stop_robot
    [ "$rc" != 0 ] && diagnose "Fehler (Code $rc)"
    for p in $PIDS; do pkill -INT -P "$p" 2>/dev/null; kill -INT "$p" 2>/dev/null; done
    sleep 1
    for p in $PIDS; do pkill -P "$p" 2>/dev/null; kill "$p" 2>/dev/null; done
    # "ros2 run" startet sllidar_node als Enkelprozess -> sonst bleibt er als
    # Waise am Port haengen und der naechste Start bekommt TIMEOUT.
    if [ -n "$LIDAR_OWN" ]; then
        pkill -INT -f sllidar_node 2>/dev/null; sleep 1; pkill -KILL -f sllidar_node 2>/dev/null
    fi
    # dasselbe fuer den einzeln gestarteten Treiberknoten
    if [ -n "$DRIVER_OWN" ] || [ -e "$DRIVER_OWN_FILE" ]; then
        pkill -INT -f Mcnamu_driver_X3 2>/dev/null; sleep 1; pkill -KILL -f Mcnamu_driver_X3 2>/dev/null
    fi
    rm -f "$STOPFILE" "$PIDFILE" "$DRIVER_OWN_FILE"
    echo "[start] beendet (Code $rc). Protokolle: $LOGDIR"
    fix_owner
    exit "$rc"
}
trap 'cleanup 130' INT
trap 'cleanup 143' TERM HUP

# Mit dem Original ~/run_docker.sh sind /dev/rplidar und /dev/myserial beim
# Container-Start eingefroren. Zeigen beide auf denselben Port -> Lidar-Timeout.
if [ -e /dev/rplidar ] && [ -e /dev/myserial ] && \
   [ "$(stat -Lc '%t:%T' /dev/rplidar)" = "$(stat -Lc '%t:%T' /dev/myserial)" ]; then
    echo "FEHLER: /dev/rplidar und /dev/myserial sind dasselbe Geraet." >&2
    echo "        Container neu starten:  bash Marlon/x3.sh neustart" >&2
    cleanup 1
fi
for d in /dev/rplidar /dev/myserial; do
    [ -e "$d" ] || echo "WARNUNG: $d fehlt (USB-Kabel? udev-Regel?)" >&2
done

# Die Web-App (main.py, Port 8000) benutzt dieselben seriellen Ports wie Treiber
# und Lidar. Beide gleichzeitig -> Lidar-Timeout, Motoren reagieren auf beide.
if (exec 3<>/dev/tcp/127.0.0.1/8000) 2>/dev/null; then
    echo "FEHLER: Auf Port 8000 laeuft ein Server (Web-App main.py?)." >&2
    echo "        Er belegt Lidar und Motorplatine. Erst beenden, dann neu starten." >&2
    cleanup 1
fi

# Explorer vom letzten Lauf (z.B. Terminal geschlossen) faehrt sonst mit
if pgrep -f "x3_explorer.py" >/dev/null; then
    echo "[start] beende alten x3_explorer.py"
    pkill -INT -f "x3_explorer.py"; sleep 2; pkill -KILL -f "x3_explorer.py"
fi

# Wartet bis zu $2 Sekunden, bis Topic $1 sichtbar ist. Der erste
# "ros2 topic list" nach Container-Start ist oft noch leer (ROS-Daemon startet
# erst), deshalb nicht nach einem einzigen Versuch aufgeben.
wait_topic() {
    for i in $(seq 1 "$2"); do
        topics | grep -qx "$1" && return 0
        sleep 1
    done
    return 1
}

# Echter Sender auf Topic $1? "ros2 topic list" zeigt ein Topic auch dann noch,
# wenn nur ein Empfaenger (z.B. ein alter Explorer) dranhaengt oder der Daemon
# veraltete Eintraege hat -> dann wuerde der Lidar nie gestartet.
has_pub() {
    timeout 5 ros2 topic info "$1" 2>/dev/null | grep -q "Publisher count: [1-9]"
}
# Echter Empfaenger auf Topic $1? (Treiber hoert auf /RGBLight)
has_sub() {
    timeout 5 ros2 topic info "$1" 2>/dev/null | grep -q "Subscription count: [1-9]"
}
driver_running() {
    if [ "$ROS_VERSION" = "2" ]; then has_sub /RGBLight
    else topics | grep -qx "/RGBLight"; fi
}

# Startet den Treiberknoten immer wieder neu, solange nicht gestoppt wird.
# Vorher muss /dev/myserial 3 s am Stueck da sein: beim Hub-Reset verschwindet
# das Board mehrmals kurz hintereinander, ein Start mittendrin scheitert sofort.
driver_loop() {
    try=1
    while [ ! -e "$STOPFILE" ]; do
        stable=0
        while [ "$stable" -lt 3 ] && [ ! -e "$STOPFILE" ]; do
            if [ -e /dev/myserial ]; then stable=$((stable + 1)); else stable=0; fi
            sleep 1
        done
        [ -e "$STOPFILE" ] && break
        echo "[treiber] Start $try  $(date '+%H:%M:%S')"
        $DRIVER_CMD
        try=$((try + 1))
        sleep 2
    done
}

# ---- 1. Treiber: nur starten, wenn wirklich keiner laeuft -------------------
# Zwei Treiber an derselben Motorplatine stoeren sich gegenseitig.
if wait_topic /cmd_vel 5 && driver_running; then
    echo "[start] Treiber laeuft schon - wird benutzt"
else
    # Ein bringup vom abgebrochenen letzten Lauf, dessen Treiberknoten tot ist:
    # beenden, sonst laufen base_node, ekf und joy danach doppelt.
    old_launch=$(pgrep -f "launch $BRINGUP_LAUNCH")
    if [ -n "$old_launch" ]; then
        echo "[start] beende alten bringup ohne Treiber (PID $(echo $old_launch))"
        for p in $old_launch; do pkill -INT -P "$p"; kill -INT "$p"; done 2>/dev/null
        sleep 3
        for p in $old_launch; do pkill -KILL -P "$p"; kill -KILL "$p"; done 2>/dev/null
        pkill -KILL -f Mcnamu_driver_X3 2>/dev/null
    fi
    echo "[start] starte Treiber: $LAUNCH $BRINGUP_LAUNCH  (Log: $BRINGUP_LOG)"
    $LAUNCH $BRINGUP_LAUNCH >"$BRINGUP_LOG" 2>&1 &
    PIDS="$PIDS $!"
    ok=""
    for i in $(seq 1 45); do
        driver_running && { ok=1; break; }
        # Beim bringup-Start meldet sich der USB-Hub neu an; findet der Treiberknoten
        # /dev/myserial in dem Moment nicht, beendet er sich und launch startet ihn
        # nicht neu. Dann selbst uebernehmen.
        if [ "$ROS_VERSION" = "2" ] && [ -z "$DRIVER_OWN" ] && [ "$i" -ge 8 ] &&            ! pgrep -f Mcnamu_driver_X3 >/dev/null; then
            echo "[start] Treiberknoten beendet (USB-Neuanmeldung?) - starte ihn einzeln neu  (Log: $DRIVER_LOG)"
            driver_loop >"$DRIVER_LOG" 2>&1 &
            PIDS="$PIDS $!"
            DRIVER_OWN=1
        fi
        sleep 1
    done
    if [ -z "$ok" ]; then
        echo "FEHLER: Treiber startet nicht. Siehe $BRINGUP_LOG" >&2
        tail -n 15 "$BRINGUP_LOG" >&2
        [ -n "$DRIVER_OWN" ] && tail -n 15 "$DRIVER_LOG" >&2
        cleanup 1
    fi
    # Beim Treiberstart meldet sich der USB-Hub kurz neu an (auch der Lidar).
    # Daher Lidar erst danach starten - wie beim Kalibrieren, als der Treiber schon lief.
    echo "[start] Treiber laeuft, warte 8 s bis USB ruhig ist"
    sleep 8
fi

# ---- 2. Lidar (genau wie beim Kalibrieren) ----------------------------------
# Laeuft als Schleife: beendet sich sllidar_node (Timeout, USB weg), wird er
# nach 2 s neu gestartet. /dev/rplidar zeigt dank -v /dev:/dev immer auf
# den aktuellen Port.
lidar_loop() {
    try=1
    while [ ! -e "$STOPFILE" ]; do
        echo "[lidar] Start $try  $(date '+%H:%M:%S')"
        $LIDAR_CMD
        try=$((try + 1))
        sleep 2
    done
}
if wait_topic /scan 3 && has_pub /scan; then
    echo "[start] Lidar laeuft schon"
else
    # Haengender sllidar_node ohne Sender (Waise vom letzten Lauf) blockiert den Port
    if pgrep -f sllidar_node >/dev/null; then
        echo "[start] beende alten sllidar_node (sendet nicht, blockiert Port)"
        pkill -INT -f sllidar_node; sleep 2; pkill -KILL -f sllidar_node
    fi
    echo "[start] starte Lidar:   $LIDAR_CMD  (Log: $LIDAR_LOG)"
    lidar_loop >"$LIDAR_LOG" 2>&1 &
    PIDS="$PIDS $!"
    LIDAR_OWN=1
fi
lidar_ready() {
    if [ -n "$LIDAR_OWN" ] && [ "$ROS_VERSION" = "2" ]; then
        grep -qi "current scan mode" "$LIDAR_LOG" 2>/dev/null
    elif [ "$ROS_VERSION" = "2" ]; then
        has_pub /scan
    else
        topics | grep -qx "/scan"
    fi
}

echo -n "[start] warte auf Lidar-Daten "
for i in $(seq 1 40); do
    lidar_ready && break
    echo -n "."; sleep 1
done
echo
if ! lidar_ready; then
    echo "FEHLER: Lidar liefert keine Daten. Siehe $LIDAR_LOG" >&2
    tail -n 15 "$LIDAR_LOG" >&2
    echo "        Bei TIMEOUT: USB-Kabel pruefen, dann  bash Marlon/x3.sh neustart" >&2
    cleanup 1
fi
echo "[start] Lidar OK"
diagnose "Start OK"

# ---- 3. Autonomie ---------------------------------------------------------
# Waechter: x3_explorer.py beruehrt /tmp/x3_scan_ok bei jedem Scan (max. 1x/s).
# Kommt laenger nichts, haengt sllidar_node (z.B. nach USB-Neuanmeldung) ->
# beenden, lidar_loop startet ihn neu.
watchdog() {
    sleep 15
    while [ ! -e "$STOPFILE" ]; do
        age=$(( $(date +%s) - $(stat -c %Y /tmp/x3_scan_ok 2>/dev/null || echo 0) ))
        if [ "$age" -gt 6 ]; then
            echo "[waechter] keine Lidar-Daten seit ${age} s -> Lidar neu starten"
            pkill -INT -f sllidar_node; sleep 2; pkill -KILL -f sllidar_node
            sleep 10
        fi
        sleep 2
    done
}
# Treiber-Waechter: Nach einem USB-Aussetzer waehrend der Fahrt laeuft der
# Treiberknoten weiter, schreibt aber an den toten Port ("set_car_motion error!")
# - der Roboter reagiert dann auf nichts mehr, auch nicht auf Stopp. Wachsen die
# Fehler, den Knoten beenden; driver_loop startet ihn neu, sobald /dev/myserial
# wieder stabil da ist. Lief der Treiber aus dem bringup, wird driver_loop hier
# erst gestartet (launch startet ihn nicht neu).
driver_watchdog() {
    log="$BRINGUP_LOG"
    [ -n "$DRIVER_OWN" ] && log="$DRIVER_LOG"
    last=$(grep -c "set_car_motion error" "$log" 2>/dev/null)
    while [ ! -e "$STOPFILE" ]; do
        sleep 2
        n=$(grep -c "set_car_motion error" "$log" 2>/dev/null)
        if [ "${n:-0}" -ge $(( ${last:-0} + 5 )) ]; then
            echo "[treiber-waechter] Treiber schreibt ins Leere (USB-Aussetzer?) -> neu starten"
            pkill -INT -f Mcnamu_driver_X3; sleep 1; pkill -KILL -f Mcnamu_driver_X3
            if [ "$log" != "$DRIVER_LOG" ]; then
                touch "$DRIVER_OWN_FILE"
                driver_loop >>"$DRIVER_LOG" 2>&1 &
                log="$DRIVER_LOG"
            fi
            sleep 15           # Neustart abwarten (3 s stabiler Port + Init)
        fi
        last=$(grep -c "set_car_motion error" "$log" 2>/dev/null)
    done
}

# Startzeitpunkt als "letzter Scan": vor dem ersten Scan zaehlt die Wartezeit ab
# hier, nicht ab 1970
touch /tmp/x3_scan_ok
echo "[start] Autonomie startet in 3 s - Abstand halten!"
sleep 3
python3 -u "$HERE/x3_explorer.py" "$@" &
EXPLORER=$!
if [ -n "$LIDAR_OWN" ] && [ "$ROS_VERSION" = "2" ]; then
    watchdog &
    PIDS="$PIDS $!"
fi
if [ "$ROS_VERSION" = "2" ]; then
    driver_watchdog &
    PIDS="$PIDS $!"
fi
wait "$EXPLORER"
rc=$?
EXPLORER=""
[ "$rc" != 0 ] && echo "FEHLER: x3_explorer.py beendet mit Code $rc" >&2
cleanup "$rc"
