#!/usr/bin/env bash
# Holt Orbbecs OpenNI2-Bibliothek für die Tiefenkamera (Astra) auf den Pi.
#
# Warum so: Das pip-Paket "openni" läuft unter Python 3.13 nicht, und einen
# arm64-Build gibt es nur im Yahboom-Image (Docker-Container mit ROS2). Von
# dort werden libOpenNI2.so und der Orbbec-Treiber nach ~/orbbec_openni2
# kopiert; sensors/tiefe.py lädt sie per ctypes (Pfad: TIEFE_OPENNI_PFAD).
#
# Aufruf auf dem Pi:  bash scripts/orbbec_einrichten.sh [Container]
set -euo pipefail

ZIEL="${HOME}/orbbec_openni2"
QUELLE="/root/yahboomcar_ros2_ws/software/library_ws/src/ros2_astra_camera/astra_camera/openni2_redist/arm64"

if [ -f "${ZIEL}/libOpenNI2.so" ] && [ -f "${ZIEL}/OpenNI2/Drivers/liborbbec.so" ]; then
    echo "Schon eingerichtet: ${ZIEL}"
    exit 0
fi

# Container: als Argument, sonst der erste (auch gestoppte) mit Yahboom-Image.
CONTAINER="${1:-}"
if [ -z "${CONTAINER}" ]; then
    CONTAINER="$(docker ps -a --format '{{.Names}} {{.Image}}' | awk 'tolower($2) ~ /yahboom/ {print $1; exit}')"
fi
if [ -z "${CONTAINER}" ]; then
    echo "Kein Yahboom-Container gefunden. Name angeben: bash $0 <container>" >&2
    docker ps -a --format '  {{.Names}}  ({{.Image}})' >&2
    exit 1
fi

echo "Kopiere OpenNI2 aus Container ${CONTAINER} nach ${ZIEL} ..."
mkdir -p "${ZIEL}"
docker cp "${CONTAINER}:${QUELLE}/." "${ZIEL}/"

if [ ! -f "${ZIEL}/libOpenNI2.so" ]; then
    echo "libOpenNI2.so fehlt nach dem Kopieren -- Pfad im Container prüfen: ${QUELLE}" >&2
    exit 1
fi

# Ohne passende udev-Regel öffnet nur root die Kamera.
if ! ls /etc/udev/rules.d/ 2>/dev/null | grep -qi orbbec; then
    echo "Hinweis: keine Orbbec-udev-Regel in /etc/udev/rules.d -- die Kamera"
    echo "lässt sich dann nur als root öffnen (56-orbbec-usb.rules aus dem Container)."
fi

echo "Fertig. Server neu starten, dann: curl -X POST localhost:8000/tiefe/test"
