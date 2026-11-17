#!/usr/bin/env bash
# Installs the USB diagnosis daemon as a systemd service (page on port 8001).
#
# Why a copy in /opt: the daemon must keep running while the server is
# stopped, updated or crashed, so it does not run from the git checkout.
# It never touches the server or /dev/myserial.
#
# On the Pi:  robot/deploy/usbdiag/install.sh [--uninstall]
set -euo pipefail

NAME="rosmaster-usbdiag"
TARGET="/opt/rosmaster-usbdiag"
UNIT="/etc/systemd/system/${NAME}.service"
MODULES="/etc/modules-load.d/usbmon.conf"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../../.." && pwd)"

if [ "${1:-}" = "--uninstall" ]; then
    sudo systemctl disable --now "${NAME}" || true
    sudo rm -f "${UNIT}"
    # the module stays loaded until the next boot, unloading it could break another capture
    sudo rm -f "${MODULES}"
    sudo rm -rf "${TARGET}"
    sudo systemctl daemon-reload
    echo "USB-Diagnose entfernt (Protokolle in /home/pi/.rosmaster/diagnose/usb bleiben)"
    exit 0
fi

# usbmon lets the daemon hear the board traffic; it loads here and at every boot, never from the daemon
echo usbmon | sudo tee "${MODULES}" > /dev/null
sudo modprobe usbmon

sudo rm -rf "${TARGET}/usbdiag"
sudo mkdir -p "${TARGET}"
sudo cp -r "${REPO}/robot/src/rosmaster_assist/usbdiag" "${TARGET}/"
sudo find "${TARGET}" -name __pycache__ -type d -prune -exec rm -rf {} +
sudo cp "${HERE}/${NAME}.service" "${UNIT}"
sudo systemctl daemon-reload
# restart, not enable --now: an already running daemon must pick up the new copy
sudo systemctl enable "${NAME}"
sudo systemctl restart "${NAME}"
echo "USB-Diagnose: http://$(hostname -I | awk '{print $1}'):8001/"
