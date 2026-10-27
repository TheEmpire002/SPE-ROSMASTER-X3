#!/bin/bash
# Wie ~/run_docker.sh von Yahboom, aber mit /dev "live" im Container.
#
# Problem beim Original: --device=/dev/rplidar friert beim Container-Start ein,
# welcher ttyUSB-Port gemeint ist. Der Lidar meldet sich nach jedem Stopp am USB
# neu an und bekommt dabei oft eine andere Nummer -> im Container TIMEOUT.
# Mit -v /dev:/dev zeigt /dev/rplidar im Container immer auf den aktuellen Port.
#
# Auf dem Pi (nicht im Container) starten:  bash ~/run_docker_x3.sh
# Zum Fahren reicht  bash Marlon/x3.sh  - das startet den Container selbst.
#
# Laeuft der Container "x3" von x3.sh schon, oeffnet das hier nur ein weiteres
# Terminal darin. Ein zweiter Container wuerde Treiber und Lidar stoeren.
if [ "$(docker inspect -f '{{.State.Running}}' x3 2>/dev/null)" = "true" ]; then
    exec docker exec -it x3 bash
fi
xhost + >/dev/null 2>&1
docker run -it --rm \
-m 4g \
--memory-swap 4g \
--net=host \
--ipc=host \
--privileged \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--security-opt apparmor:unconfined \
-v /home/pi/temp:/root/temp \
-v /home/pi/ros_bashrc:/root/.bashrc \
-v /dev:/dev \
yahboomtechnology/ros-foxy:4.0.8 /bin/bash
