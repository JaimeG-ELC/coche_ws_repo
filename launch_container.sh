docker run -it --privileged --net=host \
    -v /var/run/dbus/:/var/run/dbus \
    -v /dev:/dev \
    -v /etc/udev/rules.d:/etc/udev/rules.d \
    --device /dev/sensors/vesc:/dev/sensors/vesc \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume ~/.Xauthority:/root/.Xauthority \
    --volume=./src/f1tenth_stack:/root/coche_ws/src/f1tenth_stack  \
    --volume=./src/trayectoria_pkg:/root/coche_ws/src/trayectoria_pkg \
    --volume=./src/gap_follow_pkg:/root/coche_ws/src/gap_follow_pkg \
    --volume=./src/pure_pursuit_pkg:/root/coche_ws/src/pure_pursuit_pkg \
    --volume=./src/mpc_pkg:/root/coche_ws/src/mpc_pkg \
    --volume=./src/waypoint_generator_pkg:/root/coche_ws/src/waypoint_generator_pkg \
    f1tenth-system
