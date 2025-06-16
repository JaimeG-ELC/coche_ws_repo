docker run -it --privileged --net=host \
    --gpus all \
    -v /var/run/dbus/:/var/run/dbus \
    -v /dev:/dev \
    -v /etc/udev/rules.d:/etc/udev/rules.d \
    --device /dev/sensors/vesc:/dev/sensors/vesc \
    --device /dev/sensors/joystick:/dev/sensors/joystick \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=9 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume ~/.Xauthority:/root/.Xauthority \
    --volume=./src/f1tenth_stack:/root/coche_ws/src/f1tenth_stack  \
    --volume=./src/pure_pursuit_pkg:/root/coche_ws/src/pure_pursuit_pkg \
    --volume=./src/waypoint_generator_pkg:/root/coche_ws/src/waypoint_generator_pkg \
    --volume=./src/manual_control_pkg:/root/coche_ws/src/manual_control_pkg \
    f1tenth-system
