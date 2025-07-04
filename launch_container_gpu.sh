docker run --runtime nvidia -it --privileged --net=host \
    -v /var/run/dbus/:/var/run/dbus \
    -v /dev:/dev \
    -v /etc/udev/rules.d:/etc/udev/rules.d \
    --device /dev/sensors/vesc:/dev/sensors/vesc \
    --device /dev/sensors/joystick:/dev/sensors/joystick \
    --device /dev/sensors/imu:/dev/sensors/imu \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=9 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume ~/.Xauthority:/root/.Xauthority \
    --volume=./src/f1tenth_stack:/root/coche_ws/src/f1tenth_stack  \
    --volume=./src/pure_pursuit_pkg:/root/coche_ws/src/pure_pursuit_pkg \
    --volume=./src/reactive_follower_pkg:/root/coche_ws/src/reactive_follower_pkg \
    --volume=./src/waypoint_generator_pkg:/root/coche_ws/src/waypoint_generator_pkg \
    --volume=./src/manual_control_pkg:/root/coche_ws/src/manual_control_pkg \
    --volume=./src/interfaces_pkg:/root/coche_ws/src/interfaces_pkg \
    --volume=./src/vesc:/root/coche_ws/src/vesc \
    f1tenth-system-gpu