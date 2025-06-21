#!/bin/bash

# --- CONFIGURACIÓN ---
WORKSPACE_PATH="/root/f1tenth_ws"

# Lista de launch files en formato paquete archivo.launch.py
LAUNCH_FILES=(
    "f1tenth_stack all.launch.py"
    "pure_pursuit_pkg pure_pursuit.launch.py"
    "reactive_follower_pkg reactive_follower.launch.py"
)

# --- INICIALIZACIÓN ROS 2 ---
source /opt/ros/foxy/setup.bash
source "$WORKSPACE_PATH/install/setup.bash"

# --- LANZAMIENTO NO BLOQUEANTE ---
PIDS=()

for launch_cmd in "${LAUNCH_FILES[@]}"; do
    pkg=$(echo $launch_cmd | awk '{print $1}')
    file=$(echo $launch_cmd | awk '{print $2}')
    
    echo "Launching $pkg $file"[]

    # lanza en segundo plano y guarda PID
    ros2 launch "$pkg" "$file" &
    PIDS+=($!)
done

# --- MANEJO DE SEÑALES ---
# Esto permite matar todos si se hace Ctrl+C o el contenedor se detiene
trap "echo 'Killing all launched nodes...'; kill ${PIDS[@]}; wait; exit" SIGINT SIGTERM

# --- ESPERA ACTIVA ---
# Evita que el contenedor termine
wait
