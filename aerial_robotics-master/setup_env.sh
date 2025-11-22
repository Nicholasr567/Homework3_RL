#!/bin/bash
echo "--- Caricamento Ambiente Robotics Lab ---"

# 1. Carica ROS 2 Base
source /opt/ros/humble/setup.bash

# 2. Carica il Workspace dell'Homework
if [ -f "/root/aerial_robotics-master/ros2_ws/install/setup.bash" ]; then
    source /root/aerial_robotics-master/ros2_ws/install/setup.bash
    echo "[OK] Workspace ROS 2 caricato."
else
    echo "[!] ATTENZIONE: Devi compilare il workspace (colcon build)."
fi

# 3. Configura Variabili PX4 e Gazebo
export PX4_SOURCE_DIR=/root/aerial_robotics-master/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/root/aerial_robotics-master/PX4-Autopilot/Tools/simulation/gz/models

# 4. Fix Grafico per VM
export LIBGL_ALWAYS_SOFTWARE=1

echo "[OK] Variabili d'ambiente configurate."
echo "--- Pronto per lavorare! ---"
