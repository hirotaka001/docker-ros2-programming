docker run -itd \
    --net=host \
    --restart unless-stopped -v /home/hht/docker_workspace/ros2_ws:/root/ros2_ws -w /root humble/ubuntu:22.04
    # --env STARTUP_RUN="source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch telecontrol_server telecontrol_server.launch.py"
    # --entrypoint bash -c "source /opt/ros/humble/setup.bash && ros2 launch telecontrol_server telecontrol_server.launch.py"
    # /bin/bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch telecontrol_server telecontrol_server.launch.py"
echo "done"