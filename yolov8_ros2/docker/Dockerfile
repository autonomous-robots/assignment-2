FROM osrf/ros:humble-desktop-full

SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3-pip 

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
    setuptools==58.2.0 \
    onnxruntime

WORKDIR /opt/ros2_ws

COPY assignment-2/yolov8_ros2 /opt/ros2_ws/src/yolov8_ros2
COPY assignment-2/yolov8_ros2/docker/entrypoint.sh /entrypoint.sh 

RUN source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "yolov8_ros2", "person_detector.launch.py"]