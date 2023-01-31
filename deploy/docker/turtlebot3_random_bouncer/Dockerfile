FROM osrf/ros:humble-desktop-full

# Change the default shell to bash
SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3-pip 

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install setuptools==58.2.0

WORKDIR /opt/ros2_ws

COPY assignment-2/turtlebot3_random_bouncer /opt/ros2_ws/src/turtlebot3_random_bouncer
COPY assignment-2/deploy/docker/turtlebot3_random_bouncer/entrypoint.sh /entrypoint.sh 

RUN source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "turtlebot3_random_bouncer", "turtlebot3_random_bouncer.launch.py"]