FROM osrf/ros:foxy-desktop

# Setting Catkin Workspace
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash &&\
    mkdir -p ~/dev_ws/src &&\
    cd ~/dev_ws &&\
    colcon build &&\
    source /opt/ros/foxy/setup.bash &&\
    . install/local_setup.bash"

# Install ros bridge
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/foxy/setup.bash &&\
    apt-get install -y ros-foxy-rosbridge-suite &&\
    source /opt/ros/foxy/setup.bash"

