
FROM osrf/ros:melodic-desktop-full

# Basic
# ========================================
# Setting Catkin Workspace
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/melodic/setup.bash &&\
    mkdir -p ~/catkin_ws/src &&\
    cd ~/catkin_ws/src &&\
    catkin_init_workspace &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# Install git
RUN /bin/bash -c "apt-get install -y git"

# Install Python pip
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    apt-get install -y python-pip"

# Install ros bridge
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    apt-get install -y ros-melodic-rosbridge-suite &&\
    source /opt/ros/melodic/setup.bash"