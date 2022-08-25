FROM osrf/ros:noetic-desktop-full

# Setting Catkin Workspace
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/noetic/setup.bash &&\
    mkdir -p ~/catkin_ws/src &&\
    cd ~/catkin_ws/src &&\
    catkin_init_workspace &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# Install git
RUN /bin/bash -c "apt-get install -y git"

# Install Python pip
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    apt-get install -y python3-pip"

# Install turtlebot3 simulator packages
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone https://github.com/cliansang/uwb-tracking-ros.git &&\
    cd ~/catkin_ws &&\
    rosdep install --from-paths src --ignore-src -r -y &&\
    catkin_make &&\
    source devel/setup.bash"

# Install Python Dependencies
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    pip install pyserial &&\
    pip install numpy"