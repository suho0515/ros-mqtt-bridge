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

# Install Python3 pip
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    apt-get install -y python3-pip"

# Install Dependent Packages for turtlebot3
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
    ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
    ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
    ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers"

# Install turtlebot3 packages
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    apt install -y ros-noetic-dynamixel-sdk &&\
    apt install -y ros-noetic-turtlebot3-msgs &&\
    apt install -y ros-noetic-turtlebot3"

# Install turtlebot3 simulator packages
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone -b noetic-devel https://github.com/suho0515/turtlebot3_simulations.git &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# Install Python Package for Documentation
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    pip3 install Sphinx &&\
    pip3 install sphinx_rtd_theme &&\
    apt-get install -y ros-noetic-rosdoc-lite"