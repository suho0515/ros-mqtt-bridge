FROM osrf/ros:foxy-desktop

# Setting Catkin Workspace
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash &&\
    mkdir -p ~/dev_ws/src &&\
    cd ~/dev_ws &&\
    colcon build &&\
    source /opt/ros/foxy/setup.bash &&\
    . install/local_setup.bash"

# Install git
RUN /bin/bash -c "apt-get install -y git"

# # Install Python3 pip
# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash &&\
#     apt-get install -y python3-pip"

# Install Dependent Packages for turtlebot3
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/foxy/setup.bash &&\
    apt-get install -y ros-foxy-gazebo-* &&\
    apt install -y ros-foxy-cartographer \
    ros-foxy-cartographer-ros \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup"

# Install turtlebot3 packages
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash &&\
    source ~/.bashrc &&\
    apt install -y ros-foxy-dynamixel-sdk &&\
    apt install -y ros-foxy-turtlebot3-msgs &&\
    apt install -y ros-foxy-turtlebot3"

# Confige Environment
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash &&\
    echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc &&\
    source ~/.bashrc"

# Install turtlebot3 simulator packages
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash &&\
    cd ~/dev_ws/src/ &&\
    git clone -b foxy-devel https://github.com/suho0515/turtlebot3_simulations.git &&\
    cd ~/dev_ws && colcon build --symlink-install"

