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

# Joy Driver
# ========================================
# Install preliminaries for joy_driver packages
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    apt-get install -y ros-melodic-rospy-message-converter"

# Install joy_driver packages
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone https://github.com/suho0515/joystick_drivers.git &&\
    apt-get update -qq &&\
    rosdep update &&\
    rosdep install --from-paths ./ -i -y --rosdistro melodic &&\
    source /opt/ros/melodic/setup.bash &&\
    cd ~/catkin_ws &&\
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo &&\
    source ~/catkin_ws/devel/setup.bash"

# UR Driver
# ========================================
# Install UR Robot Driver
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/melodic/setup.bash &&\
    cd ~/catkin_ws &&\
    git clone -b staging https://github.com/suho0515/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver &&\
    git clone -b calibration_devel https://github.com/suho0515/universal_robot.git src/fmauch_universal_robot &&\
    sudo apt update -qq &&\
    rosdep update &&\
    rosdep install --from-paths src --ignore-src -y &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash" --no-cache

# Install Cartesian Controller Packages
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/melodic/setup.bash &&\
    cd ~/catkin_ws &&\
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_cartesian_control_msgs.git src/Universal_Robots_ROS_cartesian_control_msgs &&\
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian.git src/Universal_Robots_ROS_controllers_cartesian &&\
    sudo apt update -qq &&\
    rosdep update &&\
    rosdep install --from-paths src --ignore-src -y &&\
    catkin_make &&\
    source devel/setup.bash"

# Mir Driver
# ========================================
# Install mir package
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone -b melodic https://github.com/dfki-ric/mir_robot.git &&\
    apt-get update -qq &&\
    apt-get install -qq -y python-rosdep &&\
    rosdep update &&\
    rosdep install --from-paths ./ -i -y --rosdistro melodic &&\
    source /opt/ros/melodic/setup.bash &&\
    cd ~/catkin_ws &&\
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo &&\
    source devel/setup.bash"

# Mobile Manipulator Package
# ========================================
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    git clone https://github.com/suho0515/pcp &&\
    cp -r /pcp/mobile-manipulator ~/catkin_ws/src &&\
    rm -rf pcp &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# Documentation
# ========================================
# Install Python Package for Documentation
# RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
#     pip install Sphinx &&\
#     pip install sphinx_rtd_theme &&\
#     apt-get install -y ros-melodic-rosdoc-lite"