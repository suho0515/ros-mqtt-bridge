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

# Documentation
# ========================================
# Install Python Package for Documentation
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    pip install Sphinx &&\
    pip install sphinx_rtd_theme &&\
    apt-get install -y ros-melodic-rosdoc-lite"