FROM osrf/ros:melodic-desktop-full

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

# Install Python3 pip
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    apt-get install -y python-pip"

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

# Install ur package
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git &&\
    cd ~/catkin_ws &&\
    rosdep update &&\
    rosdep install -y --rosdistro melodic --ignore-src --from-paths src &&\
    catkin_make &&\
    source devel/setup.bash"

# Install Python Package for Documentation
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    pip install Sphinx &&\
    pip install sphinx_rtd_theme &&\
    apt-get install -y ros-melodic-rosdoc-lite"