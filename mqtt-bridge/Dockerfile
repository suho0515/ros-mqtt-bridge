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

# Install Prerequisites for mqtt-bridge
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    apt install -y ros-noetic-rosbridge-library &&\
    apt install -y mosquitto mosquitto-clients &&\
    pip3 install inject &&\
    pip3 install paho-mqtt &&\
    pip3 install msgpack"

# Install mqtt-bridge
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone https://github.com/suho0515/mqtt_bridge.git &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"  

# Install mqtt-bridge requirements
RUN /bin/bash -c "source ~/catkin_ws/devel/setup.bash &&\
    cd ~/catkin_ws/src/mqtt_bridge &&\
    # pip3 install freeze &&\
    pip3 freeze > dev-requirements.txt &&\
    pip3 install -r dev-requirements.txt &&\
    source ~/catkin_ws/devel/setup.bash"  

# Install Python Package for Documentation
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    pip3 install Sphinx &&\
    pip3 install sphinx_rtd_theme &&\
    apt-get install -y ros-noetic-rosdoc-lite"