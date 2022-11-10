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

# Install Python pip
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    apt-get install -y python-pip"

# Install preliminaries
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    pip install paho-mqtt"

RUN echo "build for no cache_a"

# Install ips package
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash &&\
    git clone https://github.com/suho0515/pcp &&\
    cp -r /pcp/ips/ips ~/catkin_ws/src &&\
    rm -rf pcp &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"