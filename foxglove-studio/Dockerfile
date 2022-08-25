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

# Install mir_driver packages
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone -b noetic https://github.com/dfki-ric/mir_robot.git &&\
    apt-get update -qq &&\
    rosdep update &&\
    rosdep install --from-paths ./ -i -y --rosdistro noetic &&\
    source /opt/ros/noetic/setup.bash &&\
    cd ~/catkin_ws &&\
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo &&\
    source ~/catkin_ws/devel/setup.bash"

# Install Python Package for Documentation
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    pip3 install Sphinx &&\
    pip3 install sphinx_rtd_theme &&\
    apt-get install -y ros-noetic-rosdoc-lite"

# Install curl
RUN /bin/bash -c "apt install -y curl"

# Install git-lfs
RUN /bin/bash -c "apt update &&\
    apt -y install git-lfs"

# Install Python3 pip
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    cd ~ &&\
    git clone https://github.com/foxglove/studio.git" 

RUN apt update
RUN curl -fsSL https://deb.nodesource.com/setup_16.x | sudo -E bash -
RUN curl -sL https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
RUN echo "deb https://dl.yarnpkg.com/debian/ stable main" | tee /etc/apt/sources.list.d/yarn.list
RUN apt update
RUN apt install -y nodejs yarn

WORKDIR /root/studio

RUN yarn install