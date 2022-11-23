FROM osrf/ros:kinetic-desktop-full

# Install basics
RUN /bin/bash -c "apt-get update &&\
    apt-get install -y git &&\
    source /opt/ros/kinetic/setup.bash &&\
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# Install Python
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash &&\
    apt-get install -y python-pip"

RUN echo "fast update without cache 20221122"

# Install Dynamixel Workbench Packages
RUN /bin/bash -c "source ~/catkin_ws/devel/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone https://github.com/suho0515/dynamixel-workbench.git &&\
    git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git &&\
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# Install Dependencies of RH-P12-RN Package
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash &&\
    apt install -y ros-kinetic-ros-control &&\
    apt install -y ros-kinetic-ros-controllers &&\
    apt install -y ros-kinetic-gazebo-ros-control &&\
    source /opt/ros/kinetic/setup.bash"

# Install RH-P12-RN Package
RUN /bin/bash -c "source ~/catkin_ws/devel/setup.bash &&\
    cd ~/catkin_ws/src &&\
    # git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git &&\
    git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git &&\
    git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git &&\
    git clone https://github.com/ROBOTIS-GIT/RH-P12-RN-A.git &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# # Install Python Package for Documentation
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
#     pip3 install Sphinx &&\
#     pip3 install sphinx_rtd_theme &&\
#     apt-get install -y ros-noetic-rosdoc-lite"

# # Install Python Package for ocr
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
#     apt-get install -y tesseract-ocr tesseract-ocr-script-hang tesseract-ocr-script-hang-vert &&\
#     pip3 install pytesseract &&\
#     pip3 install imutils"

# # Install Dynamixel Workbench Packages
# RUN /bin/bash -c "source ~/catkin_ws/devel/setup.bash &&\
#     cd ~/catkin_ws/src &&\
#     git clone https://github.com/suho0515/dynamixel-workbench.git &&\
#     git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git &&\
#     git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git &&\
#     cd ~/catkin_ws &&\
#     catkin_make &&\
#     source devel/setup.bash"

# # Install Python Package for Documentation
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
#     pip3 install Sphinx &&\
#     pip3 install sphinx_rtd_theme &&\
#     apt-get install -y ros-noetic-rosdoc-lite"

