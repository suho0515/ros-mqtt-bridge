FROM osrf/ros:noetic-desktop-full

# Install basics
RUN /bin/bash -c "apt-get update &&\
    apt-get install -y git &&\
    source /opt/ros/noetic/setup.bash &&\
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# Install Python
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    apt-get install -y python3-pip"

# Install usb-cam package
RUN /bin/bash -c "apt-get install -y ros-noetic-usb-cam"

# Install Python Package for ocr
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    apt-get install -y tesseract-ocr tesseract-ocr-script-hang tesseract-ocr-script-hang-vert &&\
    pip3 install pytesseract &&\
    pip3 install imutils"

# Install Python Package for serial
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    pip3 install pyserial"

# Install Dynamixel Workbench Packages
RUN /bin/bash -c "source ~/catkin_ws/devel/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone https://github.com/suho0515/dynamixel-workbench.git &&\
    git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git &&\
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# Install Python Package for Documentation
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    pip3 install Sphinx &&\
    pip3 install sphinx_rtd_theme &&\
    apt-get install -y ros-noetic-rosdoc-lite"

# for fast build update
CMD ["echo", "update modee_20221216"]

# Install ocr package
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone https://github.com/suho0515/ocr.git &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# Install modee package
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    cd ~/catkin_ws/src &&\
    git clone https://github.com/suho0515/modee.git &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"



