FROM nvidia/cudagl:11.1.1-base-ubuntu18.04
 
# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
# Install ROS Melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-melodic-desktop-full
RUN apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Build OpenCV 4.x
RUN sudo apt-get install -y wget unzip git g++ cmake
# RUN mkdir -p /home/ros/dev/opencv && cd /home/ros/dev/opencv
# RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
# RUN unzip opencv.zip
# RUN mkdir -p build && cd build
# RUN cmake ../opencv-4.x
# RUN cmake --build

# Install OpenCV 4.x
# RUN sudo make install

# Install ROS Deps
RUN apt-get install -y ros-melodic-image-transport ros-melodic-cv-bridge ros-melodic-vision-opencv

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/melodic/setup.bash && mkdir -p /home/ros/robothon_ws/src && cd /home/ros/robothon_ws && catkin_make

WORKDIR /home/ros/robothon_ws
COPY . /home/ros/robothon_ws/src/edge_detector
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN source /opt/ros/melodic/setup.bash && catkin_make
