FROM ros:melodic

# ros
RUN apt-get update && apt-get install -y ros-melodic-desktop-full
RUN apt-get install -y wget

# opencv
RUN wget "https://github.com/opencv/opencv/archive/4.3.0.zip" && unzip 4.3.0.zip && rm -r 4.3.0.zip
RUN mkdir /opencv-4.3.0/build && cd /opencv-4.3.0/build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. && make -j7 && make install

# aruco
RUN wget "https://sourceforge.net/projects/aruco/files/3.1.12/aruco-3.1.12.zip" && \
    unzip aruco-3.1.12.zip && rm -r aruco-3.1.12.zip

# one eternity later...

# code
#RUN git clone https://github.com/anton-galkovsky/aruco_ros_docker.git
COPY src /aruco_ros_docker/src
RUN mkdir /aruco_ros_docker/src/aruco_ros/lib && mv /aruco-3.1.12 /aruco_ros_docker/src/aruco_ros/lib/aruco-3.1.12

RUN /bin/bash -c "cd /aruco_ros_docker/src; source /opt/ros/melodic/setup.bash; catkin_init_workspace; cd ..; \
                  catkin_make --pkg duckietown_msgs; source devel/setup.bash; catkin_make"
CMD /bin/bash -c "cd /aruco_ros_docker; source devel/setup.bash; roslaunch aruco_ros aruco_detector.launch"