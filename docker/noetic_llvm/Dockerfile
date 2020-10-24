FROM ros:noetic

RUN apt-get update && apt-get install -y --no-install-recommends \
 && apt-get install -y --no-install-recommends wget nano build-essential \
 git clang lld libomp-dev \
 ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-rviz \
 ros-noetic-tf-conversions ros-noetic-libg2o libglm-dev libglfw3-dev \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace'
RUN git clone https://github.com/koide3/ndt_omp.git
RUN git clone https://github.com/koide3/hdl_graph_slam.git
RUN git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive

# RUN git clone https://github.com/koide3/interactive_slam.git --recursive
COPY . /root/catkin_ws/src/interactive_slam/
WORKDIR /root/catkin_ws/src/interactive_slam
RUN git submodule init
RUN git submodule update

RUN update-alternatives --install /usr/bin/ld ld /usr/bin/ld.lld-10 50

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; CC=clang CXX=clang++ catkin_make'
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
