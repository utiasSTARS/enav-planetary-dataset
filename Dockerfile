FROM osrf/ros:kinetic-desktop-full

LABEL author="Olivier Lamarre"
LABEL description="Run ENAV dataset utilities"

# Dependencies
RUN apt-get update \
    && apt-get install -y \
    cython \
    git \
    libeigen3-dev \
    libgdal-dev \
    python-pip \
    python-tk \
    ros-kinetic-grid-map \
    ros-kinetic-interactive-marker-twist-server \
    ros-kinetic-moveit-ros-visualization \
    ros-kinetic-robot-localization \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Catkin workspace & custom package
RUN mkdir -p /root/catkin_ws/src
COPY ./enav_ros /root/catkin_ws/src/enav_ros

RUN git clone https://github.com/MHarbi/bagedit.git /root/catkin_ws/src/bagedit
RUN cd /root/catkin_ws/src/bagedit && git checkout f97b21050826f65757040750cecc165abd67c0d2

RUN . /opt/ros/kinetic/setup.sh && cd /root/catkin_ws && catkin_make

RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Python data fetching script
COPY ./enav_utilities /root/enav_utilities
RUN cd /root/enav_utilities && pip install -e .

# Disable default entrypoint from parent ROS image
ENTRYPOINT []