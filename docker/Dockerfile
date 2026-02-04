ARG ROS_VERSION=noetic-desktop-full

FROM osrf/ros:$ROS_VERSION

ARG REPO_NAME="Handling_RoboticMaster"
ARG REPOS_PATH="/home/repos"
ARG MANIPULACION_WS="/workspaces/$REPO_NAME/manipulacion_ws"

# Disable interactive questions
ENV DEBIAN_FRONTEND=noninteractive
ENV REPO_NAME=$REPO_NAME
ENV REPOS_PATH=$REPOS_PATH
ENV MANIPULACION_WS=$MANIPULACION_WS

# Change docker shell
SHELL ["/bin/bash", "-c"]

# Install ubuntu tools 
RUN apt-get update && apt-get install -y --no-install-recommends \
nano \
bash \
vim \
build-essential \
cmake \
wget \
lsb-release \
bash-completion \
git \
build-essential \
curl \
python3-pip \
coreutils \
python3-catkin-tools \
&& rm -rf /var/lib/apt/lists/*

# Add the robotpkg repository and key
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    ca-certificates \
 && mkdir -p /etc/apt/keyrings \
 && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    -o /etc/apt/keyrings/robotpkg.asc \
 && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] \
    http://robotpkg.openrobots.org/packages/debian/pub \
    $(lsb_release -cs) robotpkg" \
    > /etc/apt/sources.list.d/robotpkg.list

# Install requiered packages
RUN apt-get update && apt-get install -y --no-install-recommends \
   apt-utils \
   ipython3 \
   python-is-python3 \
   python3-scipy \
   python3-wstool \
   python3-networkx \
   python3-pip \
   python3-vcstool \
   python3-rosinstall \
   python3-catkin-tools \
   libsoqt520 \
   libsoqt520-dev \
   libblas-dev \
   liblapack-dev \
   libqhull-dev \
   libeigen3-dev \
   ros-noetic-universal-robots \
   ros-noetic-rqt-joint-trajectory-controller \
   ros-noetic-kdl-parser-py \
   ros-noetic-trac-ik-python \
   python3-rospkg \
   x11-apps \
   mesa-utils \
   libgl1-mesa-glx \
   && rm -rf /var/lib/apt/lists/*

# Install pip packages
RUN pip install \
   ipykernel \
   pyyaml \
   ipywidgets

# Install pinocchio
RUN apt-get update && apt-get install -qqy -y --no-install-recommends robotpkg-py3*-pinocchio

# Clone and build graspit
RUN mkdir -p $REPOS_PATH \
   && cd $REPOS_PATH \
   && git clone https://github.com/elisabeth-ms/graspit.git --branch qt5 \
   && cd graspit \
   && mkdir build \
   && cd build \
   && cmake .. \
   && make -j4 \
   && make install

# Set environment variables
ENV PATH="/opt/openrobots/bin:$PATH"
ENV PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH"
ENV LD_LIBRARY_PATH="/usr/local/lib:$MANIPULACION_WS/devel/lib:/opt/openrobots/lib:/usr/local/lib"
ENV CMAKE_PREFIX_PATH="/opt/openrobots:$CMAKE_PREFIX_PATH"
ENV GRASPIT="$REPOS_PATH/graspit"

# Set Python path environmental varible
   # Base Paths
ENV ROS_ROOT="/opt/ros/noetic/lib/python3/dist-packages"
ENV WS_DEVEL="$MANIPULACION_WS/devel/lib/python3/dist-packages"
ENV WS_SCRIPTS="$MANIPULACION_WS/src/manipulacion_pkg/scripts"
ENV ROBOTPKG="/opt/openrobots/lib/python3.8/site-packages"

# Python path
ENV PYTHONPATH=${WS_DEVEL}:${ROS_ROOT}:${WS_SCRIPTS}:${ROBOTPKG}:${PYTHONPATH}
ENV ROS_PACKAGE_PATH=$MANIPULACION_WS/src:/opt/ros/noetic/share

CMD ["bash", "-c", "exec bash"]