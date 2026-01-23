FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
  # Install libfranka build depencencies
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libfmt-dev \
  libpoco-dev \
  ros-humble-pinocchio \
  # Install ros packages
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-pip \
  python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

# Install libfranka
ARG LIBFRANKA_VERSION=0.19.0
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
  git clone --recursive https://github.com/frankarobotics/libfranka --branch ${LIBFRANKA_VERSION} \
  && cd libfranka \
  && mkdir build \
  && cd build \
  && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
  && cmake --build . -j $(nproc) \
  && cpack -G DEB \
  && dpkg -i libfranka*.deb \
  && cd ../../ && rm -r libfranka"

# Install franka_ros2
ARG FRANKA_ROS2_VERSION=v2.2.0
ARG FRANKA_DESCRIPTION_VERSION=1.3.0
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && \
  mkdir -p /tmp/franka_ros2 && cd /tmp/franka_ros2 && \
  git clone --recursive https://github.com/frankarobotics/franka_ros2.git --branch ${FRANKA_ROS2_VERSION} && \
  git clone --recursive https://github.com/frankarobotics/franka_description.git --branch ${FRANKA_DESCRIPTION_VERSION} && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths . --ignore-src -r -y --skip-keys libfranka && \
  rm -rf /var/lib/apt/lists/* && \
  colcon build --install-base /opt/ros/humble/franka --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=off  && \
  cd .. && \
  rm -rf /tmp/franka_ros2 && \
  echo "source /opt/ros/humble/franka/setup.bash" >> ~/.bashrc && \
  echo "source /opt/ros/humble/franka/setup.sh" >> ~/.profile'
