FROM dustynv/ros:humble-ros-core-l4t-r35.4.1
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# fix gpg issue
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

SHELL ["/bin/bash","-lc"]

WORKDIR /ros2_ws
RUN mkdir -p src \
 && git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

COPY ./auv ./src/auv

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Уставливаем BehaviorTree.CPP 4.7
RUN wget https://github.com/BehaviorTree/BehaviorTree.CPP/archive/refs/tags/4.7.0.tar.gz && tar -xf 4.7.0.tar.gz && rm 4.7.0.tar.gz && cd BehaviorTree.CPP-4.7.0 && \
    pip3 install --ignore-installed PyYAML conan && conan profile detect && conan install . -of build_release --build missing -s build_type=Release && cmake -S . -B build_release -DCMAKE_TOOLCHAIN_FILE="build_release/conan_toolchain.cmake" && \
    cmake --build build_release

# 10) Устанавливаем либы для работы с камерами

# apt-get install -y libopencv-imgproc-dev

RUN cd src && wget https://github.com/ros-drivers/usb_cam/archive/refs/tags/0.8.1.tar.gz  && \
    tar -xf 0.8.1.tar.gz && rm 0.8.1.tar.gz && \
    wget https://github.com/ros-perception/image_transport_plugins/archive/refs/tags/6.1.0.tar.gz && \
    tar -xf 6.1.0.tar.gz && rm 6.1.0.tar.gz && rosdep update && \
    apt update && apt install -y libogg-dev && \
    rosdep install --from-paths src --ignore-src -y


# 9) Точка входа
# ENTRYPOINT ["/bin/bash","-c","if [ -f '/opt/ros/humble/setup.bash' ]; then source /opt/ros/humble/setup.bash; elif [ -f '/usr/local/setup.bash' ]; then source /usr/local/setup.bash; elif [ -f '/ros_entrypoint.sh' ]; then source /ros_entrypoint.sh; fi && source /ros2_ws/install/setup.bash"]