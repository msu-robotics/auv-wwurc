FROM dustynv/ros:humble-ros-core-l4t-r35.4.1
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# fix gpg issue
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

SHELL ["/bin/bash","-lc"]

WORKDIR /ros2_ws
# RUN mkdir -p src \
#  && git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# COPY ./auv ./src/auv

# RUN apt-get update && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y && \
#     rm -rf /var/lib/apt/lists/*

# Уставливаем BehaviorTree.CPP 4.7
# RUN wget https://github.com/BehaviorTree/BehaviorTree.CPP/archive/refs/tags/4.7.0.tar.gz &&\
#     tar -xf 4.7.0.tar.gz && rm 4.7.0.tar.gz && cd BehaviorTree.CPP-4.7.0 && \
#     pip3 install --ignore-installed PyYAML conan && conan profile detect && conan install . -of build_release --build missing -s build_type=Release && cmake -S . -B build_release -DCMAKE_TOOLCHAIN_FILE="build_release/conan_toolchain.cmake" && \
#     cmake --build build_release

# Устанавливаем зависимости для foxglove bridge

RUN bash -c "\
  source /opt/ros/$ROS_DISTRO/install/setup.bash && \
  git clone --depth 1 https://github.com/zaphoyd/websocketpp.git /foxglove-deps/websocketpp && \
  cd /foxglove-deps/websocketpp && mkdir build && cd build &&\
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make install && \
  git clone --branch ${ROS_DISTRO} --depth 1 https://github.com/ros/resource_retriever.git /foxglove-deps/resource_retriever && \
  cd /foxglove-deps/resource_retriever/libcurl_vendor && mkdir build && cd build && \
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make install && \
  cd /foxglove-deps/resource_retriever/resource_retriever && mkdir build && cd build && \
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local\ && make install &&\
  git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rosbag2.git /foxglove-deps/rosbag2 && \
  cd /foxglove-deps/rosbag2/rosbag2_test_common && mkdir build && cd build && \
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make install && \
  cd /foxglove-deps/rosbag2/rosbag2_storage && mkdir build && cd build && \
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make install && \
  cd /foxglove-deps/rosbag2/rosbag2_cpp && mkdir build && cd build && \
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make install && \
  git clone --depth 1 https://github.com/Tencent/rapidjson.git /foxglove-deps/rapidjson && \
  git clone --depth 1 https://github.com/facontidavide/rosx_introspection.git /foxglove-deps/rosx_introspection && \
  mv /foxglove-deps/rapidjson/include/rapidjson /foxglove-deps/rosx_introspection/include && \
  cd /foxglove-deps/rosx_introspection && mkdir build && cd build && \
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make install"

# Устанавливаем либы для работы с камерами

RUN bash -c "\
  cd /ros2_ws && \
  source /opt/ros/$ROS_DISTRO/install/setup.bash && \
  git clone --branch ${ROS_DISTRO} --depth 1 https://gitlab.com/boldhearts/ros2_v4l2_camera.git src/v4l2_camera && \
  git clone --branch ${ROS_DISTRO} --depth 1 https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
  git clone --depth 1 https://github.com/foxglove/ros-foxglove-bridge.git src/ros-foxglove-bridge && \
  apt update && rosdep update && \
  rosdep install --from-paths src -r -y && \
  colcon build --event-handlers console_direct+ --symlink-install"

# Точка входа
# ENTRYPOINT ["/bin/bash","-c","if [ -f '/opt/ros/humble/setup.bash' ]; then source /opt/ros/humble/setup.bash; elif [ -f '/usr/local/setup.bash' ]; then source /usr/local/setup.bash; elif [ -f '/ros_entrypoint.sh' ]; then source /ros_entrypoint.sh; fi && source /ros2_ws/install/setup.bash"]