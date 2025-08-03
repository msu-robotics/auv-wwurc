FROM ros:humble


WORKDIR /microros_ws

RUN git clone -b $ROS_DISTRO --depth 1 https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

RUN bash -c "\
  source /opt/ros/$ROS_DISTRO/setup.bash && \
  apt update && \
  apt install -y python3-rosdep2 python3-pip picocom tmux neovim && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -y && \
  colcon build && \
  source /microros_ws/install/local_setup.bash && \
  ros2 run micro_ros_setup create_agent_ws.sh && \
  ros2 run micro_ros_setup build_agent.sh && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*"

WORKDIR /ros2_ws

# Копируем весь ROS-пакет в контейнер
COPY ./auv ./src/auv

# Сборка пакета
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash &&\
source /microros_ws/install/local_setup.bash &&\
rosdep install --from-paths src --ignore-src -y && colcon build"

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /microros_ws/install/local_setup.bash" >> /root/.bashrc