FROM dustynv/ros:humble-ros-core-l4t-r35.4.1


WORKDIR /ros2_ws
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN sudo apt update && rosdep update && rosdep install --from-paths src --ignore-src -y
RUN colcon build

# Копируем весь ROS-пакет в контейнер
COPY ./auv ./src/auv

# Сборка пакета
RUN colcon build

# Настройка окружения при запуске
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Точка входа
ENTRYPOINT bash -c "ource /opt/ros/humble/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           source /ros2_ws/install/local_setup.bash && \
           ros2 run auv stabilize"
