
# Запуск teleop

```
ros2 run auv teleop
```

# Запуск stabilize

```
ros2 run auv stabilize
```

# Настройка PID

```
rosparam set /stabilizer_node/depth_pid/kp 2.0
rosparam set /stabilizer_node/depth_pid/ki 0.0
rosparam set /stabilizer_node/depth_pid/kd 0.5

rosparam set /stabilizer_node/yaw_pid/kp 2.0
rosparam set /stabilizer_node/yaw_pid/ki 0.0
rosparam set /stabilizer_node/yaw_pid/kd 0.5
```