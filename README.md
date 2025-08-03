
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


# Настройка окружения для разработки
Для отладки и разработки есть два Dockerfile для x86 и jetson, сборка осуществляется для x86:

```bash
docker build -t auv .
```
для jetson

```bash
docker build -t aur -f jetson.Dockerfile .
```

## Запуск Docker image для разработки и отладки

```bash
docker run -v /dev:/dev --privileged --rm -it auv bash
```

## Запуск microros-agent
Для взаимодействия esp и бортовой системой необходимо запустить агент

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev <тут порт>
```


# Отладка работы двигателей
Для отладки движков, необходимо собрать прошивку без фрага PROD, в таком режиме будет топик
`/manual_thrusters`

Узнать тип сообщений любого топика можно вот так:
```bash
rostopic type /manual_thrusters
```
Узнать подробно формат публикуемых данный вот так:
```bash
ros2 interface show std_msgs/msg/Int8MultiArray
```
вывод:
```
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
        #
        #
        #
        #
        #
        MultiArrayDimension[] dim #
                string label   #
                uint32 size    #
                uint32 stride  #
        uint32 data_offset        #
int8[]            data          # array of data
```

Что бы опубликовать в типик сообщения необходимо вызвать команду:

```bash
ros2 topic pub /manual_thrusters std_msgs/msg/Int8MultiArray "{layout: {dim: [], data_offset: 0}, data: [0, 0, 0, 0, 0]}"
```