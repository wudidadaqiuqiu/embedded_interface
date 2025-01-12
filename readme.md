rock@rock3a:~$ sudo ip link set can0 down
rock@rock3a:~$ sudo ip link set can0 type can bitrate 1000000 dbitrate 1000000 loopback on fd on
rock@rock3a:~$ sudo ip link set can0 up

sudo ip link set can0 type can bitrate 1000000 dbitrate 1000000 loopback on fd on && sudo ip link set can0 up

cansend can0 123#0102030405060708

rostopic pub /test_can_frame1 connector/IdPack "id: 1"

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 loopback off
sudo ip link set can0 up

// test_tty.cpp
测试imu的串口通信，注意con_open("/dev/ttyUSB0") 字符串填上对应的串口设备路径，串口数据以1000Hz接收，数据流向为connector recv node -> unpacker -> your callback