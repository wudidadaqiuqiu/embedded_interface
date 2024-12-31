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