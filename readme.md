rock@rock3a:~$ sudo ip link set can0 down
rock@rock3a:~$ sudo ip link set can0 type can bitrate 1000000 dbitrate 1000000 loopback on fd on
rock@rock3a:~$ sudo ip link set can0 up

sudo ip link set can0 type can bitrate 1000000 dbitrate 1000000 loopback on fd on && sudo ip link set can0 up

cansend can0 123#0102030405060708