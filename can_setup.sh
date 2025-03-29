sudo modprobe can
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 2000000 fd on berr-reporting on
sudo ip link set can0 up