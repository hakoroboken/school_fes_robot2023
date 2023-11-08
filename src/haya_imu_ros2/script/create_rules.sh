#!/bin/bash
echo "remap the device serial port(ttyACM*) to haya_imu"
echo "haya_imu usb connection as /dev/haya_imu, check it using command: ls -l /dev/ttyACM*"
echo "start copy haya_imu.rules to /etc/udev/rules.d/"ros
sudo cp ~/ros2_ws/src/haya_imu_ros2/script/haya_imu.rules /etc/udev/rules.d
echo ""
echo "restarting udev..."
echo ""
sudo service udev reload
sudo service udev restart
echo "done"
