# metaldetector-ros
A simple node interpreting the CAN messages arriving from the metaldetector board [https://github.com/sharst/metaldetector-shield](https://github.com/sharst/metaldetector-shield). 
For CAN communication, this relies on:
* LinuxCAN
* [socketcan_bridge inside canopen](https://github.com/ros-industrial/ros_canopen/tree/melodic-devel/socketcan_bridge)
* [USBtin CAN Adapter](https://www.fischl.de/usbtin)

## Setting up the USBtin with LinuxCAN
Please follow this guide to get it set up with socketcan: https://www.fischl.de/usbtin/linux_can_socketcan/

Attention!
Since the metaldetector-shield is using a baud rate of 500000kbps, we need to specify '-s6' instead of '-s5':
```
$ sudo ./slcan_attach -f -s6 -o /dev/ttyACM0
attached tty /dev/ttyACM0 to netdevice slcan0
$ sudo ./slcand ttyACM0 slcan0
$ sudo ifconfig slcan0 up
```
For a first test, connect the USBtin to the metaldetector-shield and run
``` ./candump slcan0 ```
If you do not see messages coming in, something is wrong in your wiring.

## Setting up metaldetector-ros
This is simply a ROS package, so manage and build as usual. Launch with:
```roslaunch metaldetector_ros start_with_socketcan.launch```
This launches both the socketcan_bridge, which translates ROS messages to CAN messages and vice versa, as well as the metaldetector node.


