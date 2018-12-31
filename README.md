# metaldetector-ros
A ROS driver for the metaldetector shield.
Initially this was a node to give a common CAN interface to ROS in a robotic system. It has been somewhat trimmed down to support CAN messages from the metaldetector shield [https://github.com/sharst/metaldetector-shield](https://github.com/sharst/metaldetector-shield). 
This has been forked from the excellent [socketcan_bridge inside canopen](https://github.com/ros-industrial/ros_canopen/tree/melodic-devel/socketcan_bridge). 

We have run this node in combination with the USBtin.
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
```roslaunch metaldetector_ros start_socketcan.launch```

The mapping of messages to topics happens in config/*_config.yaml.
Here's an example config:
```
61:                              # CAN ID
  1:                             # REGISTER NUMBER
    topic: metaldetector/metal0  # NAME OF TOPIC
80:
  1:
    topic: some_other_topic
    pre-offset: 100
    scale: 0.01
    offset: -1000
```

Let's dive into it.
CAN devices have a unique ID on the BUS. The first key in the config specifies one such CAN device (ID 61). This is the default ID of the metaldetector. 
The standard assumed by this node is that we receive 5-byte payloads:
```[REGISTER_NUMBER, VALUE0, VALUE1, VALUE2, VALUE3]```
The config file allows to map each register to a topic. Consequently, the next key is the number of the register, which has the topic as a child. In summary, whatever arrives on register 1 of CAN_ID 61 will be published to the topic metaldetector/metal0. The can bridge will assume that the VALUEs represent one MSB-encoded 32bit int, and combine the VALUEs like so: 
```return_val = VALUE0 | 24 + VALUE1 | 16 + VALUE2 << 8 | VALUE3 ```

Additionally, since we used the VALUEs in the payload to encode strictly UINTs, you can specify pre-offset, scale, and offset parameters to compute a float. The can bridge will internally do the following calculation before publishing to the topic:
```return_val = (value + pre_offset) * scale + offset```
If you did not specify those values in the config, it will assume pre_offset = offset = 0 and scale = 1, i.e. do nothing.

It also works the other way round: In send_config.yaml you can specify that you would like to have all info that is published on a topic to be relayed to the CAN bus. 
```
# Metaldetector                                                                                                    
62:                                                                                                                
  8:                                                                                                               
    topic: commands/enable_coils
```
The mapping in the config file is equal. For the metaldetector, you can write a std_msgs/Int32 value: 0 to commands/enable_coils to send a message with an encoded 0 to register 8 on device 62, which should switch off the coils there.

## Decoding the values of the metaldetector
Since the values coming from the metaldetector are still encoded in a 4-byte payload, we need to decode them before we can use them. With the canbridge running (see above), do a rosrun metaldetector_ros metalnode.
You will then receive decoded values on the topic "metal_detector".


## Information flow from the metaldetector-shield
![Information flow from the metaldetector-shield](https://github.com/sharst/metaldetector-shield/blob/master/images/data-transfer.png)
