#!/usr/bin/python
"""
A node that interprets and bundles the data from the metal detector sensors
"""
import rospy
from metaldetector_ros.msg import StampedFloat, StampedSensor


class MetalInterpreter(object):
    def __init__(self):
        self.pressure_sensors = 0

        self.metal_pub = rospy.Publisher("metal_detector", StampedSensor, queue_size=1)

        rospy.Subscriber("raw/metal_0", StampedFloat, self.metal_cb, 0)
        rospy.Subscriber("raw/metal_2", StampedFloat, self.metal_cb, 2)
        rospy.Subscriber("raw/metal_4", StampedFloat, self.metal_cb, 4)
        rospy.Subscriber("raw/metal_6", StampedFloat, self.metal_cb, 6)
        rospy.Subscriber("raw/metal_8", StampedFloat, self.metal_cb, 8)
        rospy.Subscriber("sensors/pressure_heartbeat", StampedFloat, self.heartbeat_cb)

    def heartbeat_cb(self, msg):
        sensno = 0
        for i in range(8):
            if (int(msg.data) >> (16 + i)) & 1:
                sensno += 1

        if sensno != self.pressure_sensors:
            rospy.logwarn("!!!! The number of pressure sensors has changed \
                           from {} to {}".format(self.pressure_sensors, sensno))

        self.pressure_sensors = sensno

    def unhassle_metal(self, msg):
        return ((int(msg.data) >> 16) & 0xFFFF,
                int(msg.data) & 0xFFFF)

    def metal_cb(self, msg, ind):
        (a, b) = self.unhassle_metal(msg)

        self.metal_pub.publish(StampedSensor(a, msg.stamp, "coil_{}".format(ind)))
        self.metal_pub.publish(StampedSensor(b, msg.stamp, "coil_{}".format(ind+1)))
