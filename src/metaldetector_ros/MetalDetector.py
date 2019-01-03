#!/usr/bin/python
"""
A node that interprets and bundles the data from the metal detector sensors
"""
import rospy
from can_msgs.msg import Frame
from std_msgs.msg import Bool

COILS = 8
PRESSURE_REGISTER = 25
HEARTBEAT_REGISTER = 30
COIL_REGISTER = 40
ENABLE_COILS_REGISTER = 8
MD_CAN_ID = 62


class MetaldetectorNode(object):
    def __init__(self):
        rospy.init_node('metaldetectornode')
        self.pressure_sensors = 0
        self.metal_sensors = [0] * COILS

        self.can_pub = rospy.Publisher("sent_messages", Frame, queue_size=1)
        rospy.Subscriber("received_messages", Frame, self.can_cb)
        rospy.Subscriber("enable_coils", Bool, self.enable_coils_cb)

    def can_cb(self, msg):
        data = [ord(d) for d in msg.data[:msg.dlc]]

        if data[0] == COIL_REGISTER:
            self.metal_sensors[data[1]] = data[2] << 8 | data[3]
            # print self.metal_sensors

        elif data[0] == PRESSURE_REGISTER:
            for i in range(8):
                if (data[3] >> i) & 1:
                    print "Pressure sensor {} switched on!".format(i)

        elif data[0] == HEARTBEAT_REGISTER:
            for i in range(8):
                if (data[1] >> i) & 1:
                    print "Pressure sensor {} is attached!".format(i)
            if data[2]:
                print "Coils are enabled!"
            else:
                print "Coils are disabled!"


    def enable_coils_cb(self, msg):
        frame = Frame()
        frame.header.stamp = rospy.Time.now()
        frame.id = MD_CAN_ID
        frame.is_rtr = False
        frame.is_extended = True
        frame.is_error = False
        frame.dlc = 2
        frame.data = ''.join([chr(x) for x in 1 << 7 | ENABLE_COILS_REGISTER, int(msg.data)])
        self.can_pub.publish(frame)


if __name__ == "__main__":
    md = MetaldetectorNode()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
