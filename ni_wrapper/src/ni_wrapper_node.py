#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from server_listener import UDPNode
import struct
import multiprocessing #import Process, Queue, Value, SimpleQueue
import time
import codecs

class NIWrapper():
    def __init__(self):
        if rospy.has_param("~communication/IP"):
            self._ip = rospy.get_param("~communication/IP")
        else:
            raise Exception("No IP param")

        if rospy.has_param("~communication/port"):
            self._port = rospy.get_param("~communication/port")
        else:
            raise Exception("No port param")

        self._last_message_dice = String()
        self._last_message_dice.data = []

        self._last_message_ft = Float64MultiArray()
        self._last_message_ft.data = [0]

        self._message_queue_ft_sensor = multiprocessing.Queue()
        self._message_queue_dice = multiprocessing.Queue()

        self._message_raw = multiprocessing.Queue()

        self._udp_node = UDPNode(self._port, self._ip)
        print("udp node created, setting callback fcn")
        self._udp_node.setMsgCallbackFunction(self.cbk_message)
        print("callback fcn set. starting udp node")
        self._udp_node.start()
        self._pub_ft = rospy.Publisher('tactile_finger_force_sensor', Float64MultiArray, queue_size=10)
        self._pub_dice = rospy.Publisher('tactile_dice_imu', String, queue_size=10)
        print("NIWrapper initialized")


    def cbk_message(self, m):
        # print(m[0].decode(b, 'utf-8'))
        self._message_raw.put(m)
        print(m)
        msgs = codecs.decode(m[0], 'utf-8').split(";;",maxsplit=1)

        msg_dice = ""
        msg_ft_sensor = msgs[0]
        if len(msgs)>1:
            msg_dice = msgs[1]

        print("ft: ", msg_ft_sensor)
        print("dice: ", msg_dice)
        
        vett = msg_ft_sensor.split("\r\n")[0].split("\t")
        self._message_queue_ft_sensor.put(vett)

        vett = msg_dice
        self._message_queue_dice.put(vett)

 
    def update(self):   
        res1 = None
        while not self._message_queue_ft_sensor.empty():
            res1 = self._message_queue_ft_sensor.get()
            self._last_message_ft.data = [self.convert_to_float_list(ii) for ii in res1]

        res2 = None
        while not self._message_queue_dice.empty():
            res2 = self._message_queue_dice.get()
            self._last_message_dice.data = res2
 
        self._pub_ft.publish(self._last_message_ft)
        self._pub_dice.publish(self._last_message_dice)


    def stop(self):
        self._udp_node.stop()

    def convert_to_float_list(self, s):
        return float(s.replace(',', '.'))


if __name__ == '__main__':
    rospy.init_node('ni_wrapper', anonymous=True)
    # ni = NIWrapper("172.25.114.19",61557)
    ni = NIWrapper()
    rate = rospy.Rate(100) # 100hz

    print("Entering loop...")
    while not rospy.is_shutdown():
        ni.update()
        rate.sleep()

    rospy.loginfo("STOP")
    ni.stop()
