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

        self._last_message_ft_1 = Float64MultiArray()
        self._last_message_ft_1.data = [0]

        self._last_message_ft_2 = Float64MultiArray()
        self._last_message_ft_2.data = [0]

        self._last_message_dice = Float64MultiArray()
        self._last_message_dice.data = [0]

        self._message_queue = multiprocessing.Queue()

        self._udp_node = UDPNode(self._port, self._ip)
        print("udp node created, setting callback fcn")
        self._udp_node.setMsgCallbackFunction(self.cbk_message)
        print("callback fcn set. starting udp node")
        self._udp_node.start()
        self._pub_ft = rospy.Publisher('tactile_finger_force_sensor_1', Float64MultiArray, queue_size=10)
        self._pub_ft = rospy.Publisher('tactile_finger_force_sensor_2', Float64MultiArray, queue_size=10)
        self._pub_ft = rospy.Publisher('tactile_dice_imu', Float64MultiArray, queue_size=10)
        print("NIWrapper initialized")


    def cbk_message(self, m):
        # print(m[0].decode(b, 'utf-8'))
        print(m)
        #msgs = codecs.decode(m[0], 'utf-8').split("\r\n",maxsplit=3)
        #num_of_msgs = len(msgs)

        msg_raw = codecs.decode(m[0], 'utf-8')
        self._message_queue.put(msg_raw)
        
 
    def update(self):   

        res1 = self._message_queue.get()
        print("res1: ",res1)
        ft_data_list = self.extract_numbers(res1, 'F')
        dice_data_list = self.extract_numbers(res1, 'D')

        print("ft: ",ft_data_list)
            #print(res1)
            #for msg in res1:
                #print(msg)
                #self._last_message_ft.data = [self.convert_to_float_list(ii) for ii in msg]
 
        print("ft_data_list[0]: ", ft_data_list[0])
        self._last_message_ft_1.data = [float(ii) for ii in ft_data_list[0]]

        self._pub_ft.publish(self._last_message_ft_1)
        self._pub_ft.publish(self._last_message_ft_2)
        #self._pub_dice.publish(self._last_message_dice)


    def stop(self):
        self._udp_node.stop()

    def extract_numbers(self, input_string, char_to_search_for):
        lines = input_string.split('\r\n')
        result = []
        for line in lines:
            if char_to_search_for in line:
                numbers = [float(num) for num in line.split('\t')[1:]]
                result.append(numbers)
        return result



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
