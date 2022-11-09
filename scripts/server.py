#!/usr/bin/env python

from os import environ
from enum import Enum
import rospy
# from time import sleep
from std_msgs.msg import String
from book_deliver.srv import Order, OrderResponse


def main():
    name_of_node = 'book_deliver_server'
    name_of_service = 'book_deliver_service'
    name_of_topic_for_drone = '/gnc_node/cmd'

    service = Service()

    rospy.init_node(name_of_node, anonymous = True)

    rospy.loginfo('Creating service server({}), please wait a minute...'.format(name_of_service))
    rospy.Service(name_of_service, Order, service.service_callback)

    rospy.loginfo('Creating publisher for "{}", please wait a minute...'.format(name_of_topic_for_drone))
    service.topic_name_for_drone_ctrl = name_of_topic_for_drone
    service.handler_for_drone_ctrl = rospy.Publisher(name_of_topic_for_drone, String, queue_size = 10)

    rospy.loginfo('Ready to go')
    rospy.spin()
    pass


class Service(object):
    def __init__(self):
        """constructor"""
        self.__topic_name_for_drone_ctrl = None  # name of topic to send command for drone
        self.__handler_for_drone_ctrl = None  # handler for publish message to drone
        self.__publish_rate = 1  # 1[Hz], publish 1 times per second

    @property
    def topic_name_for_drone_ctrl(self):
        """getter"""
        return self.__topic_name_for_drone_ctrl
    @topic_name_for_drone_ctrl.setter
    def topic_name_for_drone_ctrl(self, topic_name):
        """setter"""
        self.__topic_name_for_drone_ctrl = topic_name

    @property
    def handler_for_drone_ctrl(self):
        """getter"""
        return self.__handler_for_drone_ctrl
    @handler_for_drone_ctrl.setter
    def handler_for_drone_ctrl(self, handler):
        """setter"""
        self.__handler_for_drone_ctrl = handler
    
    @property
    def publish_rate(self):
        """getter"""
        return self.__publish_rate
    @publish_rate.setter
    def publish_rate(self, rate):
        """setter"""
        self.__publish_rate = rate
    
    def service_callback(self, req):
        """when server received message, execute this function"""
        rospy.loginfo('Server has received request: {}'.format(req.data))
        res = OrderResponse()  # create response
        if req == ServiceCmd.ORDER.value:
            with open('/home/' + environ['USER'] + '/waypoint.dat', mode = 'r', encoding = 'utf-8') as f:
                n = int(f.readline().replace(':', ''))
            
            for i in range(n):
                self.cmd_to_drone(DroneCmd.NEXT.value)
                rospy.sleep(5)
            pass
        elif req == ServiceCmd.ACK.value:
            pass
        return res  # send response to client

    def cmd_to_drone(self, cmd):
        """send command to drone"""
        if not rospy.is_shutdown():
            msg = String(data = cmd)
            self.handler_for_drone_ctrl(msg)  # publish message
            rospy.loginfo('Published message: {}'.format(msg.data))
            self.publish_rate.sleep()
        else:
            rospy.loginfo('Node({}) is shutdown'.format(self.topic_name_for_drone_ctrl))


class ServiceCmd(Enum):
    """command for service"""
    ORDER = 'order'
    ACK = 'ack'


class DroneCmd(Enum):
    """command for drone control"""
    START = 'start'
    HALT = 'halt'
    NEXT = 'next'
    BACKHOME = 'backhome'


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        print('bye')
