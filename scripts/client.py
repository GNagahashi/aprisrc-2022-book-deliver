#!/usr/bin/env python

import rospy
from enum import Enum
from book_deliver.srv import Order


def main():
    # define name (node, service, etc)
    name_of_node = 'book_deliver_client'  # this node name
    name_of_service = 'book_deliver_service'  # name of the service used by this node

    # node initialization
    rospy.init_node(name_of_node, anonymous = True)

    # create client to connect to the server
    rospy.loginfo('Create client for {}, please wait a minute...'.format(name_of_service))
    rospy.wait_for_service(name_of_service)  # when the service is not running, it is blocked here
    handler_for_service = rospy.ServiceProxy(name_of_service, Order)

    rospy.loginfo('Ready to go')

    res = handler_for_service('can you hear me?')
    if res.data:
        print('true')
    else:
        print('false')


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
