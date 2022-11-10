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

    client = Client()

    # create client to connect to the server
    rospy.loginfo('Create client for {}, please wait a minute...'.format(name_of_service))
    rospy.wait_for_service(name_of_service)  # when the service is not running, it is blocked here
    client.handler_for_service = rospy.ServiceProxy(name_of_service, Order)

    rospy.loginfo('Ready to go')

    # TODO: modify the following operations from CUI to GUI
    val = input('\nWould you like to order a book? [input "y" or "n"]\n')
    if val == 'y':
        print('The drone will come to you, please wait...')
        if client.call_service(ServiceCmd.ORDER.value):
            # request was successful
            val = input('Have you received the book? [input "y" or "n"]\n')
            if val == 'y':
                client.call_service(ServiceCmd.ACK.value)
                print('Thank you for your using')
                pass
            elif val == 'n':
                # TODO: add a operations that accepts input again
                pass
            else:
                # invalid input (input other than "y", "n")
                pass
        else:
            # request was failed
            print('The Drone has a malfunction')
            print('Your request was cancelled, We apologize for the inconvenience')
    elif val == 'n':
        # did not order
        print('Did not order')
    else:
        # invalid input (input other than "y", "n")
        print('Invalid input')
    print('End')


class Client(object):
    def __init__(self):
        """constructor"""
        self.__handler_for_service = None  # handler for send request to service

    @property
    def handler_for_service(self):
        """getter"""
        return self.__handler_for_service
    @handler_for_service.setter
    def handler_for_service(self, handler):
        """setter"""
        self.__handler_for_service = handler

    def call_service(self, req):
        """send request to service(server)"""
        res = self.handler_for_service(req)  # send request
        # response(res) from service is bool type
        # true means that the request was successful
        return res.data  # true or false


class ServiceCmd(Enum):
    """command for service"""
    ORDER = 'order'
    ACK = 'ack'


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        print('bye')
