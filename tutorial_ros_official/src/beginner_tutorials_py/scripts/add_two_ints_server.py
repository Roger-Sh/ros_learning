#!/usr/bin/env python

from __future__ import print_function

from beginner_tutorials_py.srv import AddTwoInts ,AddTwoIntsResponse
import rospy

# callback of service
def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    # rosnode init
    rospy.init_node('add_two_ints_server')
    
    # service: add_two_ints, request: AddTwoInts, 
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()