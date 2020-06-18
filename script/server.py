#!/usr/bin/env python
import rospy
from ros_cython_example import *
from ros_cython_example.srv import *

def handle_add_two_ints(req):
    return AddTwoIntsResponse(add_two_ints(req.a, req.b))

def handle_sub_two_ints(req):
    return SubTwoIntsResponse(sub_two_ints(req.a, req.b))

def handle_mul_two_ints(req):
    return MulTwoIntsResponse(mul_two_ints(req.a, req.b))

def handle_fibonacci(req):
    return FibonacciResponse(fibonacci(req.n))

if __name__ == "__main__":
    rospy.init_node('server')
    rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    rospy.Service('sub_two_ints', SubTwoInts, handle_sub_two_ints)
    rospy.Service('mul_two_ints', MulTwoInts, handle_mul_two_ints)
    rospy.Service('fibonacci', Fibonacci, handle_fibonacci)
    print("Server is ready.")
    rospy.spin()

