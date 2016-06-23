cdef extern from "ros_cython_example/sub_two_ints.h":
    int c_sub_two_ints(int a, int b)

def sub_two_ints(a, b):
    return c_sub_two_ints(a, b)
