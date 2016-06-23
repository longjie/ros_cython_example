cdef extern from "ros_cython_example/add_two_ints.h":
    int c_add_two_ints(int a, int b)

def add_two_ints(a, b):
    return c_add_two_ints(a, b)
