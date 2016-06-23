cdef extern from "ros_cython_example/mul_two_ints.h":
    int c_mul_two_ints(int a, int b)

def mul_two_ints(a, b):
    return c_mul_two_ints(a, b)
