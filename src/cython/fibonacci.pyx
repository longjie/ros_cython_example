cdef extern from "ros_cython_example/fibonacci.h":
    double c_fibonacci(double n)

def fibonacci(n):
    return c_fibonacci(n)
