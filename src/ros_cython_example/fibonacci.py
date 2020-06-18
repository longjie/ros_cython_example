#!/usr/bin/env python
import sys

def fibonacci(n):
    print ("This is python version.")
    a, b = 0.0, 1.0
    for i in range(n):
        a, b = a + b, a
    return a
