# ros_cython_example

## What is this?

This package is an example of ROS package with cython. Cython is an
great framework to couple python and C very easily.

ROS nodes written in python are sometimes suffered from performance
problem, and cython can be a solution.

This package is simplified and self-contained, so I think it is easy
to re-use for your onw ROS package.

## Quick start

On a terminal:

```
$ roslaunch ros_cython_example server.launch
```

On another terminal:

```
$ rosservice list

$ rosservice call /add_two_ints  1 2
result: 3
```

Now the first terminal screen says the server invoked the function
'add_two_ints' of python version (src/ros_cython_example/add_two_ints.py).

Try another service call as:
```
$ rosservice call /fibonacci 10
result: 55.0
```

Now the first terminal screen says the server invoked the function
'fibonacci' of C version(src/fibonacci.c). The C code are called from
python by cython framework.

You can easily switch the function version between C and python, by
modifing the CMakeLists.txt and rebuild the package.

## Contents of the package

The package contains a server written in python (script/server.py). The
server has four services:

- add_two_ints
- sub_two_ints
- mul_two_ints
- fibonacci

Each service is correspoinding to a function, and the functions are
(originally) written in python files in the package directory
(src/ros_cython_example).

Assume we want to re-implement 'mul_two_ints' and 'fibonacci' in C
because of performance reason. The functions are implemented in C in files:

- src/mul_two_ints.c
- src/fibonacci.c

These C modules are compiled as a shared library
(libros_cython_example.so), by CMakeLists.txt on the top level.

You need cython wrapper codes to call the C functions from python. The
wrapper codes are src/cython/*.pyx.

For example, mul_two_ints.pyx is:

```
cdef extern from "ros_cython_example/mul_two_ints.h":
    int c_mul_two_ints(int a, int b)

def mul_two_ints(a, b):
    return c_mul_two_ints(a, b)
```

The cmake files (src/cython/CMakeLists.txt and top level
CMakeLists.txt) specify how to compile cython codes.

The contents of cmake/ directory is from
[cython_cmake_example](https://github.com/thewtex/cython-cmake-example)
project.

## Thanks to

* [cython_catkin_example](https://github.com/marcoesposito1988/cython_catkin_example)
  by @marcoesposito1988
* [cython_cmake_example](https://github.com/thewtex/cython-cmake-example)
  by @thewtex

## Memo

This package owes much to the
[cython_catkin_example](https://github.com/marcoesposito1988/cython_catkin_example)
project. I just want to show another simplified and extendable example
to fit actual ROS usage.

In this example, I assume fairly standard ROS package contents as follows:

```
ros_catkin_example/
    cmake/ <-- cmake module files for build cython files
    CMakeLists.txt <-- for top level catkin_make
    setup.py <-- for catkin_make
    include/cython_catkin_example <-- header files for c/c++
    src/ <-- source files 
        ros_catkin_example/ <-- python package source files
	    cython/ <-- cython source files (*.pyx, etc)
		    CMakeLists.txt <-- for building cython, included from top level
```
