# CAUTION:
#  - You can specify only one .pyx file for each cython module.
#  - The name of the module and the .pyx file should be identical.
cython_add_module(add_two_ints add_two_ints.pyx ../add_two_ints.c)
cython_add_module(sub_two_ints sub_two_ints.pyx ../sub_two_ints.c)
cython_add_module(mul_two_ints mul_two_ints.pyx ../mul_two_ints.c)
cython_add_module(fibonacci fibonacci.pyx ../fibonacci.c)

# The library (libros_cython_example.so) is created in upper CMakeLists.txt
target_link_libraries(add_two_ints ros_cython_example)
target_link_libraries(sub_two_ints ros_cython_example)
target_link_libraries(mul_two_ints ros_cython_example)
target_link_libraries(fibonacci ros_cython_example)

# Set the targets to be copied in the python package directry.
# This example only place mul_two_ints and fibonacci modules.
# we can call C version of these and python version of the rest.
set_target_properties(mul_two_ints fibonacci
  PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

#install(TARGETS add_two_ints sub_two_ints
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

