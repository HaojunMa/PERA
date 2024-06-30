execute_process(COMMAND "/home/haojun/Desktop/pera/build/pera/pera_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/haojun/Desktop/pera/build/pera/pera_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
