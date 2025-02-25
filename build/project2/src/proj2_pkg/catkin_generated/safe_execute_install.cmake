execute_process(COMMAND "/home/caitlin/project2/build/project2/src/proj2_pkg/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/caitlin/project2/build/project2/src/proj2_pkg/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
