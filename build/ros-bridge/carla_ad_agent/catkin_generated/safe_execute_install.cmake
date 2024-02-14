execute_process(COMMAND "/home/maanz-ai/ahmed-workspace/build/ros-bridge/carla_ad_agent/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/maanz-ai/ahmed-workspace/build/ros-bridge/carla_ad_agent/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
