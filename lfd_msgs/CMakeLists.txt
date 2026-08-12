cmake_minimum_required(VERSION 3.5)
project(lfd_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteSkill.action"
  "action/HomeRobot.action"
  "action/RecordSkill.action"
  "action/RecordTemplate.action"
  "action/ReserveRobot.action"
  "msg/OperationStatus.msg"
  "srv/ComputeLocalization.srv"
  "srv/FinishRecording.srv"
  "srv/Heartbeat.srv"
  "srv/ReleaseReservation.srv"
  "srv/SetTemplate.srv"
  "srv/StringService.srv"
  "srv/SubmitTemplateCrop.srv"
  "srv/GetScene.srv"
  
  DEPENDENCIES action_msgs builtin_interfaces std_msgs sensor_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
