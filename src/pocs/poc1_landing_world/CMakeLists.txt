# 3.16 is the floor shipped by Ubuntu 20.04+ and avoids the deprecation
# warning CMake >= 3.27 emits for minimums below 3.10. Jazzy ships CMake 3.28.
cmake_minimum_required(VERSION 3.16)
project(poc1_landing_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Models
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# Install Worlds
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# --- CRITICAL PART: INSTALL THE PYTHON SCRIPT ---
# This copies the file to lib/poc1_landing_world/ so 'ros2 run' finds it
install(PROGRAMS
  src/mission_control.py
  DESTINATION lib/${PROJECT_NAME}
)
# ------------------------------------------------

ament_package()