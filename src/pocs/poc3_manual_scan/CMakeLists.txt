cmake_minimum_required(VERSION 3.16)
project(poc3_manual_scan)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Nodes go to lib/<pkg> for 'ros2 run'; their helper modules sit next to them
# (a script's own directory is on sys.path, see poc2_unity_camera).
install(PROGRAMS
  src/joy_flight.py
  src/scan_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  src/flight_math.py
  src/scan_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
