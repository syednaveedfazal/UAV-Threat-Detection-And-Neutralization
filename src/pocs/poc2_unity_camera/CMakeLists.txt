cmake_minimum_required(VERSION 3.16)
project(poc2_unity_camera)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Executables go to lib/<pkg> so 'ros2 run' finds them. The protocol module is
# installed next to them: Python puts a script's own directory on sys.path, so
# a plain 'import unity_bridge_protocol' works without an ament_python package.
install(PROGRAMS
  src/unity_bridge.py
  src/fake_unity_client.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  src/unity_bridge_protocol.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
