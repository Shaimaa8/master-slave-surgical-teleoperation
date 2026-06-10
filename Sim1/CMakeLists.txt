cmake_minimum_required(VERSION 3.8)
project(surgical_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# 1. تثبيت المجلدات الوصفية (URDF & Launch) في مجلد share
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# 2. تثبيت ملفات البايثون في مجلد lib (عشان الـ Launch يقدر يشوفها)
# تثبيت ملفات البايثون كـ برامج تنفيذية
install(PROGRAMS
  scripts/teleop_bridge.py
  scripts/dashboard_node.py
  scripts/marker_control.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
