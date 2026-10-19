cmake_minimum_required(VERSION 3.8)
project(rehab_arm_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/rehab_exercise.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rehab_exercise
)

install(
  PROGRAMS scripts/exercise_metrics.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME exercise_metrics
)

install(
  PROGRAMS scripts/rehab_session.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rehab_session
)

install(
  PROGRAMS scripts/safety_command_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME safety_command_gate
)

install(
  FILES scripts/safety_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
