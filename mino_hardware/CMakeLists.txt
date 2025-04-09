cmake_minimum_required(VERSION 3.8)
project(mino_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)


# Добавляем библиотеку
add_library(${PROJECT_NAME}
  src/mino_hardware.cpp
  src/mino_motors.cpp
)

# Подключаем зависимости
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Добавляем библиотеку WiringPi
target_link_libraries(${PROJECT_NAME} -lwiringPi)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

# Устанавливаем библиотеку
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Устанавливаем заголовочные файлы
install(DIRECTORY include/
  DESTINATION include
)

# Экспортируем библиотеку для использования другими пакетами
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

# Регистрируем плагин
pluginlib_export_plugin_description_file(hardware_interface mino_hardware.xml)

ament_package()