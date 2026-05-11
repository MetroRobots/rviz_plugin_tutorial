cmake_minimum_required(VERSION 3.5)
project(rviz_plugin_tutorial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Pre-probe both Qt versions with components so componentless re-lookups
# inside rviz_common's exported dependency file are satisfied via cache.
find_package(Qt6 QUIET COMPONENTS Widgets)
find_package(Qt5 QUIET COMPONENTS Widgets)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_plugin_tutorial_msgs REQUIRED)
find_package(rviz_rendering REQUIRED)

# Match the Qt version that rviz_common actually links against. Inspect
# its link interface — Qt6 preferred, Qt5 fallback.
get_target_property(_rviz_common_link rviz_common::rviz_common INTERFACE_LINK_LIBRARIES)
if("${_rviz_common_link}" MATCHES "Qt6::")
  set(QT_VERSION_MAJOR 6)
elseif("${_rviz_common_link}" MATCHES "Qt5::")
  set(QT_VERSION_MAJOR 5)
elseif(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
elseif(Qt5_FOUND)
  set(QT_VERSION_MAJOR 5)
else()
  message(FATAL_ERROR "Neither Qt6 nor Qt5 was found")
endif()

set(CMAKE_AUTOMOC ON)
if(QT_VERSION_MAJOR EQUAL 6)
  qt6_wrap_cpp(MOC_FILES
    include/rviz_plugin_tutorial/point_display.hpp
  )
else()
  qt5_wrap_cpp(MOC_FILES
    include/rviz_plugin_tutorial/point_display.hpp
  )
endif()

add_library(point_display src/point_display.cpp ${MOC_FILES})
target_include_directories(point_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(point_display PUBLIC
  ${rviz_plugin_tutorial_msgs_TARGETS}
  pluginlib::pluginlib
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
)
install(TARGETS point_display
        EXPORT export_rviz_plugin_tutorial
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/Point2D.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_rviz_plugin_tutorial)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
