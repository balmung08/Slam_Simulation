cmake_minimum_required(VERSION 3.0.2)
project(car_ctr)


find_package(catkin REQUIRED)



find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  tf
  common
)

add_message_files(FILES
  car_state_msgs.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES car_ctr
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)



add_executable(car_ctr
  src/car_ctr.cpp
)
add_dependencies(car_ctr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(car_ctr
  ${catkin_LIBRARIES}
)

add_executable(spd_ctr
  src/spd_ctr.cpp
)
add_dependencies(spd_ctr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spd_ctr
  ${catkin_LIBRARIES}
)




find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
   src/panel/carctr_panel.h
)

set(GUI_SOURCE_FILES
  src/panel/carctr_panel.cpp 
  ${MOC_FILES}
)
add_library(carctr_gui ${GUI_SOURCE_FILES})
target_link_libraries(carctr_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  carctr_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
