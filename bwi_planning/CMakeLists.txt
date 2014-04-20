cmake_minimum_required(VERSION 2.8.3)
project(bwi_planning)

find_package(catkin REQUIRED COMPONENTS
  bwi_planning_common
  message_generation
  roscpp
  std_msgs
  std_srvs
)
find_package(Boost REQUIRED)

catkin_python_setup()

add_service_files(
  FILES
  CostLearnerInterface.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  DEPENDS
    Boost
  CATKIN_DEPENDS
    bwi_planning_common
    message_runtime
    roscpp
    std_msgs
    std_srvs
  INCLUDE_DIRS
    include/
  LIBRARIES
    ${PROJECT_NAME}
)


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/libbwi_planning/cost_learner.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cost_learner src/nodes/cost_learner.cpp)
target_link_libraries(cost_learner ${PROJECT_NAME})

# Install the plugin and the plugin description
install (DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install the libary
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the node
install(TARGETS cost_learner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# unit tests are enabled for roslaunch files
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
