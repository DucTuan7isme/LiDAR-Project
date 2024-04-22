cmake_minimum_required(VERSION 3.5)
project(olelidar)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11 -g ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS 
  roscpp
  diagnostic_updater
  dynamic_reconfigure
  sensor_msgs
  olei_msgs
  std_msgs
  genmsg
  geometry_msgs

)
  

generate_dynamic_reconfigure_options(cfg/oleiPuck.cfg)



find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})




#add_executable(${PROJECT_NAME}_driver src/driver.cpp)
add_library(${PROJECT_NAME}_driver src/driver.cpp)
target_include_directories(${PROJECT_NAME}_driver PUBLIC ${catkin_INCLUDE_DIRS} src)
target_link_libraries(${PROJECT_NAME}_driver PUBLIC ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_decoder src/decoder.cpp)
target_include_directories(${PROJECT_NAME}_decoder PUBLIC ${catkin_INCLUDE_DIRS} src)
target_link_libraries(${PROJECT_NAME}_decoder PUBLIC ${catkin_LIBRARIES} ${PROJECT_NAME}_driver)


add_executable(subscriber_node src/subscriber_node.cpp)
target_include_directories(subscriber_node  PUBLIC ${catkin_INCLUDE_DIRS} src)
target_link_libraries(subscriber_node  PUBLIC ${catkin_LIBRARIES} )


add_executable(shape_detection src/shape_detection.cpp)
target_include_directories(shape_detection   PUBLIC ${catkin_INCLUDE_DIRS} src)
target_link_libraries(shape_detection   PUBLIC ${catkin_LIBRARIES} )

add_executable(denoise src/denoise.cpp)
target_include_directories(denoise   PUBLIC ${catkin_INCLUDE_DIRS} src)
target_link_libraries(denoise   PUBLIC ${catkin_LIBRARIES} )


add_executable(circle_regression src/circle_regression.cpp)
target_include_directories(circle_regression   PUBLIC ${catkin_INCLUDE_DIRS} src)
target_link_libraries(circle_regression   PUBLIC ${catkin_LIBRARIES} )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_driver ${PROJECT_NAME}_decoder  subscriber_node  shape_detection  denoise  circle_regression
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(FILES
  cfg/oleiPuck.cfg
  launch/decoder.launch
  launch/driver.launch
  launch/scan.launch
  # launch/olelidar.launch
  launch/debug.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

