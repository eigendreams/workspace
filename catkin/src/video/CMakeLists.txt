cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS cv_bridge std_msgs roscpp tf geometry_msgs sensor_msgs image_transport nav_msgs)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(detector_humanos src/detector_humanos.cpp)
target_link_libraries(detector_humanos ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(conversor_cv src/conversor_cv.cpp)
target_link_libraries(conversor_cv ${catkin_LIBRARIES})

add_executable(data_imu src/data_imu.cpp)
target_link_libraries(data_imu ${catkin_LIBRARIES})
