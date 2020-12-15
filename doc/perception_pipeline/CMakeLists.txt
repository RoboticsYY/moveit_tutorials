# add_executable(cylinder_segment src/cylinder_segment.cpp)
# target_link_libraries(cylinder_segment ${catkin_LIBRARIES})

# add_executable(bag_publisher_maintain_time src/bag_publisher_maintain_time.cpp)
# target_link_libraries(bag_publisher_maintain_time ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcd_publisher src/pcd_publisher.cpp)
ament_target_dependencies(pcd_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(pcd_publisher ${PCL_LIBRARIES})

install(
TARGETS
    # bag_publisher_maintain_time
    # cylinder_segment
    pcd_publisher
DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY data DESTINATION bin)
