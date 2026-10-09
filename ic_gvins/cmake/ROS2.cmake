find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(${PROJECT_NAME}_ros
        ROS/fusion_ros.cc
        ROS/drawer_rviz.cc)
set_target_properties(${PROJECT_NAME}_ros PROPERTIES INSTALL_RPATH "$ORIGIN/..")
target_compile_definitions(${PROJECT_NAME}_ros PRIVATE ROS_AVAILABLE=2)
target_link_libraries(${PROJECT_NAME}_ros PRIVATE
        ic_gvins_core
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        tf2_ros::tf2_ros)

install(TARGETS ic_gvins_core ${PROJECT_NAME}_ros
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES launch/ic_gvins.launch.py launch/ic_gvins.launch.xml
        DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY ../config/
        DESTINATION share/${PROJECT_NAME}/config)

ament_package()
