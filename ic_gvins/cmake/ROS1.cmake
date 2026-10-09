find_package(catkin REQUIRED COMPONENTS
        roscpp sensor_msgs nav_msgs geometry_msgs tf2_ros)

catkin_package(CATKIN_DEPENDS
        roscpp sensor_msgs nav_msgs geometry_msgs tf2_ros)

add_executable(${PROJECT_NAME}_ros
        ROS/fusion_ros.cc
        ROS/drawer_rviz.cc)
set_target_properties(${PROJECT_NAME}_ros PROPERTIES INSTALL_RPATH "$ORIGIN/..")
target_compile_definitions(${PROJECT_NAME}_ros PRIVATE ROS_AVAILABLE=1)
target_include_directories(${PROJECT_NAME}_ros PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_ros PRIVATE ic_gvins_core ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
    add_dependencies(${PROJECT_NAME}_ros ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS ic_gvins_core ${PROJECT_NAME}_ros
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/ic_gvins.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY ../config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
