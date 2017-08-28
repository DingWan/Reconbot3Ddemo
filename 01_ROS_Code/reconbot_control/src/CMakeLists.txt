add_executable(move_group_interface move_group_interface.cpp)
target_link_libraries(move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(r_arm_move_group_interface r_arm_move_group_interface.cpp)
target_link_libraries(r_arm_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS r_arm_move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(l_arm_move_group_interface l_arm_move_group_interface.cpp)
target_link_libraries(l_arm_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS l_arm_move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(basic_arm basic_arm.cpp)
target_link_libraries(basic_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS basic_arm DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(trajectory_publisher trajectory_publisher.cpp)
target_link_libraries(trajectory_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS trajectory_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(r_arm_trajectory_publisher r_arm_trajectory_publisher.cpp)
target_link_libraries(r_arm_trajectory_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS r_arm_trajectory_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(l_arm_trajectory_publisher l_arm_trajectory_publisher.cpp)
target_link_libraries(l_arm_trajectory_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS l_arm_trajectory_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(torque_enable_client torque_enable_client.cpp)
target_link_libraries(torque_enable_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS torque_enable_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(sensor_data_capture sensor_data_capture.cpp)
target_link_libraries(sensor_data_capture ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS sensor_data_capture DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
