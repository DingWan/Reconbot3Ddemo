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

add_executable(T3R_trajectory_publisher T3R_trajectory_publisher.cpp)
target_link_libraries(T3R_trajectory_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS T3R_trajectory_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(T3R_move_group_interface T3R_move_group_interface.cpp)
target_link_libraries(T3R_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS T3R_move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



add_executable(RA2C2_trajectory_publisher RA2C2_trajectory_publisher.cpp)
target_link_libraries(RA2C2_trajectory_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS RA2C2_trajectory_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(RA2C2_move_group_interface RA2C2_move_group_interface.cpp)
target_link_libraries(RA2C2_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS RA2C2_move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(T2R5_trajectory_publisher T2R5_trajectory_publisher.cpp)
target_link_libraries(T2R5_trajectory_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS T2R5_trajectory_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(T2R5_move_group_interface T2R5_move_group_interface.cpp)
target_link_libraries(T2R5_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS T2R5_move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(T1R_trajectory_publisher T1R_trajectory_publisher.cpp)
target_link_libraries(T1R_trajectory_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS T1R_trajectory_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(T1R_move_group_interface T1R_move_group_interface.cpp)
target_link_libraries(T1R_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS T1R_move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
