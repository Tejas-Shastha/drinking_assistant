cmake_minimum_required(VERSION 2.8.3)
project(drinking_assistant)

add_compile_options(-std=c++11 -O3 -DUSE_AVX_INSTRUCTIONS=ON -fopenmp) # -DCMAKE_BUILD_TYPE=Release

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	roscpp
	sensor_msgs
	geometry_msgs
	message_generation
	tf
        tf_conversions
        audio_emergency
        trac_ik_lib
        orocos_kdl
        kdl_conversions
)


add_service_files(FILES
  EnvStep.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
		CATKIN_DEPENDS
                    message_runtime
                    actionlib_msgs
                    trac_ik_lib
                DEPENDS
                    Boost
                    orocos_kdl
)


include_directories(
	${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${orocos_kdl_INCLUDE_DIRS}
)


add_executable(velocity_test src/velocity_test.cpp)
add_dependencies(velocity_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(velocity_test ${catkin_LIBRARIES})

add_executable(position_drive src/position_drive.cpp)
add_dependencies(position_drive ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(position_drive ${catkin_LIBRARIES})

add_executable(robot_control2 src/robot_control.cpp)
add_dependencies(robot_control2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robot_control2 ${catkin_LIBRARIES})

add_executable(feeder_position_control src/feeder_position_control.cpp)
add_dependencies(feeder_position_control ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(feeder_position_control	${catkin_LIBRARIES})

add_executable(feeder_straight src/feeder_straight.cpp)
add_dependencies(feeder_straight ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(feeder_straight	${catkin_LIBRARIES})

add_executable(feeder_policy_drive src/feeder_policy_drive.cpp)
add_dependencies(feeder_policy_drive ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(feeder_policy_drive	${catkin_LIBRARIES})

add_executable(feeder_straight_1D src/feeder_straight_1D.cpp)
add_dependencies(feeder_straight_1D ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(feeder_straight_1D	${catkin_LIBRARIES})

add_executable(feeder_sideways_1D src/feeder_sideways_1D.cpp)
add_dependencies(feeder_sideways_1D ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(feeder_sideways_1D	${catkin_LIBRARIES})

add_executable(feeder_sideways src/feeder_sideways.cpp)
add_dependencies(feeder_sideways ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(feeder_sideways	${catkin_LIBRARIES})

add_executable(feeder_gmm src/feeder_gmm.cpp)
add_dependencies(feeder_gmm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(feeder_gmm	${catkin_LIBRARIES})

add_executable(tf_caster src/tf_caster.cpp)
add_dependencies(tf_caster ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tf_caster	${catkin_LIBRARIES})

add_executable(velocity_drive src/velocity_drive.cpp)
add_dependencies(velocity_drive ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(velocity_drive ${catkin_LIBRARIES})

add_executable(quaternion_tester src/quaternion_tester.cpp)
add_dependencies(quaternion_tester ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(quaternion_tester ${catkin_LIBRARIES})

add_executable(data_extractor src/data_extractor.cpp)
add_dependencies(data_extractor ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(data_extractor ${catkin_LIBRARIES})

add_executable(data_extractor_gmm src/data_extractor_gmm.cpp)
add_dependencies(data_extractor_gmm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(data_extractor_gmm ${catkin_LIBRARIES})

add_executable(ik_tests src/ik_tests.cpp)
target_link_libraries(ik_tests
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
