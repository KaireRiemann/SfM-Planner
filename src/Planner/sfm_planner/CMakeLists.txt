cmake_minimum_required(VERSION 3.5)
project(sfm_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=return-type)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=unused-but-set-variable)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DUSE_ROS1)

message(STATUS "SfM-Planner: independent state2state corridor workspace")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        sensor_msgs
        message_generation
        quadrotor_msgs
        visualization_msgs
        message_filters
        cv_bridge
        map_manager
        traj_utils
        tf
)

add_message_files(
        FILES
        FaceObservation.msg
        FaceDetectionRequest.msg
        CaptureRequest.msg
        CaptureResult.msg
        MissionStatus.msg
)

add_service_files(
        FILES
        StartInspection.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex iostreams)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sfm_planner_core
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs
                       visualization_msgs message_filters cv_bridge map_manager
                       traj_utils tf quadrotor_msgs message_runtime
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        yaml-cpp
        -ldw
)

# Full state2state chain (corridor + traj_opt/fast_lbfgs + warm start).
# Exploration sources are not present in this tree.
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
# Orphaned / header-only moved units kept out of the library, same as general_planner.
list(FILTER srcs EXCLUDE REGEX ".*/src/traj_opt/exp_traj_optimizer_s4\\.cpp$")
list(FILTER srcs EXCLUDE REGEX ".*/src/traj_opt/backup_traj_optimizer_s4\\.cpp$")
list(FILTER srcs EXCLUDE REGEX ".*/src/traj_opt/yaw_traj_opt\\.cpp$")
list(FILTER srcs EXCLUDE REGEX ".*/src/utils/minco\\.cpp$")
list(REMOVE_DUPLICATES srcs)

add_library(sfm_planner_core STATIC ${srcs})
target_include_directories(sfm_planner_core PUBLIC include)
add_dependencies(sfm_planner_core
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(sfm_planner_core PUBLIC
        ${THIRD_PARTY}
        OpenMP::OpenMP_CXX
)

add_executable(fsm_node Apps/fsm_node_ros1.cpp)
add_dependencies(fsm_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fsm_node
        sfm_planner_core
        ${THIRD_PARTY}
)

add_executable(inspection_mock_node Apps/inspection_mock_node.cpp)
add_dependencies(inspection_mock_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(inspection_mock_node
        ${catkin_LIBRARIES}
)

add_executable(state2state_topology_route_self_test
        tests/state2state_topology_route_self_test.cpp
)
target_include_directories(state2state_topology_route_self_test PRIVATE
        include
        ${Eigen3_INCLUDE_DIRS}
)

add_executable(face_extent_self_test
        Tests/face_extent_self_test.cpp
)
add_dependencies(face_extent_self_test
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(face_extent_self_test
        sfm_planner_core
        ${THIRD_PARTY}
)

add_executable(inspection_face_recenter_self_test
        Tests/inspection_face_recenter_self_test.cpp
)
add_dependencies(inspection_face_recenter_self_test
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(inspection_face_recenter_self_test
        sfm_planner_core
        ${THIRD_PARTY}
)

install(TARGETS fsm_node sfm_planner_core inspection_mock_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
