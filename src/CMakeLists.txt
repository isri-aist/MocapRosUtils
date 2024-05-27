add_library(MocapRosUtils SHARED JointTrajData.cpp BvhConverter.cpp)
target_include_directories(MocapRosUtils PUBLIC ${PROJECT_SOURCE_DIR}/include
                                                ${catkin_INCLUDE_DIRS})
target_link_libraries(MocapRosUtils PUBLIC RBDyn::RBDyn RBDyn::Parsers
                                           ${catkin_LIBRARIES})
