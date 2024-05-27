set(MocapRosUtils_node_list GenerateUrdfFromBvh PublishJointStateFromBvh)

foreach(NAME IN LISTS MocapRosUtils_node_list)
  add_executable(${NAME} ${NAME}.cpp)
  target_link_libraries(${NAME} PRIVATE MocapRosUtils)
endforeach()
