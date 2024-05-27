set(MocapRosUtils_gtest_list TestBvhConverter)

foreach(NAME IN LISTS MocapRosUtils_gtest_list)
  catkin_add_gtest(${NAME} src/${NAME}.cpp)
  target_link_libraries(${NAME} MocapRosUtils)
endforeach()
