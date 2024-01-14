add_executable(task src/task.cpp)
target_link_libraries(task ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS task DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})