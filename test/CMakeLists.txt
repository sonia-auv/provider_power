catkin_add_gtest(config_test2 config_test2.cc ${provider_power_FILES})
target_link_libraries(config_test2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(config_test2 PROPERTIES LINKER_LANGUAGE CXX )