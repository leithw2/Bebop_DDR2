# CMake generated Testfile for 
# Source directory: /home/laptop/catkin_ws/src/grid_map-master/grid_map_core
# Build directory: /home/laptop/catkin_ws/build/grid_map_core
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_grid_map_core_gtest_grid_map_core-test "/home/laptop/catkin_ws/build/grid_map_core/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/laptop/catkin_ws/build/grid_map_core/test_results/grid_map_core/gtest-grid_map_core-test.xml" "--return-code" "/home/laptop/catkin_ws/devel/.private/grid_map_core/lib/grid_map_core/grid_map_core-test --gtest_output=xml:/home/laptop/catkin_ws/build/grid_map_core/test_results/grid_map_core/gtest-grid_map_core-test.xml")
set_tests_properties(_ctest_grid_map_core_gtest_grid_map_core-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/laptop/catkin_ws/src/grid_map-master/grid_map_core/CMakeLists.txt;113;catkin_add_gtest;/home/laptop/catkin_ws/src/grid_map-master/grid_map_core/CMakeLists.txt;0;")
subdirs("gtest")
