# CMake generated Testfile for 
# Source directory: /home/laptop/catkin_ws/src/grid_map-master/grid_map_costmap_2d
# Build directory: /home/laptop/catkin_ws/build/grid_map_costmap_2d
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_grid_map_costmap_2d_gtest_grid_map_costmap_2d-test "/home/laptop/catkin_ws/build/grid_map_costmap_2d/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/laptop/catkin_ws/build/grid_map_costmap_2d/test_results/grid_map_costmap_2d/gtest-grid_map_costmap_2d-test.xml" "--return-code" "/home/laptop/catkin_ws/devel/.private/grid_map_costmap_2d/lib/grid_map_costmap_2d/grid_map_costmap_2d-test --gtest_output=xml:/home/laptop/catkin_ws/build/grid_map_costmap_2d/test_results/grid_map_costmap_2d/gtest-grid_map_costmap_2d-test.xml")
set_tests_properties(_ctest_grid_map_costmap_2d_gtest_grid_map_costmap_2d-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/laptop/catkin_ws/src/grid_map-master/grid_map_costmap_2d/CMakeLists.txt;70;catkin_add_gtest;/home/laptop/catkin_ws/src/grid_map-master/grid_map_costmap_2d/CMakeLists.txt;0;")
subdirs("gtest")
subdirs("rostest")
