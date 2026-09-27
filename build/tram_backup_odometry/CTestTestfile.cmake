# CMake generated Testfile for 
# Source directory: /home/ros/tram_ws/src/tram_backup_odometry
# Build directory: /home/ros/tram_ws/build/tram_backup_odometry
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_core "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ros/tram_ws/build/tram_backup_odometry/test_results/tram_backup_odometry/test_core.xunit.xml" "--package-name" "tram_backup_odometry" "--output-file" "/home/ros/tram_ws/build/tram_backup_odometry/ament_cmake_pytest/test_core.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/ros/tram_ws/src/tram_backup_odometry/test/test_core.py" "-o" "cache_dir=/home/ros/tram_ws/build/tram_backup_odometry/ament_cmake_pytest/test_core/.cache" "--junit-xml=/home/ros/tram_ws/build/tram_backup_odometry/test_results/tram_backup_odometry/test_core.xunit.xml" "--junit-prefix=tram_backup_odometry")
set_tests_properties(test_core PROPERTIES  LABELS "pytest" TIMEOUT "300" WORKING_DIRECTORY "/home/ros/tram_ws/src/tram_backup_odometry" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/home/ros/tram_ws/src/tram_backup_odometry/CMakeLists.txt;38;ament_add_pytest_test;/home/ros/tram_ws/src/tram_backup_odometry/CMakeLists.txt;0;")
