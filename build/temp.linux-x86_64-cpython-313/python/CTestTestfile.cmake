# CMake generated Testfile for 
# Source directory: /home/gufei/YDLidar-SDK/python
# Build directory: /home/gufei/YDLidar-SDK/build/temp.linux-x86_64-cpython-313/python
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ydlidar_py_test "/home/gufei/miniconda3/bin/python3.13" "/home/gufei/YDLidar-SDK/python/test/pytest.py")
set_tests_properties(ydlidar_py_test PROPERTIES  ENVIRONMENT "PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/home/gufei/YDLidar-SDK/build/temp.linux-x86_64-cpython-313/python" _BACKTRACE_TRIPLES "/home/gufei/YDLidar-SDK/python/CMakeLists.txt;42;add_test;/home/gufei/YDLidar-SDK/python/CMakeLists.txt;0;")
subdirs("examples")
