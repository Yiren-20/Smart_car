# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/ucar/.local/lib/python3.6/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/ucar/.local/lib/python3.6/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ucar/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ucar/catkin_ws/build

# Utility rule file for clean_test_results_darknet_ros.

# Include any custom commands dependencies for this target.
include yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/compiler_depend.make

# Include the progress variables for this target.
include yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/progress.make

yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros:
	cd /home/ucar/catkin_ws/build/yolov4-for-darknet_ros/darknet_ros/darknet_ros && /usr/bin/python2 /opt/ros/melodic/share/catkin/cmake/test/remove_test_results.py /home/ucar/catkin_ws/build/test_results/darknet_ros

clean_test_results_darknet_ros: yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros
clean_test_results_darknet_ros: yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/build.make
.PHONY : clean_test_results_darknet_ros

# Rule to build all files generated by this target.
yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/build: clean_test_results_darknet_ros
.PHONY : yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/build

yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/clean:
	cd /home/ucar/catkin_ws/build/yolov4-for-darknet_ros/darknet_ros/darknet_ros && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_darknet_ros.dir/cmake_clean.cmake
.PHONY : yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/clean

yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/depend:
	cd /home/ucar/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ucar/catkin_ws/src /home/ucar/catkin_ws/src/yolov4-for-darknet_ros/darknet_ros/darknet_ros /home/ucar/catkin_ws/build /home/ucar/catkin_ws/build/yolov4-for-darknet_ros/darknet_ros/darknet_ros /home/ucar/catkin_ws/build/yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/depend

