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

# Produce verbose output by default.
VERBOSE = 1

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

# Utility rule file for xf_mic_asr_offline_generate_messages_nodejs.

# Include any custom commands dependencies for this target.
include xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/compiler_depend.make

# Include the progress variables for this target.
include xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/progress.make

xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/msg/Pcm_Msg.js
xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Offline_Result_srv.js
xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Start_Record_srv.js
xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Major_Mic_srv.js
xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Led_On_srv.js
xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Major_Mic_srv.js
xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Awake_Word_srv.js
xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.js

/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/msg/Pcm_Msg.js: /opt/ros/melodic/lib/gennodejs/gen_nodejs.py
/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/msg/Pcm_Msg.js: /home/ucar/catkin_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ucar/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating Javascript code from xf_mic_asr_offline/Pcm_Msg.msg"
	cd /home/ucar/catkin_ws/build/xf_mic_asr_offline && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gennodejs/cmake/../../../lib/gennodejs/gen_nodejs.py /home/ucar/catkin_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg -Ixf_mic_asr_offline:/home/ucar/catkin_ws/src/xf_mic_asr_offline/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p xf_mic_asr_offline -o /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/msg

/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.js: /opt/ros/melodic/lib/gennodejs/gen_nodejs.py
/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.js: /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ucar/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating Javascript code from xf_mic_asr_offline/Get_Awake_Angle_srv.srv"
	cd /home/ucar/catkin_ws/build/xf_mic_asr_offline && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gennodejs/cmake/../../../lib/gennodejs/gen_nodejs.py /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv -Ixf_mic_asr_offline:/home/ucar/catkin_ws/src/xf_mic_asr_offline/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p xf_mic_asr_offline -o /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv

/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Major_Mic_srv.js: /opt/ros/melodic/lib/gennodejs/gen_nodejs.py
/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Major_Mic_srv.js: /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ucar/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating Javascript code from xf_mic_asr_offline/Get_Major_Mic_srv.srv"
	cd /home/ucar/catkin_ws/build/xf_mic_asr_offline && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gennodejs/cmake/../../../lib/gennodejs/gen_nodejs.py /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv -Ixf_mic_asr_offline:/home/ucar/catkin_ws/src/xf_mic_asr_offline/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p xf_mic_asr_offline -o /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv

/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Offline_Result_srv.js: /opt/ros/melodic/lib/gennodejs/gen_nodejs.py
/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Offline_Result_srv.js: /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ucar/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating Javascript code from xf_mic_asr_offline/Get_Offline_Result_srv.srv"
	cd /home/ucar/catkin_ws/build/xf_mic_asr_offline && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gennodejs/cmake/../../../lib/gennodejs/gen_nodejs.py /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv -Ixf_mic_asr_offline:/home/ucar/catkin_ws/src/xf_mic_asr_offline/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p xf_mic_asr_offline -o /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv

/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Awake_Word_srv.js: /opt/ros/melodic/lib/gennodejs/gen_nodejs.py
/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Awake_Word_srv.js: /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ucar/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating Javascript code from xf_mic_asr_offline/Set_Awake_Word_srv.srv"
	cd /home/ucar/catkin_ws/build/xf_mic_asr_offline && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gennodejs/cmake/../../../lib/gennodejs/gen_nodejs.py /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv -Ixf_mic_asr_offline:/home/ucar/catkin_ws/src/xf_mic_asr_offline/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p xf_mic_asr_offline -o /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv

/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Led_On_srv.js: /opt/ros/melodic/lib/gennodejs/gen_nodejs.py
/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Led_On_srv.js: /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ucar/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating Javascript code from xf_mic_asr_offline/Set_Led_On_srv.srv"
	cd /home/ucar/catkin_ws/build/xf_mic_asr_offline && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gennodejs/cmake/../../../lib/gennodejs/gen_nodejs.py /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv -Ixf_mic_asr_offline:/home/ucar/catkin_ws/src/xf_mic_asr_offline/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p xf_mic_asr_offline -o /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv

/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Major_Mic_srv.js: /opt/ros/melodic/lib/gennodejs/gen_nodejs.py
/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Major_Mic_srv.js: /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ucar/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating Javascript code from xf_mic_asr_offline/Set_Major_Mic_srv.srv"
	cd /home/ucar/catkin_ws/build/xf_mic_asr_offline && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gennodejs/cmake/../../../lib/gennodejs/gen_nodejs.py /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv -Ixf_mic_asr_offline:/home/ucar/catkin_ws/src/xf_mic_asr_offline/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p xf_mic_asr_offline -o /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv

/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Start_Record_srv.js: /opt/ros/melodic/lib/gennodejs/gen_nodejs.py
/home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Start_Record_srv.js: /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ucar/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Generating Javascript code from xf_mic_asr_offline/Start_Record_srv.srv"
	cd /home/ucar/catkin_ws/build/xf_mic_asr_offline && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gennodejs/cmake/../../../lib/gennodejs/gen_nodejs.py /home/ucar/catkin_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv -Ixf_mic_asr_offline:/home/ucar/catkin_ws/src/xf_mic_asr_offline/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p xf_mic_asr_offline -o /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv

xf_mic_asr_offline_generate_messages_nodejs: xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs
xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/msg/Pcm_Msg.js
xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.js
xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Major_Mic_srv.js
xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Get_Offline_Result_srv.js
xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Awake_Word_srv.js
xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Led_On_srv.js
xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Set_Major_Mic_srv.js
xf_mic_asr_offline_generate_messages_nodejs: /home/ucar/catkin_ws/devel/share/gennodejs/ros/xf_mic_asr_offline/srv/Start_Record_srv.js
xf_mic_asr_offline_generate_messages_nodejs: xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/build.make
.PHONY : xf_mic_asr_offline_generate_messages_nodejs

# Rule to build all files generated by this target.
xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/build: xf_mic_asr_offline_generate_messages_nodejs
.PHONY : xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/build

xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/clean:
	cd /home/ucar/catkin_ws/build/xf_mic_asr_offline && $(CMAKE_COMMAND) -P CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/cmake_clean.cmake
.PHONY : xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/clean

xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/depend:
	cd /home/ucar/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ucar/catkin_ws/src /home/ucar/catkin_ws/src/xf_mic_asr_offline /home/ucar/catkin_ws/build /home/ucar/catkin_ws/build/xf_mic_asr_offline /home/ucar/catkin_ws/build/xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : xf_mic_asr_offline/CMakeFiles/xf_mic_asr_offline_generate_messages_nodejs.dir/depend

