# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tg/slam_mono/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tg/slam_mono/build

# Include any dependencies generated for this target.
include feature_tracker/CMakeFiles/myslam.dir/depend.make

# Include the progress variables for this target.
include feature_tracker/CMakeFiles/myslam.dir/progress.make

# Include the compile flags for this target's objects.
include feature_tracker/CMakeFiles/myslam.dir/flags.make

feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o: feature_tracker/CMakeFiles/myslam.dir/flags.make
feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o: /home/tg/slam_mono/src/feature_tracker/src/feature_tracker.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tg/slam_mono/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o"
	cd /home/tg/slam_mono/build/feature_tracker && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/src/feature_tracker.cc.o -c /home/tg/slam_mono/src/feature_tracker/src/feature_tracker.cc

feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/src/feature_tracker.cc.i"
	cd /home/tg/slam_mono/build/feature_tracker && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tg/slam_mono/src/feature_tracker/src/feature_tracker.cc > CMakeFiles/myslam.dir/src/feature_tracker.cc.i

feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/src/feature_tracker.cc.s"
	cd /home/tg/slam_mono/build/feature_tracker && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tg/slam_mono/src/feature_tracker/src/feature_tracker.cc -o CMakeFiles/myslam.dir/src/feature_tracker.cc.s

feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o.requires:

.PHONY : feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o.requires

feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o.provides: feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o.requires
	$(MAKE) -f feature_tracker/CMakeFiles/myslam.dir/build.make feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o.provides.build
.PHONY : feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o.provides

feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o.provides.build: feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o


# Object files for target myslam
myslam_OBJECTS = \
"CMakeFiles/myslam.dir/src/feature_tracker.cc.o"

# External object files for target myslam
myslam_EXTERNAL_OBJECTS =

/home/tg/slam_mono/devel/lib/libmyslam.so: feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o
/home/tg/slam_mono/devel/lib/libmyslam.so: feature_tracker/CMakeFiles/myslam.dir/build.make
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/libcv_bridge.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/libmessage_filters.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/libroscpp.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/librosconsole.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/librostime.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/libcpp_common.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
/home/tg/slam_mono/devel/lib/libmyslam.so: feature_tracker/CMakeFiles/myslam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tg/slam_mono/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/tg/slam_mono/devel/lib/libmyslam.so"
	cd /home/tg/slam_mono/build/feature_tracker && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myslam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
feature_tracker/CMakeFiles/myslam.dir/build: /home/tg/slam_mono/devel/lib/libmyslam.so

.PHONY : feature_tracker/CMakeFiles/myslam.dir/build

feature_tracker/CMakeFiles/myslam.dir/requires: feature_tracker/CMakeFiles/myslam.dir/src/feature_tracker.cc.o.requires

.PHONY : feature_tracker/CMakeFiles/myslam.dir/requires

feature_tracker/CMakeFiles/myslam.dir/clean:
	cd /home/tg/slam_mono/build/feature_tracker && $(CMAKE_COMMAND) -P CMakeFiles/myslam.dir/cmake_clean.cmake
.PHONY : feature_tracker/CMakeFiles/myslam.dir/clean

feature_tracker/CMakeFiles/myslam.dir/depend:
	cd /home/tg/slam_mono/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tg/slam_mono/src /home/tg/slam_mono/src/feature_tracker /home/tg/slam_mono/build /home/tg/slam_mono/build/feature_tracker /home/tg/slam_mono/build/feature_tracker/CMakeFiles/myslam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : feature_tracker/CMakeFiles/myslam.dir/depend

