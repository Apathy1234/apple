# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "feature_tracker: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ifeature_tracker:/home/sfox/slam_mono/src/feature_tracker/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(feature_tracker_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg" NAME_WE)
add_custom_target(_feature_tracker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "feature_tracker" "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg" ""
)

get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg" NAME_WE)
add_custom_target(_feature_tracker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "feature_tracker" "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg" "feature_tracker/FeatureTrackerResult:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(feature_tracker
  "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feature_tracker
)
_generate_msg_cpp(feature_tracker
  "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg"
  "${MSG_I_FLAGS}"
  "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feature_tracker
)

### Generating Services

### Generating Module File
_generate_module_cpp(feature_tracker
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feature_tracker
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(feature_tracker_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(feature_tracker_generate_messages feature_tracker_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg" NAME_WE)
add_dependencies(feature_tracker_generate_messages_cpp _feature_tracker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg" NAME_WE)
add_dependencies(feature_tracker_generate_messages_cpp _feature_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(feature_tracker_gencpp)
add_dependencies(feature_tracker_gencpp feature_tracker_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS feature_tracker_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(feature_tracker
  "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feature_tracker
)
_generate_msg_eus(feature_tracker
  "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg"
  "${MSG_I_FLAGS}"
  "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feature_tracker
)

### Generating Services

### Generating Module File
_generate_module_eus(feature_tracker
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feature_tracker
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(feature_tracker_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(feature_tracker_generate_messages feature_tracker_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg" NAME_WE)
add_dependencies(feature_tracker_generate_messages_eus _feature_tracker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg" NAME_WE)
add_dependencies(feature_tracker_generate_messages_eus _feature_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(feature_tracker_geneus)
add_dependencies(feature_tracker_geneus feature_tracker_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS feature_tracker_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(feature_tracker
  "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feature_tracker
)
_generate_msg_lisp(feature_tracker
  "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg"
  "${MSG_I_FLAGS}"
  "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feature_tracker
)

### Generating Services

### Generating Module File
_generate_module_lisp(feature_tracker
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feature_tracker
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(feature_tracker_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(feature_tracker_generate_messages feature_tracker_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg" NAME_WE)
add_dependencies(feature_tracker_generate_messages_lisp _feature_tracker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg" NAME_WE)
add_dependencies(feature_tracker_generate_messages_lisp _feature_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(feature_tracker_genlisp)
add_dependencies(feature_tracker_genlisp feature_tracker_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS feature_tracker_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(feature_tracker
  "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feature_tracker
)
_generate_msg_nodejs(feature_tracker
  "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg"
  "${MSG_I_FLAGS}"
  "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feature_tracker
)

### Generating Services

### Generating Module File
_generate_module_nodejs(feature_tracker
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feature_tracker
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(feature_tracker_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(feature_tracker_generate_messages feature_tracker_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg" NAME_WE)
add_dependencies(feature_tracker_generate_messages_nodejs _feature_tracker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg" NAME_WE)
add_dependencies(feature_tracker_generate_messages_nodejs _feature_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(feature_tracker_gennodejs)
add_dependencies(feature_tracker_gennodejs feature_tracker_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS feature_tracker_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(feature_tracker
  "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feature_tracker
)
_generate_msg_py(feature_tracker
  "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg"
  "${MSG_I_FLAGS}"
  "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feature_tracker
)

### Generating Services

### Generating Module File
_generate_module_py(feature_tracker
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feature_tracker
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(feature_tracker_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(feature_tracker_generate_messages feature_tracker_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/FeatureTrackerResult.msg" NAME_WE)
add_dependencies(feature_tracker_generate_messages_py _feature_tracker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sfox/slam_mono/src/feature_tracker/msg/CameraTrackerResult.msg" NAME_WE)
add_dependencies(feature_tracker_generate_messages_py _feature_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(feature_tracker_genpy)
add_dependencies(feature_tracker_genpy feature_tracker_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS feature_tracker_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feature_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feature_tracker
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(feature_tracker_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feature_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feature_tracker
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(feature_tracker_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feature_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feature_tracker
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(feature_tracker_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feature_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feature_tracker
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(feature_tracker_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feature_tracker)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feature_tracker\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feature_tracker
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(feature_tracker_generate_messages_py std_msgs_generate_messages_py)
endif()
