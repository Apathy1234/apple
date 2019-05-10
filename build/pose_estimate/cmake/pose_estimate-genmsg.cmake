# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pose_estimate: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ipose_estimate:/home/sfox/slam_mono/src/pose_estimate/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pose_estimate_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg" NAME_WE)
add_custom_target(_pose_estimate_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pose_estimate" "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pose_estimate
  "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pose_estimate
)

### Generating Services

### Generating Module File
_generate_module_cpp(pose_estimate
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pose_estimate
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pose_estimate_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pose_estimate_generate_messages pose_estimate_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg" NAME_WE)
add_dependencies(pose_estimate_generate_messages_cpp _pose_estimate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pose_estimate_gencpp)
add_dependencies(pose_estimate_gencpp pose_estimate_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pose_estimate_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pose_estimate
  "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pose_estimate
)

### Generating Services

### Generating Module File
_generate_module_eus(pose_estimate
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pose_estimate
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pose_estimate_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pose_estimate_generate_messages pose_estimate_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg" NAME_WE)
add_dependencies(pose_estimate_generate_messages_eus _pose_estimate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pose_estimate_geneus)
add_dependencies(pose_estimate_geneus pose_estimate_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pose_estimate_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pose_estimate
  "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pose_estimate
)

### Generating Services

### Generating Module File
_generate_module_lisp(pose_estimate
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pose_estimate
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pose_estimate_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pose_estimate_generate_messages pose_estimate_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg" NAME_WE)
add_dependencies(pose_estimate_generate_messages_lisp _pose_estimate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pose_estimate_genlisp)
add_dependencies(pose_estimate_genlisp pose_estimate_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pose_estimate_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pose_estimate
  "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pose_estimate
)

### Generating Services

### Generating Module File
_generate_module_nodejs(pose_estimate
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pose_estimate
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pose_estimate_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pose_estimate_generate_messages pose_estimate_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg" NAME_WE)
add_dependencies(pose_estimate_generate_messages_nodejs _pose_estimate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pose_estimate_gennodejs)
add_dependencies(pose_estimate_gennodejs pose_estimate_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pose_estimate_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pose_estimate
  "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimate
)

### Generating Services

### Generating Module File
_generate_module_py(pose_estimate
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimate
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pose_estimate_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pose_estimate_generate_messages pose_estimate_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sfox/slam_mono/src/pose_estimate/msg/PoseEstimateResult.msg" NAME_WE)
add_dependencies(pose_estimate_generate_messages_py _pose_estimate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pose_estimate_genpy)
add_dependencies(pose_estimate_genpy pose_estimate_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pose_estimate_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pose_estimate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pose_estimate
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pose_estimate_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pose_estimate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pose_estimate
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(pose_estimate_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pose_estimate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pose_estimate
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(pose_estimate_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pose_estimate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pose_estimate
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(pose_estimate_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimate)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimate\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimate
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pose_estimate_generate_messages_py std_msgs_generate_messages_py)
endif()
