# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "nao_tutoring_behaviors: 2 messages, 0 services")

set(MSG_I_FLAGS "-Inao_tutoring_behaviors:/root/catkin_ws/src/nao_tutoring_behaviors/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(nao_tutoring_behaviors_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/catkin_ws/src/nao_tutoring_behaviors/msg/TabletMsg.msg" NAME_WE)
add_custom_target(_nao_tutoring_behaviors_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nao_tutoring_behaviors" "/root/catkin_ws/src/nao_tutoring_behaviors/msg/TabletMsg.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/src/nao_tutoring_behaviors/msg/ControlMsg.msg" NAME_WE)
add_custom_target(_nao_tutoring_behaviors_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nao_tutoring_behaviors" "/root/catkin_ws/src/nao_tutoring_behaviors/msg/ControlMsg.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(nao_tutoring_behaviors
  "/root/catkin_ws/src/nao_tutoring_behaviors/msg/TabletMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nao_tutoring_behaviors
)
_generate_msg_cpp(nao_tutoring_behaviors
  "/root/catkin_ws/src/nao_tutoring_behaviors/msg/ControlMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nao_tutoring_behaviors
)

### Generating Services

### Generating Module File
_generate_module_cpp(nao_tutoring_behaviors
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nao_tutoring_behaviors
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(nao_tutoring_behaviors_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(nao_tutoring_behaviors_generate_messages nao_tutoring_behaviors_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/nao_tutoring_behaviors/msg/TabletMsg.msg" NAME_WE)
add_dependencies(nao_tutoring_behaviors_generate_messages_cpp _nao_tutoring_behaviors_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/nao_tutoring_behaviors/msg/ControlMsg.msg" NAME_WE)
add_dependencies(nao_tutoring_behaviors_generate_messages_cpp _nao_tutoring_behaviors_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nao_tutoring_behaviors_gencpp)
add_dependencies(nao_tutoring_behaviors_gencpp nao_tutoring_behaviors_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nao_tutoring_behaviors_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(nao_tutoring_behaviors
  "/root/catkin_ws/src/nao_tutoring_behaviors/msg/TabletMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nao_tutoring_behaviors
)
_generate_msg_lisp(nao_tutoring_behaviors
  "/root/catkin_ws/src/nao_tutoring_behaviors/msg/ControlMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nao_tutoring_behaviors
)

### Generating Services

### Generating Module File
_generate_module_lisp(nao_tutoring_behaviors
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nao_tutoring_behaviors
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(nao_tutoring_behaviors_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(nao_tutoring_behaviors_generate_messages nao_tutoring_behaviors_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/nao_tutoring_behaviors/msg/TabletMsg.msg" NAME_WE)
add_dependencies(nao_tutoring_behaviors_generate_messages_lisp _nao_tutoring_behaviors_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/nao_tutoring_behaviors/msg/ControlMsg.msg" NAME_WE)
add_dependencies(nao_tutoring_behaviors_generate_messages_lisp _nao_tutoring_behaviors_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nao_tutoring_behaviors_genlisp)
add_dependencies(nao_tutoring_behaviors_genlisp nao_tutoring_behaviors_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nao_tutoring_behaviors_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(nao_tutoring_behaviors
  "/root/catkin_ws/src/nao_tutoring_behaviors/msg/TabletMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nao_tutoring_behaviors
)
_generate_msg_py(nao_tutoring_behaviors
  "/root/catkin_ws/src/nao_tutoring_behaviors/msg/ControlMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nao_tutoring_behaviors
)

### Generating Services

### Generating Module File
_generate_module_py(nao_tutoring_behaviors
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nao_tutoring_behaviors
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(nao_tutoring_behaviors_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(nao_tutoring_behaviors_generate_messages nao_tutoring_behaviors_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/nao_tutoring_behaviors/msg/TabletMsg.msg" NAME_WE)
add_dependencies(nao_tutoring_behaviors_generate_messages_py _nao_tutoring_behaviors_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/nao_tutoring_behaviors/msg/ControlMsg.msg" NAME_WE)
add_dependencies(nao_tutoring_behaviors_generate_messages_py _nao_tutoring_behaviors_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nao_tutoring_behaviors_genpy)
add_dependencies(nao_tutoring_behaviors_genpy nao_tutoring_behaviors_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nao_tutoring_behaviors_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nao_tutoring_behaviors)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nao_tutoring_behaviors
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(nao_tutoring_behaviors_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nao_tutoring_behaviors)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nao_tutoring_behaviors
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(nao_tutoring_behaviors_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nao_tutoring_behaviors)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nao_tutoring_behaviors\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nao_tutoring_behaviors
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(nao_tutoring_behaviors_generate_messages_py std_msgs_generate_messages_py)
endif()
