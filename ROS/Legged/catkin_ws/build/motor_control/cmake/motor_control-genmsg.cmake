# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "motor_control: 9 messages, 0 services")

set(MSG_I_FLAGS "-Imotor_control:/home/legged/Legged/catkin_ws/src/motor_control/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(motor_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_Encoder.msg" NAME_WE)
add_custom_target(_motor_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_control" "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_Encoder.msg" ""
)

get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_PWM.msg" NAME_WE)
add_custom_target(_motor_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_control" "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_PWM.msg" ""
)

get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_Encoder.msg" NAME_WE)
add_custom_target(_motor_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_control" "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_Encoder.msg" ""
)

get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_PWM.msg" NAME_WE)
add_custom_target(_motor_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_control" "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_PWM.msg" ""
)

get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_Encoder.msg" NAME_WE)
add_custom_target(_motor_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_control" "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_Encoder.msg" ""
)

get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_PWM.msg" NAME_WE)
add_custom_target(_motor_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_control" "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_PWM.msg" ""
)

get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/Position.msg" NAME_WE)
add_custom_target(_motor_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_control" "/home/legged/Legged/catkin_ws/src/motor_control/msg/Position.msg" ""
)

get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_Encoder.msg" NAME_WE)
add_custom_target(_motor_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_control" "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_Encoder.msg" ""
)

get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_PWM.msg" NAME_WE)
add_custom_target(_motor_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_control" "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_PWM.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
)
_generate_msg_cpp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
)
_generate_msg_cpp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
)
_generate_msg_cpp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
)
_generate_msg_cpp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
)
_generate_msg_cpp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
)
_generate_msg_cpp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
)
_generate_msg_cpp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
)
_generate_msg_cpp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(motor_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(motor_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(motor_control_generate_messages motor_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_cpp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_cpp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_cpp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_cpp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_cpp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_cpp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/Position.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_cpp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_cpp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_cpp _motor_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_control_gencpp)
add_dependencies(motor_control_gencpp motor_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_control_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
)
_generate_msg_lisp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
)
_generate_msg_lisp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
)
_generate_msg_lisp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
)
_generate_msg_lisp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
)
_generate_msg_lisp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
)
_generate_msg_lisp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
)
_generate_msg_lisp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
)
_generate_msg_lisp(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(motor_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(motor_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(motor_control_generate_messages motor_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_lisp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_lisp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_lisp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_lisp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_lisp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_lisp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/Position.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_lisp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_lisp _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_lisp _motor_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_control_genlisp)
add_dependencies(motor_control_genlisp motor_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_control_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
)
_generate_msg_py(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
)
_generate_msg_py(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
)
_generate_msg_py(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
)
_generate_msg_py(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
)
_generate_msg_py(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
)
_generate_msg_py(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_PWM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
)
_generate_msg_py(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
)
_generate_msg_py(motor_control
  "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_Encoder.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
)

### Generating Services

### Generating Module File
_generate_module_py(motor_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(motor_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(motor_control_generate_messages motor_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_py _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_py _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_py _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BL_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_py _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_py _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FR_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_py _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/Position.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_py _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/BR_Encoder.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_py _motor_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/legged/Legged/catkin_ws/src/motor_control/msg/FL_PWM.msg" NAME_WE)
add_dependencies(motor_control_generate_messages_py _motor_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_control_genpy)
add_dependencies(motor_control_genpy motor_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(motor_control_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(motor_control_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(motor_control_generate_messages_py std_msgs_generate_messages_py)
