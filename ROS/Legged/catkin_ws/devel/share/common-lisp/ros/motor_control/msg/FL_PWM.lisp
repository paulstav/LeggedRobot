; Auto-generated. Do not edit!


(cl:in-package motor_control-msg)


;//! \htmlinclude FL_PWM.msg.html

(cl:defclass <FL_PWM> (roslisp-msg-protocol:ros-message)
  ((pwm_duty
    :reader pwm_duty
    :initarg :pwm_duty
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FL_PWM (<FL_PWM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FL_PWM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FL_PWM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_control-msg:<FL_PWM> is deprecated: use motor_control-msg:FL_PWM instead.")))

(cl:ensure-generic-function 'pwm_duty-val :lambda-list '(m))
(cl:defmethod pwm_duty-val ((m <FL_PWM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control-msg:pwm_duty-val is deprecated.  Use motor_control-msg:pwm_duty instead.")
  (pwm_duty m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FL_PWM>) ostream)
  "Serializes a message object of type '<FL_PWM>"
  (cl:let* ((signed (cl:slot-value msg 'pwm_duty)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FL_PWM>) istream)
  "Deserializes a message object of type '<FL_PWM>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwm_duty) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FL_PWM>)))
  "Returns string type for a message object of type '<FL_PWM>"
  "motor_control/FL_PWM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FL_PWM)))
  "Returns string type for a message object of type 'FL_PWM"
  "motor_control/FL_PWM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FL_PWM>)))
  "Returns md5sum for a message object of type '<FL_PWM>"
  "cc1c76e848affc91996664a93666ea97")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FL_PWM)))
  "Returns md5sum for a message object of type 'FL_PWM"
  "cc1c76e848affc91996664a93666ea97")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FL_PWM>)))
  "Returns full string definition for message of type '<FL_PWM>"
  (cl:format cl:nil "int8 pwm_duty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FL_PWM)))
  "Returns full string definition for message of type 'FL_PWM"
  (cl:format cl:nil "int8 pwm_duty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FL_PWM>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FL_PWM>))
  "Converts a ROS message object to a list"
  (cl:list 'FL_PWM
    (cl:cons ':pwm_duty (pwm_duty msg))
))
