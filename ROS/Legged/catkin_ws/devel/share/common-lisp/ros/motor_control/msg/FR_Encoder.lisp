; Auto-generated. Do not edit!


(cl:in-package motor_control-msg)


;//! \htmlinclude FR_Encoder.msg.html

(cl:defclass <FR_Encoder> (roslisp-msg-protocol:ros-message)
  ((encoder
    :reader encoder
    :initarg :encoder
    :type cl:integer
    :initform 0))
)

(cl:defclass FR_Encoder (<FR_Encoder>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FR_Encoder>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FR_Encoder)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_control-msg:<FR_Encoder> is deprecated: use motor_control-msg:FR_Encoder instead.")))

(cl:ensure-generic-function 'encoder-val :lambda-list '(m))
(cl:defmethod encoder-val ((m <FR_Encoder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control-msg:encoder-val is deprecated.  Use motor_control-msg:encoder instead.")
  (encoder m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FR_Encoder>) ostream)
  "Serializes a message object of type '<FR_Encoder>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'encoder)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'encoder)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'encoder)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'encoder)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FR_Encoder>) istream)
  "Deserializes a message object of type '<FR_Encoder>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'encoder)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'encoder)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'encoder)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'encoder)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FR_Encoder>)))
  "Returns string type for a message object of type '<FR_Encoder>"
  "motor_control/FR_Encoder")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FR_Encoder)))
  "Returns string type for a message object of type 'FR_Encoder"
  "motor_control/FR_Encoder")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FR_Encoder>)))
  "Returns md5sum for a message object of type '<FR_Encoder>"
  "281bf2da8de91e03210bb87e7782dcbd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FR_Encoder)))
  "Returns md5sum for a message object of type 'FR_Encoder"
  "281bf2da8de91e03210bb87e7782dcbd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FR_Encoder>)))
  "Returns full string definition for message of type '<FR_Encoder>"
  (cl:format cl:nil "uint32 encoder~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FR_Encoder)))
  "Returns full string definition for message of type 'FR_Encoder"
  (cl:format cl:nil "uint32 encoder~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FR_Encoder>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FR_Encoder>))
  "Converts a ROS message object to a list"
  (cl:list 'FR_Encoder
    (cl:cons ':encoder (encoder msg))
))
