; Auto-generated. Do not edit!


(cl:in-package coman_msgs-msg)


;//! \htmlinclude ForceTorque.msg.html

(cl:defclass <ForceTorque> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (xForceNewtons
    :reader xForceNewtons
    :initarg :xForceNewtons
    :type cl:float
    :initform 0.0)
   (yForceNewtons
    :reader yForceNewtons
    :initarg :yForceNewtons
    :type cl:float
    :initform 0.0)
   (zForceNewtons
    :reader zForceNewtons
    :initarg :zForceNewtons
    :type cl:float
    :initform 0.0)
   (xTorqueNewtonMeters
    :reader xTorqueNewtonMeters
    :initarg :xTorqueNewtonMeters
    :type cl:float
    :initform 0.0)
   (yTorqueNewtonMeters
    :reader yTorqueNewtonMeters
    :initarg :yTorqueNewtonMeters
    :type cl:float
    :initform 0.0)
   (zTorqueNewtonMeters
    :reader zTorqueNewtonMeters
    :initarg :zTorqueNewtonMeters
    :type cl:float
    :initform 0.0))
)

(cl:defclass ForceTorque (<ForceTorque>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ForceTorque>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ForceTorque)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coman_msgs-msg:<ForceTorque> is deprecated: use coman_msgs-msg:ForceTorque instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ForceTorque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coman_msgs-msg:header-val is deprecated.  Use coman_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'xForceNewtons-val :lambda-list '(m))
(cl:defmethod xForceNewtons-val ((m <ForceTorque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coman_msgs-msg:xForceNewtons-val is deprecated.  Use coman_msgs-msg:xForceNewtons instead.")
  (xForceNewtons m))

(cl:ensure-generic-function 'yForceNewtons-val :lambda-list '(m))
(cl:defmethod yForceNewtons-val ((m <ForceTorque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coman_msgs-msg:yForceNewtons-val is deprecated.  Use coman_msgs-msg:yForceNewtons instead.")
  (yForceNewtons m))

(cl:ensure-generic-function 'zForceNewtons-val :lambda-list '(m))
(cl:defmethod zForceNewtons-val ((m <ForceTorque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coman_msgs-msg:zForceNewtons-val is deprecated.  Use coman_msgs-msg:zForceNewtons instead.")
  (zForceNewtons m))

(cl:ensure-generic-function 'xTorqueNewtonMeters-val :lambda-list '(m))
(cl:defmethod xTorqueNewtonMeters-val ((m <ForceTorque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coman_msgs-msg:xTorqueNewtonMeters-val is deprecated.  Use coman_msgs-msg:xTorqueNewtonMeters instead.")
  (xTorqueNewtonMeters m))

(cl:ensure-generic-function 'yTorqueNewtonMeters-val :lambda-list '(m))
(cl:defmethod yTorqueNewtonMeters-val ((m <ForceTorque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coman_msgs-msg:yTorqueNewtonMeters-val is deprecated.  Use coman_msgs-msg:yTorqueNewtonMeters instead.")
  (yTorqueNewtonMeters m))

(cl:ensure-generic-function 'zTorqueNewtonMeters-val :lambda-list '(m))
(cl:defmethod zTorqueNewtonMeters-val ((m <ForceTorque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coman_msgs-msg:zTorqueNewtonMeters-val is deprecated.  Use coman_msgs-msg:zTorqueNewtonMeters instead.")
  (zTorqueNewtonMeters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ForceTorque>) ostream)
  "Serializes a message object of type '<ForceTorque>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'xForceNewtons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yForceNewtons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'zForceNewtons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'xTorqueNewtonMeters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yTorqueNewtonMeters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'zTorqueNewtonMeters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ForceTorque>) istream)
  "Deserializes a message object of type '<ForceTorque>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xForceNewtons) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yForceNewtons) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zForceNewtons) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xTorqueNewtonMeters) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yTorqueNewtonMeters) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zTorqueNewtonMeters) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ForceTorque>)))
  "Returns string type for a message object of type '<ForceTorque>"
  "coman_msgs/ForceTorque")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ForceTorque)))
  "Returns string type for a message object of type 'ForceTorque"
  "coman_msgs/ForceTorque")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ForceTorque>)))
  "Returns md5sum for a message object of type '<ForceTorque>"
  "b34fdd2e02f35ff10dc9a8e2137fa7ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ForceTorque)))
  "Returns md5sum for a message object of type 'ForceTorque"
  "b34fdd2e02f35ff10dc9a8e2137fa7ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ForceTorque>)))
  "Returns full string definition for message of type '<ForceTorque>"
  (cl:format cl:nil "Header header~%~%float64 xForceNewtons~%float64 yForceNewtons~%float64 zForceNewtons~%float64 xTorqueNewtonMeters~%float64 yTorqueNewtonMeters~%float64 zTorqueNewtonMeters~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ForceTorque)))
  "Returns full string definition for message of type 'ForceTorque"
  (cl:format cl:nil "Header header~%~%float64 xForceNewtons~%float64 yForceNewtons~%float64 zForceNewtons~%float64 xTorqueNewtonMeters~%float64 yTorqueNewtonMeters~%float64 zTorqueNewtonMeters~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ForceTorque>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ForceTorque>))
  "Converts a ROS message object to a list"
  (cl:list 'ForceTorque
    (cl:cons ':header (header msg))
    (cl:cons ':xForceNewtons (xForceNewtons msg))
    (cl:cons ':yForceNewtons (yForceNewtons msg))
    (cl:cons ':zForceNewtons (zForceNewtons msg))
    (cl:cons ':xTorqueNewtonMeters (xTorqueNewtonMeters msg))
    (cl:cons ':yTorqueNewtonMeters (yTorqueNewtonMeters msg))
    (cl:cons ':zTorqueNewtonMeters (zTorqueNewtonMeters msg))
))
