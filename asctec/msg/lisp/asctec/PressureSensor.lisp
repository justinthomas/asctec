; Auto-generated. Do not edit!


(in-package asctec-msg)


;//! \htmlinclude PressureSensor.msg.html

(defclass <PressureSensor> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (height
    :reader height-val
    :initarg :height
    :type float
    :initform 0.0)
   (dheight
    :reader dheight-val
    :initarg :dheight
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <PressureSensor>) ostream)
  "Serializes a message object of type '<PressureSensor>"
  (serialize (slot-value msg 'header) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'height))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'dheight))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <PressureSensor>) istream)
  "Deserializes a message object of type '<PressureSensor>"
  (deserialize (slot-value msg 'header) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'dheight) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<PressureSensor>)))
  "Returns string type for a message object of type '<PressureSensor>"
  "asctec/PressureSensor")
(defmethod md5sum ((type (eql '<PressureSensor>)))
  "Returns md5sum for a message object of type '<PressureSensor>"
  "260fa61dcf85be8923dc1c907e47ffda")
(defmethod message-definition ((type (eql '<PressureSensor>)))
  "Returns full string definition for message of type '<PressureSensor>"
  (format nil "Header header~%float32 height~%float32 dheight~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <PressureSensor>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
))
(defmethod ros-message-to-list ((msg <PressureSensor>))
  "Converts a ROS message object to a list"
  (list '<PressureSensor>
    (cons ':header (header-val msg))
    (cons ':height (height-val msg))
    (cons ':dheight (dheight-val msg))
))
