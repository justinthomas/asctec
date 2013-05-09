; Auto-generated. Do not edit!


(in-package asctec-msg)


;//! \htmlinclude Voltage.msg.html

(defclass <Voltage> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (battery_voltage
    :reader battery_voltage-val
    :initarg :battery_voltage
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <Voltage>) ostream)
  "Serializes a message object of type '<Voltage>"
  (serialize (slot-value msg 'header) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'battery_voltage))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <Voltage>) istream)
  "Deserializes a message object of type '<Voltage>"
  (deserialize (slot-value msg 'header) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'battery_voltage) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Voltage>)))
  "Returns string type for a message object of type '<Voltage>"
  "asctec/Voltage")
(defmethod md5sum ((type (eql '<Voltage>)))
  "Returns md5sum for a message object of type '<Voltage>"
  "3a5308053d8356e587cb07d69c689fff")
(defmethod message-definition ((type (eql '<Voltage>)))
  "Returns full string definition for message of type '<Voltage>"
  (format nil "Header header~%float32 battery_voltage~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Voltage>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
))
(defmethod ros-message-to-list ((msg <Voltage>))
  "Converts a ROS message object to a list"
  (list '<Voltage>
    (cons ':header (header-val msg))
    (cons ':battery_voltage (battery_voltage-val msg))
))
