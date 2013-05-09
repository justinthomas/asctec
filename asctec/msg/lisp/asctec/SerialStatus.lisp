; Auto-generated. Do not edit!


(in-package asctec-msg)


;//! \htmlinclude SerialStatus.msg.html

(defclass <SerialStatus> (ros-message)
  ((serial
    :reader serial-val
    :initarg :serial
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <SerialStatus>) ostream)
  "Serializes a message object of type '<SerialStatus>"
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'serial) 1 0)) ostream)
)
(defmethod deserialize ((msg <SerialStatus>) istream)
  "Deserializes a message object of type '<SerialStatus>"
  (setf (slot-value msg 'serial) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<SerialStatus>)))
  "Returns string type for a message object of type '<SerialStatus>"
  "asctec/SerialStatus")
(defmethod md5sum ((type (eql '<SerialStatus>)))
  "Returns md5sum for a message object of type '<SerialStatus>"
  "ddbb2a6dc4d82106e2df21241955c01f")
(defmethod message-definition ((type (eql '<SerialStatus>)))
  "Returns full string definition for message of type '<SerialStatus>"
  (format nil "bool serial~%~%~%"))
(defmethod serialization-length ((msg <SerialStatus>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <SerialStatus>))
  "Converts a ROS message object to a list"
  (list '<SerialStatus>
    (cons ':serial (serial-val msg))
))
