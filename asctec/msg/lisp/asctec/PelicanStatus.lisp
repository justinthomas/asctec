; Auto-generated. Do not edit!


(in-package asctec-msg)


;//! \htmlinclude PelicanStatus.msg.html

(defclass <PelicanStatus> (ros-message)
  ((battery_voltage
    :reader battery_voltage-val
    :initarg :battery_voltage
    :type float
    :initform 0.0)
   (cpu_load
    :reader cpu_load-val
    :initarg :cpu_load
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <PelicanStatus>) ostream)
  "Serializes a message object of type '<PelicanStatus>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'battery_voltage))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'cpu_load)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'cpu_load)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'cpu_load)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'cpu_load)) ostream)
)
(defmethod deserialize ((msg <PelicanStatus>) istream)
  "Deserializes a message object of type '<PelicanStatus>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'battery_voltage) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'cpu_load)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'cpu_load)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'cpu_load)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'cpu_load)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<PelicanStatus>)))
  "Returns string type for a message object of type '<PelicanStatus>"
  "asctec/PelicanStatus")
(defmethod md5sum ((type (eql '<PelicanStatus>)))
  "Returns md5sum for a message object of type '<PelicanStatus>"
  "d3910c357b76a00c1f7571595c0fda7c")
(defmethod message-definition ((type (eql '<PelicanStatus>)))
  "Returns full string definition for message of type '<PelicanStatus>"
  (format nil "float32 battery_voltage~%uint32 cpu_load~%~%~%"))
(defmethod serialization-length ((msg <PelicanStatus>))
  (+ 0
     4
     4
))
(defmethod ros-message-to-list ((msg <PelicanStatus>))
  "Converts a ROS message object to a list"
  (list '<PelicanStatus>
    (cons ':battery_voltage (battery_voltage-val msg))
    (cons ':cpu_load (cpu_load-val msg))
))
