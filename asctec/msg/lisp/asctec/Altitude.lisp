; Auto-generated. Do not edit!


(in-package asctec-msg)


;//! \htmlinclude Altitude.msg.html

(defclass <Altitude> (ros-message)
  ((height
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
(defmethod serialize ((msg <Altitude>) ostream)
  "Serializes a message object of type '<Altitude>"
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
(defmethod deserialize ((msg <Altitude>) istream)
  "Deserializes a message object of type '<Altitude>"
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
(defmethod ros-datatype ((msg (eql '<Altitude>)))
  "Returns string type for a message object of type '<Altitude>"
  "asctec/Altitude")
(defmethod md5sum ((type (eql '<Altitude>)))
  "Returns md5sum for a message object of type '<Altitude>"
  "4c2ce8aa26ec0677804da2400083fc22")
(defmethod message-definition ((type (eql '<Altitude>)))
  "Returns full string definition for message of type '<Altitude>"
  (format nil "float32 height~%float32 dheight~%~%"))
(defmethod serialization-length ((msg <Altitude>))
  (+ 0
     4
     4
))
(defmethod ros-message-to-list ((msg <Altitude>))
  "Converts a ROS message object to a list"
  (list '<Altitude>
    (cons ':height (height-val msg))
    (cons ':dheight (dheight-val msg))
))
