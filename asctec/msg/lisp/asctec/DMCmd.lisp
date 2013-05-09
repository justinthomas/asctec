; Auto-generated. Do not edit!


(in-package asctec-msg)


;//! \htmlinclude DMCmd.msg.html

(defclass <DMCmd> (ros-message)
  ((u1
    :reader u1-val
    :initarg :u1
    :type float
    :initform 0.0)
   (u2
    :reader u2-val
    :initarg :u2
    :type float
    :initform 0.0)
   (u3
    :reader u3-val
    :initarg :u3
    :type float
    :initform 0.0)
   (u4
    :reader u4-val
    :initarg :u4
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <DMCmd>) ostream)
  "Serializes a message object of type '<DMCmd>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'u1))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'u2))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'u3))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'u4))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <DMCmd>) istream)
  "Deserializes a message object of type '<DMCmd>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'u1) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'u2) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'u3) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'u4) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<DMCmd>)))
  "Returns string type for a message object of type '<DMCmd>"
  "asctec/DMCmd")
(defmethod md5sum ((type (eql '<DMCmd>)))
  "Returns md5sum for a message object of type '<DMCmd>"
  "a8e3bd0295c76b1fec501bbdac183cec")
(defmethod message-definition ((type (eql '<DMCmd>)))
  "Returns full string definition for message of type '<DMCmd>"
  (format nil "float32 u1~%float32 u2~%float32 u3~%float32 u4~%~%~%"))
(defmethod serialization-length ((msg <DMCmd>))
  (+ 0
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <DMCmd>))
  "Converts a ROS message object to a list"
  (list '<DMCmd>
    (cons ':u1 (u1-val msg))
    (cons ':u2 (u2-val msg))
    (cons ':u3 (u3-val msg))
    (cons ':u4 (u4-val msg))
))
