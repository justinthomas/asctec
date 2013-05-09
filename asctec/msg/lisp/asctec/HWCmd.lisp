; Auto-generated. Do not edit!


(in-package asctec-msg)


;//! \htmlinclude HWCmd.msg.html

(defclass <HWCmd> (ros-message)
  ((thrust
    :reader thrust-val
    :initarg :thrust
    :type float
    :initform 0.0)
   (roll
    :reader roll-val
    :initarg :roll
    :type float
    :initform 0.0)
   (pitch
    :reader pitch-val
    :initarg :pitch
    :type float
    :initform 0.0)
   (yaw
    :reader yaw-val
    :initarg :yaw
    :type float
    :initform 0.0)
   (cmd
    :reader cmd-val
    :initarg :cmd
    :type (vector boolean)
   :initform (make-array 4 :element-type 'boolean :initial-element nil)))
)
(defmethod serialize ((msg <HWCmd>) ostream)
  "Serializes a message object of type '<HWCmd>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'thrust))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'roll))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'pitch))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yaw))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) (if ele 1 0)) ostream))(slot-value msg 'cmd))
)
(defmethod deserialize ((msg <HWCmd>) istream)
  "Deserializes a message object of type '<HWCmd>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'thrust) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  (setf (slot-value msg 'cmd) (make-array 4))
  (let ((vals (slot-value msg 'cmd)))
    (dotimes (i 4)
(setf (aref vals i) (not (zerop (read-byte istream))))))
  msg
)
(defmethod ros-datatype ((msg (eql '<HWCmd>)))
  "Returns string type for a message object of type '<HWCmd>"
  "asctec/HWCmd")
(defmethod md5sum ((type (eql '<HWCmd>)))
  "Returns md5sum for a message object of type '<HWCmd>"
  "f487b81408b480bd5b23cf4ed04d109d")
(defmethod message-definition ((type (eql '<HWCmd>)))
  "Returns full string definition for message of type '<HWCmd>"
  (format nil "float32 thrust~%float32 roll~%float32 pitch~%float32 yaw~%bool[4] cmd~%~%~%"))
(defmethod serialization-length ((msg <HWCmd>))
  (+ 0
     4
     4
     4
     4
     0 (reduce #'+ (slot-value msg 'cmd) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
))
(defmethod ros-message-to-list ((msg <HWCmd>))
  "Converts a ROS message object to a list"
  (list '<HWCmd>
    (cons ':thrust (thrust-val msg))
    (cons ':roll (roll-val msg))
    (cons ':pitch (pitch-val msg))
    (cons ':yaw (yaw-val msg))
    (cons ':cmd (cmd-val msg))
))
