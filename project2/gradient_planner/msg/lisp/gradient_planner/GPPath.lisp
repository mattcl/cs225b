; Auto-generated. Do not edit!


(in-package gradient_planner-msg)


;//! \htmlinclude GPPath.msg.html

(defclass <GPPath> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (length
    :reader length-val
    :initarg :length
    :type integer
    :initform 0)
   (x
    :reader x-val
    :initarg :x
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (y
    :reader y-val
    :initarg :y
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0)))
)
(defmethod serialize ((msg <GPPath>) ostream)
  "Serializes a message object of type '<GPPath>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'length)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'length)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'length)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'length)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'x))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'x))
  (let ((__ros_arr_len (length (slot-value msg 'y))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'y))
)
(defmethod deserialize ((msg <GPPath>) istream)
  "Deserializes a message object of type '<GPPath>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'length)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'length)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'length)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'length)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'x) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'x)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'y) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'y)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(defmethod ros-datatype ((msg (eql '<GPPath>)))
  "Returns string type for a message object of type '<GPPath>"
  "gradient_planner/GPPath")
(defmethod md5sum ((type (eql '<GPPath>)))
  "Returns md5sum for a message object of type '<GPPath>"
  "0c497d9c4f459ad6e83d5642da79a4c1")
(defmethod message-definition ((type (eql '<GPPath>)))
  "Returns full string definition for message of type '<GPPath>"
  (format nil "Header header~%int32 length~%float32[] x~%float32[] y~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <GPPath>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4 (reduce #'+ (slot-value msg 'x) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'y) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <GPPath>))
  "Converts a ROS message object to a list"
  (list '<GPPath>
    (cons ':header (header-val msg))
    (cons ':length (length-val msg))
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
))
