; Auto-generated. Do not edit!


(in-package gradient_planner-msg)


;//! \htmlinclude GPCostmapM.msg.html

(defclass <GPCostmapM> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (name
    :reader name-val
    :initarg :name
    :type string
    :initform "")
   (rows
    :reader rows-val
    :initarg :rows
    :type integer
    :initform 0)
   (cols
    :reader cols-val
    :initarg :cols
    :type integer
    :initform 0)
   (data
    :reader data-val
    :initarg :data
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0)))
)
(defmethod serialize ((msg <GPCostmapM>) ostream)
  "Serializes a message object of type '<GPCostmapM>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'name))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'name))
    (write-byte (ldb (byte 8 0) (slot-value msg 'rows)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'rows)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'rows)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'rows)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'cols)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'cols)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'cols)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'cols)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'data))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'data))
)
(defmethod deserialize ((msg <GPCostmapM>) istream)
  "Deserializes a message object of type '<GPCostmapM>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'name) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'name) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'rows)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'rows)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'rows)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'rows)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'cols)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'cols)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'cols)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'cols)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'data) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'data)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(defmethod ros-datatype ((msg (eql '<GPCostmapM>)))
  "Returns string type for a message object of type '<GPCostmapM>"
  "gradient_planner/GPCostmapM")
(defmethod md5sum ((type (eql '<GPCostmapM>)))
  "Returns md5sum for a message object of type '<GPCostmapM>"
  "5fb9da7eea71c83e315b2e5130c03354")
(defmethod message-definition ((type (eql '<GPCostmapM>)))
  "Returns full string definition for message of type '<GPCostmapM>"
  (format nil "Header header~%string name~%int32 rows~%int32 cols~%float32[] data~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <GPCostmapM>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'name))
     4
     4
     4 (reduce #'+ (slot-value msg 'data) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <GPCostmapM>))
  "Converts a ROS message object to a list"
  (list '<GPCostmapM>
    (cons ':header (header-val msg))
    (cons ':name (name-val msg))
    (cons ':rows (rows-val msg))
    (cons ':cols (cols-val msg))
    (cons ':data (data-val msg))
))
