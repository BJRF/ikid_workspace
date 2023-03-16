; Auto-generated. Do not edit!


(cl:in-package calculate_position_pkg-msg)


;//! \htmlinclude image_points.msg.html

(cl:defclass <image_points> (roslisp-msg-protocol:ros-message)
  ((x1
    :reader x1
    :initarg :x1
    :type cl:integer
    :initform 0)
   (x2
    :reader x2
    :initarg :x2
    :type cl:integer
    :initform 0)
   (x3
    :reader x3
    :initarg :x3
    :type cl:integer
    :initform 0)
   (x4
    :reader x4
    :initarg :x4
    :type cl:integer
    :initform 0)
   (y1
    :reader y1
    :initarg :y1
    :type cl:integer
    :initform 0)
   (y2
    :reader y2
    :initarg :y2
    :type cl:integer
    :initform 0)
   (y3
    :reader y3
    :initarg :y3
    :type cl:integer
    :initform 0)
   (y4
    :reader y4
    :initarg :y4
    :type cl:integer
    :initform 0))
)

(cl:defclass image_points (<image_points>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <image_points>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'image_points)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name calculate_position_pkg-msg:<image_points> is deprecated: use calculate_position_pkg-msg:image_points instead.")))

(cl:ensure-generic-function 'x1-val :lambda-list '(m))
(cl:defmethod x1-val ((m <image_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calculate_position_pkg-msg:x1-val is deprecated.  Use calculate_position_pkg-msg:x1 instead.")
  (x1 m))

(cl:ensure-generic-function 'x2-val :lambda-list '(m))
(cl:defmethod x2-val ((m <image_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calculate_position_pkg-msg:x2-val is deprecated.  Use calculate_position_pkg-msg:x2 instead.")
  (x2 m))

(cl:ensure-generic-function 'x3-val :lambda-list '(m))
(cl:defmethod x3-val ((m <image_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calculate_position_pkg-msg:x3-val is deprecated.  Use calculate_position_pkg-msg:x3 instead.")
  (x3 m))

(cl:ensure-generic-function 'x4-val :lambda-list '(m))
(cl:defmethod x4-val ((m <image_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calculate_position_pkg-msg:x4-val is deprecated.  Use calculate_position_pkg-msg:x4 instead.")
  (x4 m))

(cl:ensure-generic-function 'y1-val :lambda-list '(m))
(cl:defmethod y1-val ((m <image_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calculate_position_pkg-msg:y1-val is deprecated.  Use calculate_position_pkg-msg:y1 instead.")
  (y1 m))

(cl:ensure-generic-function 'y2-val :lambda-list '(m))
(cl:defmethod y2-val ((m <image_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calculate_position_pkg-msg:y2-val is deprecated.  Use calculate_position_pkg-msg:y2 instead.")
  (y2 m))

(cl:ensure-generic-function 'y3-val :lambda-list '(m))
(cl:defmethod y3-val ((m <image_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calculate_position_pkg-msg:y3-val is deprecated.  Use calculate_position_pkg-msg:y3 instead.")
  (y3 m))

(cl:ensure-generic-function 'y4-val :lambda-list '(m))
(cl:defmethod y4-val ((m <image_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calculate_position_pkg-msg:y4-val is deprecated.  Use calculate_position_pkg-msg:y4 instead.")
  (y4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <image_points>) ostream)
  "Serializes a message object of type '<image_points>"
  (cl:let* ((signed (cl:slot-value msg 'x1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <image_points>) istream)
  "Deserializes a message object of type '<image_points>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x3) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x4) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y3) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y4) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<image_points>)))
  "Returns string type for a message object of type '<image_points>"
  "calculate_position_pkg/image_points")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'image_points)))
  "Returns string type for a message object of type 'image_points"
  "calculate_position_pkg/image_points")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<image_points>)))
  "Returns md5sum for a message object of type '<image_points>"
  "d50612e5b17d5b27e8f9e37d8fc9f6f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'image_points)))
  "Returns md5sum for a message object of type 'image_points"
  "d50612e5b17d5b27e8f9e37d8fc9f6f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<image_points>)))
  "Returns full string definition for message of type '<image_points>"
  (cl:format cl:nil "int32 x1~%int32 x2~%int32 x3~%int32 x4~%int32 y1~%int32 y2~%int32 y3~%int32 y4~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'image_points)))
  "Returns full string definition for message of type 'image_points"
  (cl:format cl:nil "int32 x1~%int32 x2~%int32 x3~%int32 x4~%int32 y1~%int32 y2~%int32 y3~%int32 y4~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <image_points>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <image_points>))
  "Converts a ROS message object to a list"
  (cl:list 'image_points
    (cl:cons ':x1 (x1 msg))
    (cl:cons ':x2 (x2 msg))
    (cl:cons ':x3 (x3 msg))
    (cl:cons ':x4 (x4 msg))
    (cl:cons ':y1 (y1 msg))
    (cl:cons ':y2 (y2 msg))
    (cl:cons ':y3 (y3 msg))
    (cl:cons ':y4 (y4 msg))
))
