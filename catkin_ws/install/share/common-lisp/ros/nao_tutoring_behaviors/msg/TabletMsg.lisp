; Auto-generated. Do not edit!


(cl:in-package nao_tutoring_behaviors-msg)


;//! \htmlinclude TabletMsg.msg.html

(cl:defclass <TabletMsg> (roslisp-msg-protocol:ros-message)
  ((msgType
    :reader msgType
    :initarg :msgType
    :type cl:string
    :initform "")
   (questionNum
    :reader questionNum
    :initarg :questionNum
    :type cl:integer
    :initform 0)
   (questionType
    :reader questionType
    :initarg :questionType
    :type cl:string
    :initform "")
   (robotSpeech
    :reader robotSpeech
    :initarg :robotSpeech
    :type cl:string
    :initform "")
   (otherInfo
    :reader otherInfo
    :initarg :otherInfo
    :type cl:string
    :initform ""))
)

(cl:defclass TabletMsg (<TabletMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TabletMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TabletMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_tutoring_behaviors-msg:<TabletMsg> is deprecated: use nao_tutoring_behaviors-msg:TabletMsg instead.")))

(cl:ensure-generic-function 'msgType-val :lambda-list '(m))
(cl:defmethod msgType-val ((m <TabletMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_tutoring_behaviors-msg:msgType-val is deprecated.  Use nao_tutoring_behaviors-msg:msgType instead.")
  (msgType m))

(cl:ensure-generic-function 'questionNum-val :lambda-list '(m))
(cl:defmethod questionNum-val ((m <TabletMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_tutoring_behaviors-msg:questionNum-val is deprecated.  Use nao_tutoring_behaviors-msg:questionNum instead.")
  (questionNum m))

(cl:ensure-generic-function 'questionType-val :lambda-list '(m))
(cl:defmethod questionType-val ((m <TabletMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_tutoring_behaviors-msg:questionType-val is deprecated.  Use nao_tutoring_behaviors-msg:questionType instead.")
  (questionType m))

(cl:ensure-generic-function 'robotSpeech-val :lambda-list '(m))
(cl:defmethod robotSpeech-val ((m <TabletMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_tutoring_behaviors-msg:robotSpeech-val is deprecated.  Use nao_tutoring_behaviors-msg:robotSpeech instead.")
  (robotSpeech m))

(cl:ensure-generic-function 'otherInfo-val :lambda-list '(m))
(cl:defmethod otherInfo-val ((m <TabletMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_tutoring_behaviors-msg:otherInfo-val is deprecated.  Use nao_tutoring_behaviors-msg:otherInfo instead.")
  (otherInfo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TabletMsg>) ostream)
  "Serializes a message object of type '<TabletMsg>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msgType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msgType))
  (cl:let* ((signed (cl:slot-value msg 'questionNum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'questionType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'questionType))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robotSpeech))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robotSpeech))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'otherInfo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'otherInfo))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TabletMsg>) istream)
  "Deserializes a message object of type '<TabletMsg>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msgType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msgType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'questionNum) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'questionType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'questionType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robotSpeech) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robotSpeech) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'otherInfo) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'otherInfo) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TabletMsg>)))
  "Returns string type for a message object of type '<TabletMsg>"
  "nao_tutoring_behaviors/TabletMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TabletMsg)))
  "Returns string type for a message object of type 'TabletMsg"
  "nao_tutoring_behaviors/TabletMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TabletMsg>)))
  "Returns md5sum for a message object of type '<TabletMsg>"
  "03de84d6bf38d762b3a2b1dcaee4fc76")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TabletMsg)))
  "Returns md5sum for a message object of type 'TabletMsg"
  "03de84d6bf38d762b3a2b1dcaee4fc76")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TabletMsg>)))
  "Returns full string definition for message of type '<TabletMsg>"
  (cl:format cl:nil "string msgType~%int64 questionNum~%string questionType~%string robotSpeech~%string otherInfo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TabletMsg)))
  "Returns full string definition for message of type 'TabletMsg"
  (cl:format cl:nil "string msgType~%int64 questionNum~%string questionType~%string robotSpeech~%string otherInfo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TabletMsg>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msgType))
     8
     4 (cl:length (cl:slot-value msg 'questionType))
     4 (cl:length (cl:slot-value msg 'robotSpeech))
     4 (cl:length (cl:slot-value msg 'otherInfo))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TabletMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'TabletMsg
    (cl:cons ':msgType (msgType msg))
    (cl:cons ':questionNum (questionNum msg))
    (cl:cons ':questionType (questionType msg))
    (cl:cons ':robotSpeech (robotSpeech msg))
    (cl:cons ':otherInfo (otherInfo msg))
))
