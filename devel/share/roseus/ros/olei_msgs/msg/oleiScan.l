;; Auto-generated. Do not edit!


(when (boundp 'olei_msgs::oleiScan)
  (if (not (find-package "OLEI_MSGS"))
    (make-package "OLEI_MSGS"))
  (shadow 'oleiScan (find-package "OLEI_MSGS")))
(unless (find-package "OLEI_MSGS::OLEISCAN")
  (make-package "OLEI_MSGS::OLEISCAN"))

(in-package "ROS")
;;//! \htmlinclude oleiScan.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass olei_msgs::oleiScan
  :super ros::object
  :slots (_header _packets ))

(defmethod olei_msgs::oleiScan
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:packets __packets) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _packets __packets)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:packets
   (&rest __packets)
   (if (keywordp (car __packets))
       (send* _packets __packets)
     (progn
       (if __packets (setq _packets (car __packets)))
       _packets)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; olei_msgs/oleiPacket[] _packets
    (apply #'+ (send-all _packets :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; olei_msgs/oleiPacket[] _packets
     (write-long (length _packets) s)
     (dolist (elem _packets)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; olei_msgs/oleiPacket[] _packets
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _packets (let (r) (dotimes (i n) (push (instance olei_msgs::oleiPacket :init) r)) r))
     (dolist (elem- _packets)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get olei_msgs::oleiScan :md5sum-) "99104d6cdcf01a9d450db883042f49ea")
(setf (get olei_msgs::oleiScan :datatype-) "olei_msgs/oleiScan")
(setf (get olei_msgs::oleiScan :definition-)
      "# olei LIDAR scan packets.

Header           header         # standard ROS message header
oleiPacket[] packets        # vector of raw packets

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: olei_msgs/oleiPacket
# Raw olei LIDAR packet.

time stamp              # packet timestamp
uint8[1240] data        # packet contents


")



(provide :olei_msgs/oleiScan "99104d6cdcf01a9d450db883042f49ea")


