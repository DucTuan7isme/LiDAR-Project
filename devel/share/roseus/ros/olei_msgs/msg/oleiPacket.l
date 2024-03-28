;; Auto-generated. Do not edit!


(when (boundp 'olei_msgs::oleiPacket)
  (if (not (find-package "OLEI_MSGS"))
    (make-package "OLEI_MSGS"))
  (shadow 'oleiPacket (find-package "OLEI_MSGS")))
(unless (find-package "OLEI_MSGS::OLEIPACKET")
  (make-package "OLEI_MSGS::OLEIPACKET"))

(in-package "ROS")
;;//! \htmlinclude oleiPacket.msg.html


(defclass olei_msgs::oleiPacket
  :super ros::object
  :slots (_stamp _data ))

(defmethod olei_msgs::oleiPacket
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:data __data) (make-array 1240 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _data __data)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; uint8[1240] _data
    (* 1    1240)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; uint8[1240] _data
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[1240] _data
   (setq _data (make-array 1240 :element-type :char))
   (replace _data buf :start2 ptr-) (incf ptr- 1240)
   ;;
   self)
  )

(setf (get olei_msgs::oleiPacket :md5sum-) "b3af84facd1013c90b679a5bc5454fd6")
(setf (get olei_msgs::oleiPacket :datatype-) "olei_msgs/oleiPacket")
(setf (get olei_msgs::oleiPacket :definition-)
      "# Raw olei LIDAR packet.

time stamp              # packet timestamp
uint8[1240] data        # packet contents


")



(provide :olei_msgs/oleiPacket "b3af84facd1013c90b679a5bc5454fd6")


