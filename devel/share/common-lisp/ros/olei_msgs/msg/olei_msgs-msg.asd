
(cl:in-package :asdf)

(defsystem "olei_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "oleiPacket" :depends-on ("_package_oleiPacket"))
    (:file "_package_oleiPacket" :depends-on ("_package"))
    (:file "oleiScan" :depends-on ("_package_oleiScan"))
    (:file "_package_oleiScan" :depends-on ("_package"))
  ))