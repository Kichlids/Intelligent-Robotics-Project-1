
(cl:in-package :asdf)

(defsystem "robot_teleop-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Teleop" :depends-on ("_package_Teleop"))
    (:file "_package_Teleop" :depends-on ("_package"))
  ))