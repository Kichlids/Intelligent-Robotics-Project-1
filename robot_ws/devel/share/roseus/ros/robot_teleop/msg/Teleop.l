;; Auto-generated. Do not edit!


(when (boundp 'robot_teleop::Teleop)
  (if (not (find-package "ROBOT_TELEOP"))
    (make-package "ROBOT_TELEOP"))
  (shadow 'Teleop (find-package "ROBOT_TELEOP")))
(unless (find-package "ROBOT_TELEOP::TELEOP")
  (make-package "ROBOT_TELEOP::TELEOP"))

(in-package "ROS")
;;//! \htmlinclude Teleop.msg.html


(defclass robot_teleop::Teleop
  :super ros::object
  :slots (_is_teleop _command ))

(defmethod robot_teleop::Teleop
  (:init
   (&key
    ((:is_teleop __is_teleop) nil)
    ((:command __command) "")
    )
   (send-super :init)
   (setq _is_teleop __is_teleop)
   (setq _command (string __command))
   self)
  (:is_teleop
   (&optional __is_teleop)
   (if __is_teleop (setq _is_teleop __is_teleop)) _is_teleop)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; bool _is_teleop
    1
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_teleop
       (if _is_teleop (write-byte -1 s) (write-byte 0 s))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_teleop
     (setq _is_teleop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get robot_teleop::Teleop :md5sum-) "c9b3c6bb662db54412007781db0793c9")
(setf (get robot_teleop::Teleop :datatype-) "robot_teleop/Teleop")
(setf (get robot_teleop::Teleop :definition-)
      "bool is_teleop
string command

")



(provide :robot_teleop/Teleop "c9b3c6bb662db54412007781db0793c9")


