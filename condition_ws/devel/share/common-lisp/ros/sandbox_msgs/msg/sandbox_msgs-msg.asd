
(cl:in-package :asdf)

(defsystem "sandbox_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AprilObject" :depends-on ("_package_AprilObject"))
    (:file "_package_AprilObject" :depends-on ("_package"))
    (:file "ChassisCommand" :depends-on ("_package_ChassisCommand"))
    (:file "_package_ChassisCommand" :depends-on ("_package"))
    (:file "Trajectory" :depends-on ("_package_Trajectory"))
    (:file "_package_Trajectory" :depends-on ("_package"))
    (:file "TrajectoryPoint" :depends-on ("_package_TrajectoryPoint"))
    (:file "_package_TrajectoryPoint" :depends-on ("_package"))
  ))