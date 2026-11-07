
(cl:in-package :asdf)

(defsystem "collab_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "KeyboardInput" :depends-on ("_package_KeyboardInput"))
    (:file "_package_KeyboardInput" :depends-on ("_package"))
  ))