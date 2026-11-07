;; Auto-generated. Do not edit!


(when (boundp 'collab_planner::KeyboardInput)
  (if (not (find-package "COLLAB_PLANNER"))
    (make-package "COLLAB_PLANNER"))
  (shadow 'KeyboardInput (find-package "COLLAB_PLANNER")))
(unless (find-package "COLLAB_PLANNER::KEYBOARDINPUT")
  (make-package "COLLAB_PLANNER::KEYBOARDINPUT"))
(unless (find-package "COLLAB_PLANNER::KEYBOARDINPUTREQUEST")
  (make-package "COLLAB_PLANNER::KEYBOARDINPUTREQUEST"))
(unless (find-package "COLLAB_PLANNER::KEYBOARDINPUTRESPONSE")
  (make-package "COLLAB_PLANNER::KEYBOARDINPUTRESPONSE"))

(in-package "ROS")





(defclass collab_planner::KeyboardInputRequest
  :super ros::object
  :slots (_data ))

(defmethod collab_planner::KeyboardInputRequest
  (:init
   (&key
    ((:data __data) "")
    )
   (send-super :init)
   (setq _data (string __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass collab_planner::KeyboardInputResponse
  :super ros::object
  :slots (_success ))

(defmethod collab_planner::KeyboardInputResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass collab_planner::KeyboardInput
  :super ros::object
  :slots ())

(setf (get collab_planner::KeyboardInput :md5sum-) "e7ac98ca304b04e45988c61adb6bc4ce")
(setf (get collab_planner::KeyboardInput :datatype-) "collab_planner/KeyboardInput")
(setf (get collab_planner::KeyboardInput :request) collab_planner::KeyboardInputRequest)
(setf (get collab_planner::KeyboardInput :response) collab_planner::KeyboardInputResponse)

(defmethod collab_planner::KeyboardInputRequest
  (:response () (instance collab_planner::KeyboardInputResponse :init)))

(setf (get collab_planner::KeyboardInputRequest :md5sum-) "e7ac98ca304b04e45988c61adb6bc4ce")
(setf (get collab_planner::KeyboardInputRequest :datatype-) "collab_planner/KeyboardInputRequest")
(setf (get collab_planner::KeyboardInputRequest :definition-)
      "string data
---
bool success


")

(setf (get collab_planner::KeyboardInputResponse :md5sum-) "e7ac98ca304b04e45988c61adb6bc4ce")
(setf (get collab_planner::KeyboardInputResponse :datatype-) "collab_planner/KeyboardInputResponse")
(setf (get collab_planner::KeyboardInputResponse :definition-)
      "string data
---
bool success


")



(provide :collab_planner/KeyboardInput "e7ac98ca304b04e45988c61adb6bc4ce")


