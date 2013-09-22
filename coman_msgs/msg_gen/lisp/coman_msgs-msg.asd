
(cl:in-package :asdf)

(defsystem "coman_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointExtraInformation" :depends-on ("_package_JointExtraInformation"))
    (:file "_package_JointExtraInformation" :depends-on ("_package"))
    (:file "ForceTorque" :depends-on ("_package_ForceTorque"))
    (:file "_package_ForceTorque" :depends-on ("_package"))
    (:file "PID" :depends-on ("_package_PID"))
    (:file "_package_PID" :depends-on ("_package"))
  ))