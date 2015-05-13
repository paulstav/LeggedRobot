
(cl:in-package :asdf)

(defsystem "legged_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Position" :depends-on ("_package_Position"))
    (:file "_package_Position" :depends-on ("_package"))
    (:file "Encoder" :depends-on ("_package_Encoder"))
    (:file "_package_Encoder" :depends-on ("_package"))
    (:file "PWM" :depends-on ("_package_PWM"))
    (:file "_package_PWM" :depends-on ("_package"))
  ))