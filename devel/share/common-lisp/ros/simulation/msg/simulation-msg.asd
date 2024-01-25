
(cl:in-package :asdf)

(defsystem "simulation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Control" :depends-on ("_package_Control"))
    (:file "_package_Control" :depends-on ("_package"))
    (:file "IRLock" :depends-on ("_package_IRLock"))
    (:file "_package_IRLock" :depends-on ("_package"))
  ))