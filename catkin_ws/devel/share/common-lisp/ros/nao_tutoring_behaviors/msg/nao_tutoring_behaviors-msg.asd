
(cl:in-package :asdf)

(defsystem "nao_tutoring_behaviors-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ControlMsg" :depends-on ("_package_ControlMsg"))
    (:file "_package_ControlMsg" :depends-on ("_package"))
    (:file "TabletMsg" :depends-on ("_package_TabletMsg"))
    (:file "_package_TabletMsg" :depends-on ("_package"))
  ))