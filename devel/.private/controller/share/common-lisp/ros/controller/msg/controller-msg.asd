
(cl:in-package :asdf)

(defsystem "controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "newMsg" :depends-on ("_package_newMsg"))
    (:file "_package_newMsg" :depends-on ("_package"))
  ))