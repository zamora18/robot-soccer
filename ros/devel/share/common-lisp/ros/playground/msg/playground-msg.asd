
(cl:in-package :asdf)

(defsystem "playground-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "coords" :depends-on ("_package_coords"))
    (:file "_package_coords" :depends-on ("_package"))
  ))