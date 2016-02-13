
(cl:in-package :asdf)

(defsystem "playground-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "coords" :depends-on ("_package_coords"))
    (:file "_package_coords" :depends-on ("_package"))
    (:file "WorldVelocities" :depends-on ("_package_WorldVelocities"))
    (:file "_package_WorldVelocities" :depends-on ("_package"))
    (:file "QPPS" :depends-on ("_package_QPPS"))
    (:file "_package_QPPS" :depends-on ("_package"))
    (:file "EncoderEstimates" :depends-on ("_package_EncoderEstimates"))
    (:file "_package_EncoderEstimates" :depends-on ("_package"))
  ))