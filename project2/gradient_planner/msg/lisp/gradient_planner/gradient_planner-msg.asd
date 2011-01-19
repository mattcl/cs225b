
(in-package :asdf)

(defsystem "gradient_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "GPPath" :depends-on ("_package"))
    (:file "_package_GPPath" :depends-on ("_package"))
    (:file "GPCostmapM" :depends-on ("_package"))
    (:file "_package_GPCostmapM" :depends-on ("_package"))
    ))
