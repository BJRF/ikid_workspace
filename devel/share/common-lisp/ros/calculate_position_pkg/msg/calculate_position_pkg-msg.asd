
(cl:in-package :asdf)

(defsystem "calculate_position_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "image_points" :depends-on ("_package_image_points"))
    (:file "_package_image_points" :depends-on ("_package"))
  ))