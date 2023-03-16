;; Auto-generated. Do not edit!


(when (boundp 'realtime_detect_pkg::image_points)
  (if (not (find-package "REALTIME_DETECT_PKG"))
    (make-package "REALTIME_DETECT_PKG"))
  (shadow 'image_points (find-package "REALTIME_DETECT_PKG")))
(unless (find-package "REALTIME_DETECT_PKG::IMAGE_POINTS")
  (make-package "REALTIME_DETECT_PKG::IMAGE_POINTS"))

(in-package "ROS")
;;//! \htmlinclude image_points.msg.html


(defclass realtime_detect_pkg::image_points
  :super ros::object
  :slots (_x1 _x2 _x3 _x4 _y1 _y2 _y3 _y4 ))

(defmethod realtime_detect_pkg::image_points
  (:init
   (&key
    ((:x1 __x1) 0)
    ((:x2 __x2) 0)
    ((:x3 __x3) 0)
    ((:x4 __x4) 0)
    ((:y1 __y1) 0)
    ((:y2 __y2) 0)
    ((:y3 __y3) 0)
    ((:y4 __y4) 0)
    )
   (send-super :init)
   (setq _x1 (round __x1))
   (setq _x2 (round __x2))
   (setq _x3 (round __x3))
   (setq _x4 (round __x4))
   (setq _y1 (round __y1))
   (setq _y2 (round __y2))
   (setq _y3 (round __y3))
   (setq _y4 (round __y4))
   self)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:x3
   (&optional __x3)
   (if __x3 (setq _x3 __x3)) _x3)
  (:x4
   (&optional __x4)
   (if __x4 (setq _x4 __x4)) _x4)
  (:y1
   (&optional __y1)
   (if __y1 (setq _y1 __y1)) _y1)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:y3
   (&optional __y3)
   (if __y3 (setq _y3 __y3)) _y3)
  (:y4
   (&optional __y4)
   (if __y4 (setq _y4 __y4)) _y4)
  (:serialization-length
   ()
   (+
    ;; int32 _x1
    4
    ;; int32 _x2
    4
    ;; int32 _x3
    4
    ;; int32 _x4
    4
    ;; int32 _y1
    4
    ;; int32 _y2
    4
    ;; int32 _y3
    4
    ;; int32 _y4
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _x1
       (write-long _x1 s)
     ;; int32 _x2
       (write-long _x2 s)
     ;; int32 _x3
       (write-long _x3 s)
     ;; int32 _x4
       (write-long _x4 s)
     ;; int32 _y1
       (write-long _y1 s)
     ;; int32 _y2
       (write-long _y2 s)
     ;; int32 _y3
       (write-long _y3 s)
     ;; int32 _y4
       (write-long _y4 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _x1
     (setq _x1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x2
     (setq _x2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x3
     (setq _x3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x4
     (setq _x4 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y1
     (setq _y1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y2
     (setq _y2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y3
     (setq _y3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y4
     (setq _y4 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get realtime_detect_pkg::image_points :md5sum-) "d50612e5b17d5b27e8f9e37d8fc9f6f6")
(setf (get realtime_detect_pkg::image_points :datatype-) "realtime_detect_pkg/image_points")
(setf (get realtime_detect_pkg::image_points :definition-)
      "int32 x1
int32 x2
int32 x3
int32 x4
int32 y1
int32 y2
int32 y3
int32 y4
")



(provide :realtime_detect_pkg/image_points "d50612e5b17d5b27e8f9e37d8fc9f6f6")


