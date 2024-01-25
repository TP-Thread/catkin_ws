;; Auto-generated. Do not edit!


(when (boundp 'simulation::IRLock)
  (if (not (find-package "SIMULATION"))
    (make-package "SIMULATION"))
  (shadow 'IRLock (find-package "SIMULATION")))
(unless (find-package "SIMULATION::IRLOCK")
  (make-package "SIMULATION::IRLOCK"))

(in-package "ROS")
;;//! \htmlinclude IRLock.msg.html


(defclass simulation::IRLock
  :super ros::object
  :slots (_seq _stamp _frame_id _pos_x _pos_y _size_x _size_y ))

(defmethod simulation::IRLock
  (:init
   (&key
    ((:seq __seq) 0)
    ((:stamp __stamp) (instance ros::time :init))
    ((:frame_id __frame_id) "")
    ((:pos_x __pos_x) 0.0)
    ((:pos_y __pos_y) 0.0)
    ((:size_x __size_x) 0.0)
    ((:size_y __size_y) 0.0)
    )
   (send-super :init)
   (setq _seq (round __seq))
   (setq _stamp __stamp)
   (setq _frame_id (string __frame_id))
   (setq _pos_x (float __pos_x))
   (setq _pos_y (float __pos_y))
   (setq _size_x (float __size_x))
   (setq _size_y (float __size_y))
   self)
  (:seq
   (&optional __seq)
   (if __seq (setq _seq __seq)) _seq)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:pos_x
   (&optional __pos_x)
   (if __pos_x (setq _pos_x __pos_x)) _pos_x)
  (:pos_y
   (&optional __pos_y)
   (if __pos_y (setq _pos_y __pos_y)) _pos_y)
  (:size_x
   (&optional __size_x)
   (if __size_x (setq _size_x __size_x)) _size_x)
  (:size_y
   (&optional __size_y)
   (if __size_y (setq _size_y __size_y)) _size_y)
  (:serialization-length
   ()
   (+
    ;; uint32 _seq
    4
    ;; time _stamp
    8
    ;; string _frame_id
    4 (length _frame_id)
    ;; float64 _pos_x
    8
    ;; float64 _pos_y
    8
    ;; float64 _size_x
    8
    ;; float64 _size_y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _seq
       (write-long _seq s)
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;; float64 _pos_x
       (sys::poke _pos_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pos_y
       (sys::poke _pos_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _size_x
       (sys::poke _size_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _size_y
       (sys::poke _size_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _seq
     (setq _seq (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _pos_x
     (setq _pos_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pos_y
     (setq _pos_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _size_x
     (setq _size_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _size_y
     (setq _size_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get simulation::IRLock :md5sum-) "2fcca3ed526ec96516d6caa794588551")
(setf (get simulation::IRLock :datatype-) "simulation/IRLock")
(setf (get simulation::IRLock :definition-)
      "uint32 seq	
time stamp
string frame_id
float64 pos_x		
float64 pos_y		
float64 size_x		
float64 size_y		

")



(provide :simulation/IRLock "2fcca3ed526ec96516d6caa794588551")


