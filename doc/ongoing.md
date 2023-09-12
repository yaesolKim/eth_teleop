## Ongoing works

* Kalman filter for hand tracking noise?   
=======
* hand movement vector?   
* Kalman filter for hand tracking noise?  
* Set initial joint poses
--------

* surface roughness: 8 levels (orientation, translational velocity(mm/sec))    
  1-(0, 0) 2-(0, 40) 3-(0,80) 4-(45,0) 5-(90,0) 6-(45,80) 7-(90,40) 8-(90,80) 

0 degree for 1, 2, 3,   
45 degree for 4, 6    
90 degree for 5, 7, 8   

-> process this in Unreal? before publish PoseStamped data? YES!

-> let's just control the translational velocity -> this also in unreal?
* hand movement vector? -> manage this also in Unreal?

----

* need force init process...    
  measure the force after move to the initialized pose, and subtract this value to the realtime measured force.

----
