from lib import * 
import math

# density of common materials
Aluminum        = 2.7  
Ceramic         = 2000
carbon_steel    = 7.8
Glass           = 2.6
Plastic         = 0.92 
Wood            = 0.6
wall            = 1500

height = 3
width  = 4.2
depth  = 1.5

mb = ModelBuilder()

# Wall's
wall_depth = 0.2

back_wall   = RectangularObj( "back_wall",   wall, width,		wall_depth, height + wall_depth)
ceeling     = RectangularObj( "ceeling",     wall, width, 		depth, 		wall_depth  )
left_wall   = RectangularObj( "left_wall",   wall, wall_depth, 	depth, 		height )
wright_wall = RectangularObj( "wright_wall", wall, wall_depth, 	depth, 		height )

back_wall.set_loc	(width/2, 				-wall_depth/2, 	(height + wall_depth)/2)
ceeling.set_loc	 	(width/2, 				depth/2, 		height+(wall_depth/2))
left_wall.set_loc	(wall_depth/2, 			(depth)/2,		height/2)
wright_wall.set_loc	(width-(wall_depth/2), 	(depth)/2, 		height/2)

mb.add( back_wall.get_link()).add( ceeling.get_link()).add( left_wall.get_link()).add( wright_wall.get_link()).add( ceeling.get_fixed_joint(back_wall, (0, -1, 0))).add( left_wall.get_fixed_joint(back_wall, (0, -1, 0))).add( wright_wall.get_fixed_joint(back_wall, (0, -1, 0))) 



# cabinet
cabinet_depth  = depth/3
cabinet_length = height/3 
cabinet_width  = width - wall_depth * 2

top_cabinet   	= RectangularObj( "top_cabinet",    Wood, cabinet_width,	cabinet_depth, wall_depth/4         )
back_cabinet  	= RectangularObj( "back_cabinet",  	Wood, cabinet_width,	cabinet_depth, cabinet_length       )
front_cabinet 	= RectangularObj( "front_cabinet",  Wood, cabinet_width,	cabinet_depth, cabinet_length  	    )
robot_carrier 	= RectangularObj( "robot_carrier",  Wood, cabinet_width,	cabinet_depth, (cabinet_length*4)/5 )

top_cabinet.set_loc   ( width/2, cabinet_depth/2, 	   (cabinet_length*7)/4 )
back_cabinet.set_loc  ( width/2, cabinet_depth/2, 	   cabinet_length/2     )
robot_carrier.set_loc ( width/2, (cabinet_depth*3)/2,  (cabinet_length*2)/5 )
front_cabinet.set_loc ( width/2, (cabinet_depth*5)/2,  cabinet_length/2     )

mb.add( back_cabinet.get_link())
mb.add( front_cabinet.get_link())
mb.add( top_cabinet.get_link())
mb.add( robot_carrier.get_link())

mb.add( back_cabinet.get_fixed_joint(back_wall, (0, -1, 0)))
mb.add( top_cabinet.get_fixed_joint(back_wall, (0, -1, 0)))
mb.add( robot_carrier.get_fixed_joint(back_cabinet, (0, -1, 0)))
mb.add( front_cabinet.get_fixed_joint(robot_carrier, (0, -1, 0)))


# Robot Arm
arm_base_length = 0.2
arm_base_radius = 0.2

arm_length = 0.25
arm_radius = arm_base_radius/6

arm_rotetor_length = 0.25
arm_rotetor_radius = arm_base_radius/6

arm_base 	= CylindericalObj( "arm_base",    Plastic, arm_base_length,		arm_base_radius    )
arm 	 	= CylindericalObj( "arm",   	  Plastic, arm_length,			arm_radius 		   )
arm_rotetor = CylindericalObj( "arm_rotetor", Plastic, arm_rotetor_length,	arm_rotetor_radius )

arm_base_pos_z 		= (cabinet_length/2) + (cabinet_depth/2) + (arm_base_length/2)
arm_pos_z 	  	 	= arm_base_pos_z + (arm_base_length/2)+ (arm_length/2)
arm_rotetor_pos_z 	= arm_pos_z + (arm_length)/2 + (arm_rotetor_length)/2

arm_base.set_loc( width/2, (cabinet_depth*3)/2, arm_base_pos_z)
arm.set_loc( width/2, (cabinet_depth*3)/2, arm_pos_z)
arm_rotetor.set_loc( width/2, (cabinet_depth*3)/2, arm_rotetor_pos_z)

mb.add( arm_base.get_link())
mb.add( arm.get_link())
mb.add( arm_rotetor.get_link())

mb.add( arm_base.get_prismatic_joint( robot_carrier, Orientation(1, 0, 0), 1.6, -1.6))
mb.add( arm.get_prismatic_joint( arm_base, Orientation(0, 0, 1), 1.6, -1.6))
mb.add( arm_rotetor.get_rovolute_joint(	arm, Orientation(1, 0, 0), math.pi, -math.pi))


#Gripper

wrist_length = arm_length/10
wrist_radius = (arm_radius*4)/5

finger_height = arm_length/8
finger_width  = arm_length/32
finger_depth  = arm_length/3


wrist_rx = CylindericalObj( "wrist_rx", Plastic, wrist_length/3, wrist_radius )
wrist_ry = CylindericalObj( "wrist_ry", Plastic, wrist_length/3, wrist_radius )
wrist_rz = CylindericalObj( "wrist_rz", Plastic, wrist_length/3, wrist_radius )
finger1  = RectangularObj ( "finger1",  Plastic, finger_width, finger_height, finger_depth )
finger2  = RectangularObj ( "finger2",  Plastic, finger_width, finger_height, finger_depth )

wrist_pos_z  = arm_rotetor_pos_z + (arm_rotetor_length/2)
finger_pos_z = wrist_pos_z + finger_depth/2 + wrist_length/2
space_bn_fingers  = arm_length/12


wrist_rx.set_loc ( width/2, (cabinet_depth*3)/2, wrist_pos_z + (wrist_length/6))
wrist_ry.set_loc ( width/2, (cabinet_depth*3)/2, wrist_pos_z + (wrist_length/2))
wrist_rz.set_loc ( width/2, (cabinet_depth*3)/2, wrist_pos_z + (wrist_length*5)/6)
finger1.set_loc ( width/2 - (space_bn_fingers + finger_width)/2, (cabinet_depth*3)/2, finger_pos_z)
finger2.set_loc ( width/2 + (space_bn_fingers + finger_width)/2, (cabinet_depth*3)/2, finger_pos_z)

finger1.add_sensor()
finger2.add_sensor()


mb.add( wrist_rx.get_link())
mb.add( wrist_ry.get_link())
mb.add( wrist_rz.get_link())
mb.add( finger1.get_link())
mb.add( finger2.get_link())



mb.add( wrist_rx.get_rovolute_joint( arm_rotetor, Orientation(0, 0, 1), math.pi,   -math.pi  ))
mb.add( wrist_ry.get_rovolute_joint( wrist_rx,  Orientation(0, 0, 1), math.pi,   -math.pi  ))
mb.add( wrist_rz.get_rovolute_joint( wrist_ry,  Orientation(0, 0, 1), math.pi,   -math.pi  ))
mb.add( finger1.get_rovolute_joint( wrist_rz,   Orientation(0, 1, 0), math.pi/2, -math.pi/2))
mb.add( finger2.get_rovolute_joint( wrist_rz,   Orientation(0, 1, 0), math.pi/2, -math.pi/2))


mb.add(Plugin("arm_controller", "libarm_controller.so", {}))
mb.add(Plugin("gripper_controller", "libgripper_controller.so", {}))
mb.build()


# For later use by FK
print ("arm_length 			= " + format(arm_length, '.8f'))
print ("arm_base_length 	= " + format(arm_base_length, '.8f'))
print ("wrist_length 		= " + format(wrist_length, '.8f'))



