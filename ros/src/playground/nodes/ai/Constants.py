# field constants. Distances measured in meters
field_length       	= 3.68 # (12ft)
field_width        	= 2.62 # (8.58 ft)
fourth_field_length = field_length/4
half_field          = 0

robot_width        	= 0.175 # (7.0 in)
robot_half_width   	= robot_width/2
# add constant that is the front of the robot, where we actually kick.

goal_box_width     	= 0.660 # (26 in)
goal_box_length    	= 0.1143 # (4.5 in)
goal_position_home 	= [-field_length/2, 0] #this could change depending on camera
goal_position_opp  	= [-goal_position_home[0], 0]

des_dist_from_ball 	= 0.0762 #(3.0in)
kick_dist          	= 0.1524 #(6.0in)
goalie_x_pos       	= goal_position_home[0] + goal_box_length + robot_half_width


