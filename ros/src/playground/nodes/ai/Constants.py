# field constants. Distances measured in meters
field_length        = 3.40 # (134 in)  ## Old field was 3.68 # (12ft) 
field_width         = 2.38 # (93.875 in) ## Old field was 2.62 # (8.58 ft)
fourth_field_length = field_length/4
half_field          = 0

robot_width         = 0.175 # (7.0 in)
robot_half_width    = robot_width/2
# add constant that is the front of the robot, where we actually kick.

goal_box_width      = 0.619 # (24.375 in)
goal_box_length     = 0.127 # (5.0 in)
goal_position_home  = [-field_length/2, 0]
goal_position_opp   = [-goal_position_home[0], 0]
goal_score_threshold= 0.07 # .03 IN REAL LIFE?, 0.00 IN SIMULATOR

center_circle_radius= 0.25

des_dist_from_ball  = 0.0762 #(3.0in)
kick_dist           = 0.1524 #(6.0in)
push_ball_dist      = 0.2032 #(8.0in)
goalie_x_pos        = goal_position_home[0] + goal_box_length + robot_half_width
goalie_radius       = goal_box_width/2

dribble_distance    = robot_half_width + .05
kickable_distance   = 0.04
steal_ball_dist     = robot_half_width + 0.12

ally1_start_pos     = [-0.50, 0, 0]
ally2_start_pos     = [goal_position_home[0]+goalie_radius, 0, 0]

teammate_gap        = robot_width + 0.20

distance_behind_ball_for_kick       = robot_half_width # Used to be robot_width
distance_behind_ball_for_dribble    = robot_half_width + 0.05
open_for_pass_y_dist                = robot_width*4.5

field_x_lim         = field_length/2 - robot_width*0.60 # A little more than half_width
field_y_lim         = field_width/2 - robot_width*0.60 # A little more than half_width

own_goal_y_dist		= robot_width*1.5
own_goal_x_dist		= robot_width + 0.10

not_stuck_dist		= 0.10
