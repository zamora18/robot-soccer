import numpy as np

# field constants
_field_length       = 3.048 # meters (10ft)
_field_width        = 1.52  # meters (5 ft)
_robot_width        = 0.1841 # meters (7.25in)
_robot_half_width   = _robot_width/2
_goal_width         = _field_width/3
_goal_box_length    = _field_length/10 + 0.100
_goal_position_home = [-_field_length/2, 0, 0] #this could change depending on camera
_goal_position_opp  = [-_goal_position_home[0], 0, 0]
_dist_behind_ball   = 0.0762 # meters (3.0in)
_kick_dist          = 0.1524 # meters (6.0in)


def choose_strategy(robot, ball):
    return _strong_defense(robot, ball)
    return _strong_offense(robot, ball)


def _strong_offense(robot, ball):
    # for now we want to make one robot kick the ball into the open goal
    #
    # arctan2([y], [x])
    theta_ball_to_goal      = np.arctan2([ ball['yhat'] - _goal_position_opp[1] ], [ _goal_position_opp[0] - ball['xhat'] ])
    theta_ball_to_goal_deg  = theta_ball_to_goal*180/np.pi
    theta_bot_to_goal       = np.arctan2([ robot['yhat'] - _goal_position_opp[1] ], [ _goal_position_opp[0] - robot['xhat'] ])
    theta_bot_to_goal_deg   = theta_bot_to_goal*180/np.pi
    dist_from_ball          = _get_distance(robot, ball)

    if (ball['xhat'] > _goal_position_opp[0]): #might have to tweak this a little bit
        #ball is in goal, don't move robot anywhere
        return (robot['xhat'], robot['yhat'], robot['thetahat'])
    else:
        # if robot is behind the ball and aligned towards goal
        #if (theta_ball_to_goal_deg == robot['thetahat'] and \
         #   theta_bot_to_goal_deg == theta_ball_to_goal_deg and \
          #  dist_from_ball <= (_dist_behind_ball+_robot_half_width)): #taking into account 1/2 robot width
          #  #kick ball towards goal 6 inches
          #  x_c = ball['xhat'] + (_dist_behind_ball+_kick_dist)*np.cos(theta_ball_to_goal)
          #  y_c = ball['yhat'] + (_dist_behind_ball+_kick_dist)*np.sin(theta_ball_to_goal)
          #  return (x_c, y_c, theta_ball_to_goal_deg)
       # else: 
            # get aligned with ball facing goal
            x_c = ball['xhat'] - (_dist_behind_ball+_robot_half_width)*np.cos(theta_ball_to_goal)
            y_c = ball['yhat'] - (_dist_behind_ball+_robot_half_width)*np.sin(theta_ball_to_goal)
            return (x_c, y_c, theta_ball_to_goal_deg)
            

def _strong_defense(robot, ball):
    #for not we want to make one robot defend the goal
    theta_c = np.arctan2([ _goal_position_home[1] + ball['yhat'] ], [ ball['xhat'] - _goal_position_opp[0] ])
    theta_c_deg = theta_c*180/np.pi
    x_c = _goal_position_home[0] + (_goal_box_length+_robot_half_width)*np.cos(theta_c)
    y_c = _goal_position_home[1] + (_goal_box_length+_robot_half_width)*np.sin(theta_c)
    return (x_c, y_c, theta_c_deg)
    
def _get_distance(object_1, object_2):
    x_dist = object_1['xhat'] - object_2['xhat']
    y_dist = object_1['yhat'] - object_2['yhat']
    distance = np.sqrt(x_dist**2 + y_dist**2)
    return distance







