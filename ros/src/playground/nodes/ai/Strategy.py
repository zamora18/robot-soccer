import numpy as np

# field constants. Distances measured in meters
_field_length       = 3.68 # (12ft)
_field_width        = 2.62 # (8.58 ft)
_robot_width        = 0.1841 # (7.25 in)
_robot_half_width   = _robot_width/2
_goal_box_width     = 0.660 # (26 in)
_goal_box_length    = 0.1143 #(4.5 in)
_goal_position_home = [-_field_length/2, 0, 0] #this could change depending on camera
_goal_position_opp  = [-_goal_position_home[0], 0, 0]
_des_dist_from_ball = 0.0762 #(3.0in)
_kick_dist          = 0.1524 #(6.0in)

_done = False

def choose_strategy(robot, ball):
    #return _strong_defense(robot, ball)
    return _strong_offense(robot, ball)


def _strong_offense(robot, ball):

    return _hack_offense(robot, ball)

    # for now we want to make one robot kick the ball into the open goal
    #
    # arctan2([y], [x])
    theta_ball_to_goal      = np.arctan2([ ball['yhat'] - _goal_position_opp[1] ], [ _goal_position_opp[0] - ball['xhat'] ])
    theta_ball_to_goal_deg  = theta_ball_to_goal*180/np.pi
    theta_bot_to_goal       = np.arctan2([ robot['yhat'] - _goal_position_opp[1] ], [ _goal_position_opp[0] - robot['xhat'] ])
    theta_bot_to_goal_deg   = theta_bot_to_goal*180/np.pi
    dist_from_ball          = _get_distance(robot, ball)

    #print("theta_ball_to_goal: {}\t\ttheta_bot_to_goal: {}\r".format(theta_ball_to_goal_deg, theta_bot_to_goal_deg))

    if (ball['xhat'] > _goal_position_opp[0]): #might have to tweak this a little bit
        #ball is in goal, don't move robot anywhere
        return (robot['xhat'], robot['yhat'], robot['thetahat'])
    else:
        # if robot is behind the ball and aligned towards goal
        if ( _close(theta_ball_to_goal_deg, theta_bot_to_goal_deg) and \
             #_close(theta_ball_to_goal_deg, robot['thetahat']) and \
            dist_from_ball <= (_des_dist_from_ball+_robot_half_width)): #taking into account 1/2 robot width
            #kick ball towards goal 6 inches
            x_c = ball['xhat'] + (_des_dist_from_ball+_kick_dist)*np.cos(theta_ball_to_goal)
            y_c = ball['yhat'] + (_des_dist_from_ball+_kick_dist)*np.sin(theta_ball_to_goal)
            return (x_c, y_c, theta_ball_to_goal_deg)
        else: 
            #get aligned with ball facing goal
            x_c = ball['xhat'] - (_des_dist_from_ball+_robot_half_width)*np.cos(theta_ball_to_goal)
            y_c = ball['yhat'] - (_des_dist_from_ball+_robot_half_width)*np.sin(theta_ball_to_goal)
            return (x_c, y_c, theta_ball_to_goal_deg)
            

def _strong_defense(robot, ball):
    #for now we want to make one robot defend the goal
    theta_c = np.arctan2([ _goal_position_home[1] + ball['yhat_future'] ], [ ball['xhat_future'] - _goal_position_opp[0] ])
    theta_c_deg = theta_c*180/np.pi
    x_c = _goal_position_home[0] + _goal_box_length + _robot_half_width #_goal_position_home[0] + (_goal_box_length+_robot_half_width)*np.cos(theta_c)
    
    if (ball['yhat_future'] > _goal_box_width/2):
        y_c = _goal_box_width/2
    elif (ball['yhat_future'] < -_goal_box_width/2):
        y_c = -_goal_box_width/2
    else:
        y_c = ball['yhat_future']

    #_goal_position_home[1] + (_goal_box_length+_robot_half_width)*np.sin(theta_c)
    theta_c_deg = 0
    return (x_c, y_c, theta_c_deg)
    
def _get_distance(object_1, object_2):
    x_dist = object_1['xhat'] - object_2['xhat']
    y_dist = object_1['yhat'] - object_2['yhat']
    distance = np.sqrt(x_dist**2 + y_dist**2)
    return distance


def _close(a, b, tolerance=20.0):
    return abs(a - b) <= tolerance


def _hack_offense(robot, ball):

    STOP_THRESH = 1.75 # m

    if robot['xhat'] > STOP_THRESH:
        return (0, 0, robot['thetahat'])

    theta_ball_to_goal      = np.arctan2([ ball['yhat'] - _goal_position_opp[1] ], [ _goal_position_opp[0] - ball['xhat'] ])

    robot_width = 0.1841
    offset_behind_ball = 2.5*robot_width

    global _done

    x = ball['xhat'] - offset_behind_ball*np.cos(theta_ball_to_goal)
    y = ball['yhat'] - offset_behind_ball*np.sin(-2*theta_ball_to_goal)

    if not _close(robot['xhat'], x, tolerance=0.100) or not _close(robot['yhat'], y, tolerance=0.100) and not _done:
        _done = True
        return (x, y, robot['thetahat'])

    # kick
    kick_point = (STOP_THRESH+.500, 0, robot['thetahat'])
    return kick_point
