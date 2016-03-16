import numpy as np

import Skills
import Utilities
import Constants


class ShootState:
    setup = 0
    attack = 1
    shoot = 2

_beginning_trick_shot = False


_shoot_state = ShootState.setup
_trick_state = ShootState.setup

def _robot_close_to_point(robot, x, y, theta):
    return Utilities.close(x, robot['xhat'], tolerance = .20) and Utilities.close(y, robot['yhat'], tolerance=.20) \
                and Utilities.close(theta, robot['thetahat'], tolerance = 20)

def shoot(robot, ball, distance_from_center):
    """ this sets up for a shot at a distance from the center. Distance from center should be between -1 and 1 (really like .75 and -.75).
        0 signifies straight on, 1 is top corner and -1 is bottom corner
        it also attacks the ball then actuates the kicker"""


    global _shoot_state

    # this is the desired setup point, the whole state machine needs it so it is
    # calculated here
    desired_setup_c = Skills.set_up_kick_facing_goal(ball, distance_from_center)

    # transition states

    # set up state
    if(_shoot_state == ShootState.setup):
        # if the robot is close enough to the correct angle and its in front of the ball change to the attack state
        if _robot_close_to_point(robot, *desired_setup_c) and not Utilities.is_ball_behind_robot(robot, ball):
            _shoot_state = ShootState.attack


    elif _shoot_state == ShootState.attack:
        # get the distance to the ball

        distance_to_ball = Utilities.get_distance_between_points(robot['xhat'], robot['yhat'], ball['xhat'], ball['yhat'])

        # if the ball is behind the robot, go back to set up
        if (Utilities.is_ball_behind_robot(robot, ball)):
            _shoot_state = ShootState.setup
        # if the ball is close enough, go to the shoot state
        elif(distance_to_ball < Constants.robot_width * (3.0/4.0)):
            _shoot_state = ShootState.shoot

    # if we just shot, go back to setup
    elif _shoot_state == ShootState.shoot:
        _shoot_state = ShootState.setup

    # default state, go to setup
    else:
        _shoot_state = ShootState.setup

    # action
    # go to the desired setup location
    if(_shoot_state == ShootState.setup):
        return desired_setup_c

    # attack the ball
    elif  _shoot_state == ShootState.attack:
        return Skills.attack_ball(robot,ball)

    # GGGGGOOOOOOOOOOOAAAAAAAAAALLLLLLLLLLLL!!!!!!!!!!!!!!!!!!
    elif _shoot_state == ShootState.shoot:
        print "KICKING"
        Skills.kick()
        return Skills.attack_ball(robot,ball) # keep attacking the ball as you kick

    # wait for state machine to start
    else:
        print ('default')
        return robot['xhat'], robot['yhat'], robot['thetahat']




def avoid_own_goal(robot, ball):
    dist_to_avoid_collision = Constants.robot_width
    #Checks to see if ball is still behind the robot, and moves to a distance perpendicular to the ball
    #if (ball['xhat'] < robot['xhat']):
    x_c = ball['xhat_future']
    if (ball['yhat_future'] > 0):
        y_c = ball['yhat_future']+dist_to_avoid_collision
        if (abs(y_c) > Constants.field_width/2): 
            y_c = ball['yhat_future']-dist_to_avoid_collision
    else:
        y_c = ball['yhat_future']-dist_to_avoid_collision
        if (abs(y_c) > Constants.field_width/2): 
            y_c = ball['yhat_future']+dist_to_avoid_collision

    theta_c = Utilities.get_angle_between_points(robot['xhat'], robot['yhat'], x_c, y_c)
    theta_c = Utilities.rad_to_deg(theta_c)
    return (x_c, y_c, theta_c)
    #else:
      #  dist_to_maintain = 0.75 # In my mind I'm trying to be 75% closer to the ball
      #  return Skills.stay_between_points_at_distance(Constants.goal_position_home[0], Constants.goal_position_home[1], ball['xhat_future'], ball['yhat_future'], dist_to_maintain)



def play_goalie (robot, ball):
    theta_c = Utilities.get_angle_between_points(Constants.goal_position_home[0], Constants.goal_position_home[1], ball['xhat_future'], ball['yhat_future'])
    theta_c_deg = Utilities.rad_to_deg(theta_c)
    #x_c =  Constants.goalie_x_pos
    x_c = Constants.goal_position_home[0] + Constants.goalie_radius*np.cos(theta_c)

    # defends at yhat future
    y_c = Constants.goal_position_home[1] + Constants.goalie_radius*np.sin(theta_c)
    return (x_c, y_c, theta_c_deg)





def trick_play(robot, ball):

    global _trick_state
    global _beginning_trick_shot

    distance_to_ball = Utilities.get_distance_between_points(robot['xhat'], robot['yhat'], ball['xhat'], ball['yhat'])

    set_up_distance = 0.4
    theta_c = Utilities.get_angle_between_points(ball['xhat'], ball['yhat'], Constants.field_length/2, Constants.field_width*5/6-ball['yhat'])
    x_c_before = ball['xhat']-set_up_distance*np.cos(theta_c)
    y_c_before = ball['yhat']-set_up_distance*np.sin(theta_c)

    x_c_after = ball['xhat']+set_up_distance*np.cos(theta_c)
    y_c_after = ball['yhat']+set_up_distance*np.sin(theta_c)

    theta_c = Utilities.rad_to_deg(theta_c)

    # transition
    if _trick_state == ShootState.setup:
        if _robot_close_to_point(robot, x_c_before, y_c_before, theta_c):
            _trick_state = ShootState.attack

    elif _trick_state == ShootState.attack:
        if(distance_to_ball < Constants.robot_width * (3.8/4.0)):
            _trick_state = ShootState.shoot

    elif _trick_state == ShootState.shoot:
        _trick_state == ShootState.setup
    else:
        _trick_state = ShootState.setup

    

    # Moore output
    if _trick_state == ShootState.setup:
        return (x_c_before, y_c_before, theta_c)

    elif _trick_state == ShootState.attack:
        return (x_c_after, y_c_after, theta_c)

    elif _trick_state == ShootState.shoot:
        Skills.kick()
        _beginning_trick_shot = True
        return (x_c_before, y_c_before, theta_c)

    else:
        return (x_c_before, y_c_before, theta_c)        

def is_trick_play_complete():
    global _beginning_trick_shot
    return _beginning_trick_shot


# def advance_ball(robot, opponent, ball):


