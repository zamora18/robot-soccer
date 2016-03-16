import numpy as np

import Skills
import Utilities
import Constants


class ShootState:
    setup = 0
    attack = 1
    shoot = 2


_shoot_state = ShootState.setup

def _robot_close_to_point(robot, x, y, theta):
    return Utilities.close(x, robot['xhat'], tolerance=.15) and Utilities.close(y, robot['yhat'], tolerance=.15) \
                and Utilities.close(theta, robot['thetahat'], tolerance = 15)

def shoot(robot, ball, distance_from_center):
    global _shoot_state



    #transitions
    # new state machine
    desired_setup_c = Skills.set_up_kick_facing_goal(ball, distance_from_center)

    # if the robot is lined up with goal for shot, attack the ball
    if(_shoot_state == ShootState.setup):
        if _robot_close_to_point(robot, *desired_setup_c) and not Utilities.is_ball_behind_robot(robot,ball):
            _shoot_state = ShootState.attack
        elif Utilities.is_ball_behind_robot(robot,ball):
            _shoot_state = ShootState.setup

    # if the ball is right next to the front of the robot go into the shoot state
    elif(_shoot_state == ShootState.attack):

        distance_to_ball = Utilities.get_distance_between_points(robot['xhat'], robot['yhat'], ball['xhat'], ball['yhat'])
        if Utilities.is_ball_behind_robot(robot, ball):
            _shoot_state = ShootState.setup
        elif(distance_to_ball < Constants.robot_width * (3.0/4.0)):
            _shoot_state = ShootState.shoot

    # if we just actuated, go to setup phase
    elif(_shoot_state == ShootState.setup):
        _shoot_state = ShootState.setup

    # action
    if(_shoot_state == ShootState.setup):
        return desired_setup_c

    elif  _shoot_state == ShootState.attack:
        return Skills.attack_ball(robot, ball)

    elif _shoot_state == ShootState.shoot:
        print "KICKING"
        Skills.kick()
        return Skills.attack_ball(robot,ball) # keep attacking the ball as you kick
    else:
        print ('default')
        return robot['xhat'], robot['yhat'], robot['thetahat']

    # desired_setup_c = Skills.set_up_kick_facing_goal(ball, distance_from_center)

    # # transition

    # #set up state
    # if(_shoot_state == ShootState.setup):
    #     x, y, theta = Skills.set_up_kick_facing_goal(ball, 0)

    #     # if the robot is close enough to the correct angle and its in front of the ball change to the attack state
    #     if _robot_close(robot, *desired_setup_c) and not Utilities.is_ball_behind_robot(robot, ball):
    #         _shoot_state = ShootState.attack


    # elif _shoot_state == ShootState.attack:
    #     c = Utilities.get_distance_between_points(robot['xhat'], robot['yhat'], ball['xhat'], ball['yhat'])

    #     # if the ball is behind the robot, go back to set up
    #     if (Utilities.is_ball_behind_robot(robot, ball)):
    #         _shoot_state = ShootState.setup
    #     elif(c < Constants.robot_width * (3.0/4.0)):
    #         _shoot_state = ShootState.shoot

    # elif _shoot_state == ShootState.shoot:
    #     _shoot_state = ShootState.setup

    # else:
    #     _shoot_state = ShootState.setup

    # # action
    # if(_shoot_state == ShootState.setup):
    #     return desired_setup_c

    # elif  _shoot_state == ShootState.attack:
    #     return Skills.attack_ball(robot,ball)

    # elif _shoot_state == ShootState.shoot:
    #     print "KICKING"
    #     Skills.kick()
    #     return Skills.attack_ball(robot,ball) # keep attacking the ball as you kick
    # else:
    #     print ('default')
    #     return robot['xhat'], robot['yhat'], robot['thetahat']




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
    pass


# def advance_ball(robot, opponent, ball):


