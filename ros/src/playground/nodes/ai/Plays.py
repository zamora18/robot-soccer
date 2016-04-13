import numpy as np

import Skills
import Utilities
import Constants

class ShootState:
    setup   = 0
    attack  = 1
    shoot   = 2

class TrickState:
    setvars = 0
    setup   = 1
    attack  = 2
    shoot   = 3


_shoot_state        = ShootState.setup
_trick_state        = TrickState.setvars
_steal_ball_state   = ShootState.setup

_wait_steal_timer   = 0
_WAIT_STEAL_MAX     = 20
_ball_stuck_timer   = 0
_BALL_STUCK_MAX     = 300

_recently_kicked            = False
_kicker_wait_counter        = 0
_KICKER_WAIT_MAX            = 75 # 750 ms in between kicks. (3/4 of a second)

_offensive  = 0
_defensive  = 1
_neutral    = 2

_trick_x_before = None
_trick_y_before = None
_trick_x_after  = None
_trick_y_after  = None

_beginning_trick_shot = False


def beginning_trick_shot_done():
    global _beginning_trick_shot

    return _beginning_trick_shot


##########################################
# Plays mainly for "attacker" position:  #
##########################################

def shoot_on_goal(me, ball, distance_from_center, opponent1, opponent2):
    """ this sets up for a shot at a distance from the center. Distance from center should be between -1 and 1 (really like .75 and -.75).
        0 signifies straight on, 1 is top corner and -1 is bottom corner
        it also attacks the ball then actuates the kicker"""

    global _shoot_state
    global _ball_stuck_timer, _BALL_STUCK_MAX
    global _recently_kicked, _kicker_wait_counter, _KICKER_WAIT_MAX

    # this is the desired setup point, the whole state machine needs it so it is
    desired_setup_position = Skills.set_up_kick_facing_goal(ball, distance_from_center)#!!!!!!should we try and do future? So it predicts
    # get the distance to the ball
    (x_pos, y_pos) = Utilities.get_front_of_robot(me)
    distance_from_kicker_to_ball = Utilities.get_distance_between_points(x_pos, y_pos, ball.xhat, ball.yhat)

    # We don't want to kicker to actuate so often, so we need to wait for the counter
    # This will happen every time the function is called, to make sure we don't miss a kick in between transitions
    _kicker_wait_counter = _kicker_wait_counter + 1
    if _kicker_wait_counter >= _KICKER_WAIT_MAX:
        _recently_kicked = False
        _kicker_wait_counter = 0

    #########################
    ### transition states ###
    #########################
    if _shoot_state == ShootState.setup:
        # _recently_kicked = False # Usually if it's in the setup state, we will have enough time to actuate the kicker.
        _ball_stuck_timer = _ball_stuck_timer + 1
        # if the robot is close enough to the correct angle and its in front of the ball change to the attack state
        if Utilities.robot_close_to_point(me, *desired_setup_position): 
            if not Utilities.is_ball_behind_robot(me, ball): 
                _shoot_state = ShootState.attack

    elif _shoot_state == ShootState.attack:
        # if the ball is behind the robot, go back to set up
        if (Utilities.is_ball_behind_robot(me, ball) or distance_from_kicker_to_ball >= Constants.robot_width):
            _shoot_state = ShootState.setup
        # if the ball is close enough, go to the shoot state
        elif(distance_from_kicker_to_ball <=  Constants.kickable_distance):
            _shoot_state = ShootState.shoot

    elif _shoot_state == ShootState.shoot:
        # Always go to the setup right after so that it only kicks once.
        _shoot_state = ShootState.setup 

    # default state, go to setup
    else:
        _shoot_state = ShootState.setup

    ###############################
    ### Moore Outputs in states ###
    ###############################
    # go to the desired setup location
    
    if _shoot_state == ShootState.setup:
        if _ball_stuck_timer >= _BALL_STUCK_MAX:
            _ball_stuck_timer = 0
            return Skills.attack_ball(me, ball)
        else:
            return desired_setup_position

    # attack the ball
    elif  _shoot_state == ShootState.attack:
        # return Skills.attack_ball_towards_goal(me, ball, distance_from_center)
        return Skills.attack_ball(me, ball)

    elif _shoot_state == ShootState.shoot:
        if not Utilities.is_opp_too_close_to_kicker(me, opponent1, opponent2, ball):
            Skills.kick()
        else:
            print "Opponent too close and could damage our kicker"
        # return Skills.attack_ball_towards_goal(me, ball, distance_from_center) # keep attacking the ball as you kick
        return Skills.attack_ball(me, ball)

    # wait for state machine to start
    else:
        print ('default')
        return (me.xhat, me.yhat, me.thetahat)


def shoot_off_the_wall(me, ball):
    global _trick_state
    global _trick_x_before, _trick_x_after, _trick_y_before, _trick_y_after
    global _beginning_trick_shot

    (x,y) = Utilities.get_front_of_robot(me)
    distance_from_kicker_to_ball = Utilities.get_distance_between_points(x, y, ball.xhat, ball.yhat)
    set_up_distance = Constants.robot_half_width + Constants.robot_width 

    y_tweak_value = 8.0/10.0 # Ideally, we should aim for goal pos mirrored above/below us, but it won't be perfect so this should handle that
    if ball.yhat > 0:
        # will calculate kicking off of the "closest" wall, which is the top wall
        theta_c = Utilities.get_angle_between_points(ball.xhat, ball.yhat, Constants.field_length/2, Constants.field_width*y_tweak_value-ball.yhat)
    else:
        # will calculate kicking off of the bottom wall.
        theta_c = Utilities.get_angle_between_points(ball.xhat, ball.yhat, Constants.field_length/2, -Constants.field_width*y_tweak_value+ball.yhat)
    theta_c = Utilities.rad_to_deg(theta_c)

    #########################
    ### transition states ###
    #########################
    if _trick_state == TrickState.setvars:
        # Assign desired positions
        _trick_x_before = ball.xhat-set_up_distance*np.cos(theta_c)
        _trick_y_before = ball.yhat-set_up_distance*np.sin(theta_c)

        _trick_x_after = ball.xhat+set_up_distance*np.cos(theta_c)
        _trick_y_after = ball.yhat+set_up_distance*np.sin(theta_c)
        # Transition to setup state
        _trick_state = TrickState.setup

    elif _trick_state == TrickState.setup:
        if Utilities.robot_close_to_point(me, _trick_x_before, _trick_y_before, theta_c):
            _trick_state = TrickState.attack

    elif _trick_state == TrickState.attack:
        if(distance_from_kicker_to_ball <= Constants.kickable_distance):
            _trick_state = TrickState.shoot

    elif _trick_state == TrickState.shoot:
        if distance_from_kicker_to_ball > Constants.kickable_distance:
            _trick_state == TrickState.setvars

    else:
        _trick_state = TrickState.setvars

    
    ###############################
    ### Moore Outputs in states ###
    ###############################
    if _trick_state == TrickState.setup:
        return (_trick_x_before, _trick_y_before, theta_c)

    elif _trick_state == TrickState.attack:
        return (_trick_x_after, _trick_y_after, theta_c)

    elif _trick_state == TrickState.shoot:
        Skills.kick()
        _beginning_trick_shot = True
        return (_trick_x_after, _trick_y_after, theta_c)

    else:
        return (_trick_x_before, _trick_y_before, theta_c)


def steal_ball_from_opponent(me, opponent, ball):
    """
    Using xhat_future and yhat_future to predict where the ball will be, and 
    put itself in front of the opponent, facing opponent's goal, ready to kick the 
    ball towards their goal
    """
    global _steal_ball_state
    global _wait_steal_timer, _WAIT_STEAL_MAX

    # WE ARE USING FUTURE POSITIONS SO WE WILL BE WHERE THE BALL IS HOPEFULLY BEFORE THE OPPONENT
    theta_des = Utilities.get_angle_between_points(ball.xhat, ball.yhat, opponent.xhat, opponent.yhat)
    x_des = ball.xhat - Constants.steal_ball_dist*np.cos(theta_des)
    y_des = ball.yhat - Constants.steal_ball_dist*np.sin(theta_des)
    # but we need the kicker to get the current position so it will kick correctly
    (x,y) = Utilities.get_front_of_robot(me)
    distance_from_kicker_to_ball = Utilities.get_distance_between_points(x, y, ball.xhat, ball.yhat)



    #########################
    ### transition states ###
    #########################
    if _steal_ball_state == ShootState.setup:
        opp_dist_from_ball = Utilities.get_distance_between_points(opponent.xhat, opponent.yhat, ball.xhat_future, ball.yhat_future)
        if Utilities.robot_close_to_point(me, x_des, y_des, theta_des):
            _wait_steal_timer = _wait_steal_timer + 1
            if (opp_dist_from_ball >= Constants.steal_ball_dist or _wait_steal_timer >= _WAIT_STEAL_MAX):
                _wait_steal_timer = 0
                _steal_ball_state = ShootState.attack
    elif _steal_ball_state == ShootState.attack:
        if distance_from_kicker_to_ball <= Constants.kickable_distance:
            _steal_ball_state = ShootState.shoot
    elif _steal_ball_state == ShootState.shoot:
        if distance_from_kicker_to_ball > Constants.kickable_distance:
            _steal_ball_state = ShootState.setup
    else:
        _steal_ball_state = ShootState.setup

    ###############################
    ### Moore Outputs in states ###
    ###############################
    if _steal_ball_state == ShootState.setup:
        return (x_des, y_des, theta_des)
    elif _steal_ball_state == ShootState.attack:
        return Skills.attack_ball(me, ball)
    elif _steal_ball_state == ShootState.shoot:
        Skills.kick()
        return Skills.attack_ball(me, ball)
    else:
        return (me.xhat, me.yhat, Utilities.rad_to_deg(me.thetahat))


def stay_open_for_pass(me, my_teammate, ball):
    """
    Teammate(me) stays a 0.5 meters ahead of the ball and follows 'my_teammate' towards the goal
    'me' maintains a distance away from 'my_teammate' and normally should be 
    """
    if (ball.yhat > 0): 
        r_l_toggle = -1 
    else: 
        r_l_toggle = 1

    y_c = my_teammate.yhat + Constants.open_for_pass_y_dist*r_l_toggle
    x_c = ball.xhat + 0.50

    (x_c, y_c) = Utilities.limit_xy_passing(x_c, y_c)
    theta_c = -45*r_l_toggle
    return (x_c, y_c, theta_c)
    

def pass_to_teammate(me, my_teammate, ball):
    (x_pos, y_pos) = Utilities.get_front_of_robot(me)
    dist_to_ball = Utilities.get_distance_between_points(x_pos, y_pos, ball.xhat, ball.yhat)

    desired_setup = Skills.go_behind_ball_facing_target(ball, Constants.distance_behind_ball_for_kick, my_teammate.xhat_future, my_teammate.yhat_future)
    
    if Utilities.robot_close_to_point(me, *desired_setup):
        if dist_to_ball <= Constants.kickable_distance:
            Skills.kick()
        return Skills.attack_ball_towards_point(me, ball, my_teammate.xhat_future, my_teammate.yhat_future)
    else:
        return desired_setup

        

##########################################
# Plays mainly for "defender" position:  #
##########################################

def stay_at_midfield_follow_ball(me, opponent1, opponent2, ball):
    x_c = Constants.half_field
    y_c = ball.yhat_future
    theta = Utilities.get_angle_between_points(me.xhat, me.yhat, Constants.goal_position_opp[0], Constants.goal_position_opp[1])
    theta = Utilities.rad_to_deg(theta)
    return (x_c, y_c, theta)


def stay_at_front_quarter_follow_ball(me, opponent1, opponent2, ball):
    x_c = Constants.field_length/4
    y_c = ball.yhat_future
    theta = Utilities.get_angle_between_points(me.xhat, me.yhat, Constants.goal_position_opp[0], Constants.goal_position_opp[1])
    theta = Utilities.rad_to_deg(theta)
    return (x_c, y_c, theta)


def stay_between_points_at_distance(x1, y1, x2, y2, distance):
    """
    Distance should be between 0 and 1, scaled from the first point
    example follow ball 2/3 distance from goal to ball
    """
    theta = Utilities.get_angle_between_points(x1, y1, x2, y2)
    c = Utilities.get_distance_between_points(x1, y1, x2, y2)
    cprime = c*(1-distance)

    # aprime is the length of the simalar triangle with hypotenuse cprime
    aprime = cprime*np.cos(theta)
    # bprime is the height of the simalar triangle with hypotenuse cprime
    bprime = cprime*np.sin(theta)

    x_desired = x2 - aprime
    y_desired = y2 - bprime
    theta = Utilities.rad_to_deg(theta)

    return (x_desired, y_desired, theta)

##########################################
# Plays mainly for "goalie" position:    #
##########################################
