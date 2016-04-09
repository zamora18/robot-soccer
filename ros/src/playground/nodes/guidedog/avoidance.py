import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'ai/'))
import Utilities
import Constants
import numpy as np


distancetoworryabout = Constants.robot_width * 2



def get_perpendicular_point(robot, dest, opp):
    
    

    x1 = float(robot[0])
    y1 = float(robot[1])
    x2 = float(dest[0])
    y2 = float(dest[1])
    x3 = float(opp[0])
    y3 = float(opp[1])

    mid = midpoint((x1,y1),(x2,y2))

    radius = Utilities.get_distance_between_points(x1,y1,x2,y2)/2
    distancetothird = Utilities.get_distance_between_points(mid[0], mid[1], x3,y3)

    if distancetothird > radius:
        return None

    k = ((y2-y1) * (x3-x1) - (x2-x1) * (y3-y1)) / ((y2-y1)**2 + (x2-x1)**2)
    x4 = x3 - k * (y2-y1)
    y4 = y3 + k * (x2-x1)

    return (x4,y4)

def midpoint(p1, p2):
    return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)



def do_i_need_to_avoid(p1,p2):
    global distancetoworryabout
    if Utilities.get_distance_between_points(p1[0], p1[1], p2[0],p2[1]) < distancetoworryabout:
        return True
    return False


def point_to_go_through(opp, perp_point):

    p1 = opp
    p2 = perp_point
    d = Utilities.get_distance_between_points(p1[0], p1[1], p2[0], p2[1])
    dnew = distancetoworryabout - d
    theta = Utilities.get_angle_between_points(p1[0], p1[1], p2[0], p2[1])
    

    theta = theta + np.pi

    a = dnew*np.cos(theta)
    b = dnew*np.sin(theta)

    x = p2[0] - a
    y = p2[1] - b

    return (x,y)


def above_or_below(robot, dest, opp):
    """returns true to go above robot or false to go below"""

    # opponent is to close to the top
    if opp[1] > (Constants.field_width/2 - Constants.robot_width * 2.5):
        return False
    # oppenent is to close to the bottom
    elif opp[1] < (-Constants.field_width/2 + Constants.robot_width * 2.5):
        return True

    # check to see if we should go above or below the robot
    robotdistance = abs(robot[1] - opp[1])
    destdistance = abs(dest[1] - opp[1])
    if robotdistance > destdistance:
        if robot[1] > opp[1]:
            return True
        else:
            return False
    else:
        if dest[1] > opp[1]:
            return True
        else:
            return False


def avoid(me, dest, opp):
    """
    """
    
    perp_point = get_perpendicular_point(me, dest, opp)

    if perp_point is None:
        return dest

    if do_i_need_to_avoid(opp, perp_point):
        # not sure what to do right now
        go_above = above_or_below(me, dest, opp)
        # eventually do this

        crit_point = point_to_go_through(opp, perp_point)

        angle = Utilities.get_angle_between_points(me[0], me[1], crit_point[0], crit_point[1])

        c = Utilities.get_distance_between_points(me[0], me[1], dest[0], dest[1])

        a = c*np.cos(angle)
        b = c*np.sin(angle)

        x = me[0] + a
        y = me[0] + b

        return (x,y)

    else:
        return dest


def main():
    robot = (1,1)
    dest = (-1,-1)

    opp = (0,.2)

    perp_point = get_perpendicular_point(robot, dest, opp)

    if perp_point is None:
        print "notevenclose"

    if do_i_need_to_avoid(opp, perp_point):

        # not sure what to do right now
        go_above = above_or_below(robot, dest, opp)
        # eventually do this

        crit_point = point_to_go_through(opp, perp_point)

        angle = Utilities.get_angle_between_points(robot[0], robot[1], crit_point[0], crit_point[1])

        c = Utilities.get_distance_between_points(robot[0], robot[1], dest[0], dest[1])

        a = c*np.cos(angle)
        b = c*np.sin(angle)

        x = robot[0] + a
        y = robot[0] + b

        print (x,y)

    else:
        print "do not need to avoid"

if __name__ == '__main__':
    main()
