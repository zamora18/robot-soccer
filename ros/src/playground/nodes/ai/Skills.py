import os
import numpy as np


# kicks solenoid, 1 then 0? vector krum does 0 then 1, is sleep long enough to fully acutate?
def kick():
    os.system("echo 1 > /sys/class/gpio/gpio200/value; sleep .1; echo 0 > /sys/class/gpio/gpio200/value")

# distance should be between 0 and 1, scaled from the first point
# example follow ball 2/3 distance between goal and ball
def stay_between_points_at_distance(x1, y1, x2, y2, distance):

    # a, b and c are lengths of the side of a right triangle betweem p1 and p2
    a = x2 - x1
    b = y2 - y1
    c = np.sqrt(a**2 + b**2)

    # theta is the angle between y=0 line at our goal to the ball
    theta = np.arctan2(b,a)

    cprime = c*(1-distance)

    # aprime is the length of the simalar triangle with hypotenous d
    aprime = cprime*np.cos(theta)
    # bprime is the height of the simalar triangle with hypotenous d
    bprime = cprime*np.sin(theta)

    x_desired = x2 - aprime
    y_desired = y2 - bprime

    theta = theta*180/np.pi

    return (x_desired, y_desired, theta)







