import sys, time

import numpy as np
import matplotlib.pyplot as plt

sys.path.append('../../ros/src/playground/nodes/guidedog/')
import astar

field_length = 0.100 #3.68;
field_height = 0.100 #1.52;
separation = 0.010 #0.200;

def main():

    graph = astar.init_grid(field_length, field_height, separation)

    Nx = field_length/separation;
    Ny = field_height/separation;
    x_max = (field_length/2);
    y_max = (field_height/2);
    x = np.linspace(-x_max, x_max, Nx);
    y = np.linspace(-y_max, y_max, Ny);
    X, Y = np.meshgrid(x, y);

    plt.scatter(X, Y)
    # plt.grid()
    plt.show()

if __name__ == "__main__":
    main()