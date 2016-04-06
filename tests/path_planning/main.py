import sys, time

import numpy as np
import matplotlib.pyplot as plt

sys.path.append('../../ros/src/playground/nodes/guidedog/')
import pathplanning

field_length = 3.40;
field_height = 2.33;
separation = 0.067;

def main():
    i=0
    for i in xrange(10):
        pathplanning.init_graph(field_length, field_height, distance_between_points=separation)

        if (i < 5):
            obstacles = [(1,.30), (1,-.30), (-.25, -.25)]
        else:
            obstacles = [(-1,-.30), (-1,.30), (.25, .25)]



        # How many points in x, y directions
        Nx = field_length/separation;
        Ny = field_height/separation;

        

        # x_max = (field_length/2);
        # y_max = (field_height/2);
        # x = np.linspace(-x_max, x_max, Nx);
        # y = np.linspace(-y_max, y_max, Ny);
        # X, Y = np.meshgrid(x, y);

        x = np.linspace(0, Nx-1, Nx)
        y = np.linspace(-(Ny-1), 0, Ny)
        X, Y = np.meshgrid(x, y)

        start = (-1.60,-1)
        end = (1.60,1)

        # Solve!
        start_time = time.time()

        for obstacle in obstacles:
            pathplanning.add_obstacle(obstacle)

        path = pathplanning.get_path(start, end)

        total_time = time.time() - start_time

        start = pathplanning._convert_location_to_node(start)
        end = pathplanning._convert_location_to_node(end)

        start = pathplanning._fix_node(start)
        end = pathplanning._fix_node(end)
        

        x = []
        y = []

        prev_node = start
        for node in path:
            # print("{} -> {}".format(pathplanning._convert_node_to_location(prev_node), pathplanning._convert_node_to_location(node)))  
            x.append(node[0]) 
            y.append(-node[1])
            prev_node = node

        plt.plot(x, y, color='r', linestyle='--')

        plt.scatter(X, Y)

        for obstacle in obstacles:
            temp = pathplanning._convert_location_to_node(obstacle)
            plt.scatter(temp[0], -temp[1], color='w', marker='x', s=20)

        print total_time, "seconds."

        # plt.grid()
    plt.show()
    print("{}, {} = {}".format(Nx, Ny, Nx * Ny))

if __name__ == "__main__":
    main()


# # Create a figure with the costs
# fig = plt.figure(1)
# ax = fig.add_subplot(111)

# ax.scatter(X, Y)

# # c_row = np.reshape(c,(1,-1)).tolist()[0]

# # import ipdb; ipdb.set_trace()

# for i, col in enumerate(c):
#     for j, cost in enumerate(col):
#         # print("({},{}) -> {}".format(i,j,cost))
#         ax.annotate(cost, (X[i][j],-(n-1)-Y[i][j]))


# # Create the path planning figure
# plt.figure(2)
# plt.scatter(X, Y)
