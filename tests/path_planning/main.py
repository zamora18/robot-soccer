import sys, time

import numpy as np
import matplotlib.pyplot as plt

sys.path.append('../../ros/src/playground/nodes/guidedog/')
import astar

field_length = 0.100 #3.68;
field_height = 0.100 #1.52;
separation = 0.01 #0.200;

def main():

    graph = astar.init_graph(field_length, field_height, distance_between_points=separation)


    obstacles = [ (5,4) ]

    for obstacle in obstacles:
        graph.add_obstacle(obstacle, 3)

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

    start = (0,0)
    end = (9,9)

    # Solve!
    start_time = time.time()
    path = graph.path(start, end)
    total_time = time.time() - start_time

    print total_time, "seconds."

    x = []
    y = []

    prev_node = start
    for node in path:
        print("{} -> {}".format(prev_node, node))
        x.append(node[0]) 
        y.append(-node[1])
        prev_node = node

    plt.plot(x, y, color='r', linestyle='--')

    plt.scatter(X, Y)

    for obstacle in obstacles:
        plt.scatter(obstacle[0], -obstacle[1], color='w', marker='x', s=20)

    # plt.grid()
    plt.show()

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
