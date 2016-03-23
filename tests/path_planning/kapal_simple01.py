from kapal.algo import *
from kapal.world import *
from kapal.state import *
import kapal.tools
import time

import numpy as np
import matplotlib.pyplot as plt


# =====================================================================
# TODO: walk through example with comments
# TODO: post this example on wiki as a tutorial

n = 10      # width/height of world
c = kapal.tools.rand_cost_map(n, n, min_val=1, max_val=10, flip=False)
w = World2d(c, state_type=State2dAStar, diags=True)

astar = AStar(w, w.state(0,0), w.state(n-1, n-1))
start_time = time.time()
path = astar.plan()
total_time = time.time() - start_time

print total_time, "seconds."

# TODO: finish the example. show the output in a human-readable format.
#       perhaps possible interface with Seaship.
# =====================================================================


print c

x = np.linspace(0, n-1, n)
y = np.linspace(-(n-1), 0, n)
X, Y = np.meshgrid(x, y)

# Create a figure with the costs
fig = plt.figure(1)
ax = fig.add_subplot(111)

ax.scatter(X, Y)

# c_row = np.reshape(c,(1,-1)).tolist()[0]

# import ipdb; ipdb.set_trace()

for i, col in enumerate(c):
    for j, cost in enumerate(col):
        # print("({},{}) -> {}".format(i,j,cost))
        ax.annotate(cost, (X[i][j],-(n-1)-Y[i][j]))


# Create the path planning figure
plt.figure(2)
plt.scatter(X, Y)

# Build path to plot

# So that we can start at home
path.reverse()

x = []
y = []

current = (0, 0)

for state in path:
    if state:
        # Where I started
        x.append(state.x)
        y.append(-state.y)

        # if state.bp:
        #     # Where I ended
        #     x.append(state.bp.x)
        #     y.append(-state.bp.y)

plt.plot(x, y)

##############

path = astar.path()

x = []
y = []

current = (0, 0)

for state in path:
    if state:
        # Where I started
        x.append(state.x)
        y.append(-state.y)

        # if state.bp:
        #     # Where I ended
        #     x.append(state.bp.x)
        #     y.append(-state.bp.y)

plt.plot(x, y, color='r', linestyle='--')

plt.show()

# import ipdb; ipdb.set_trace()