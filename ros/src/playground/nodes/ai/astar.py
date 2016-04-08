from collections import defaultdict
import numpy as np

inf = 1e100
closest_h_node = None
original_graph = None

class Graph:
    def __init__(self, n, m, edge_value=10, diag_value=14):
        """inits the graph so you can setup the grid"""

        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}
        self.n = n
        self.m = m
        self.edge_distance = edge_value
        self.diag_distance = diag_value



    def get_adjacent_nodes(self, node):
        return self.edges[node]

    def add_node(self, coord):
        """adds a node at the specified coord"""

        value = self.convert_coord_to_node(coord)
        self.nodes.add(value)

    def add_edge(self, from_coord, to_coord, distance):
        """adds the specified edge"""

        if to_coord[0] < 0 or to_coord[0] > self.n or to_coord[1] < 0 or to_coord[1] > self.m:
            return

        from_node = self.convert_coord_to_node(from_coord)
        to_node = self.convert_coord_to_node(to_coord)
        if from_node == None or to_node == None:
            return
        self.edges[from_node].append(to_node)
        # only add from_node to to_node, or else you get doubles of everything
        # self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = distance

    def convert_coord_to_node(self, coord):
        """converts a coord to a nodes value"""

        result = coord[0] + coord[1] * self.n
        if result < 0 or result >= self.n * self.m:
            return None
        return result

    def convert_node_to_coord(self, node):
        """coverts a nodes value to the coord system"""

        j = node / self.n
        i = node % self.n
        return (i,j)

    def add_edges(self,coord, distance, diag_distance):
        """adds all 8 edges in graph to current node at distance for straight edges
           and diag_disatnce for diags"""

        self.add_edge(coord, (coord[0]-1, coord[1]), distance)
        self.add_edge(coord, (coord[0]+1, coord[1]), distance)
        self.add_edge(coord, (coord[0], coord[1]-1), distance)
        self.add_edge(coord, (coord[0], coord[1]+1), distance)
        self.add_edge(coord, (coord[0]-1, coord[1]-1), diag_distance)
        self.add_edge(coord, (coord[0]-1, coord[1]+1), diag_distance)
        self.add_edge(coord, (coord[0]+1, coord[1]-1), diag_distance)
        self.add_edge(coord, (coord[0]+1, coord[1]+1), diag_distance)

    def get_node(self, coord):
        return self.convert_coord_to_node(coord)


    def _find_path(self, closedict, end):
        """starts and end and goes back to the start node to find the best path"""


        path = list()
        currentkey = end
        current = closedict[currentkey]
        while current[1] != None:
            path.insert(0,currentkey)

            currentkey = current[1]
            current = closedict[currentkey]

        path.insert(0,currentkey)

        return path

    def _path(self, current, end, g, opendict, closedict, func=None):
        """recursive function to find the quickest way to the end node.
           Not to be called by the user"""

        global closest_h_node

        # add the current node to nodes that have been visited, and remove it from the open considered nodes
        closedict[current] = opendict[current];
        del opendict[current]

        # print self.convert_node_to_coord(current)

        if current == end:
            # print ('found end')
            return  self._find_path(closedict, end)

        adjacent = self.get_adjacent_nodes(current)

        lowest_f_node = None
        for n in adjacent:
            # if we've already been there dont consider it
            if (closedict.get(n) == None):
                node = opendict.get(n)
                distance = self.distances[(current, n)]
                if distance != inf:
                    h = self.h(n,end)
                    if closest_h_node == None or h < closest_h_node[1]:
                        # if closest_h_node == None:
                        #     print None
                        closest_h_node = (n,h)
                        # print closest_h_node

                    f = (distance + g) + h
                    # if the node already exists in open
                    if node != None:
                        # check to see if the path through this current node is better then 
                        # the other node it was calculated from
                        if (distance + g) < node[0]:
                            # update the node in the open list
                            node = ((distance + g), current)
                            opendict[n] = node
                    else:
                        # add the node to teh dictionary
                        opendict[n] = ((distance + g), current)

                    # searches for the lowest f value among the adjacent nodes considered
                    if lowest_f_node == None or f < lowest_f_node[0]:
                        lowest_f_node = (f, n, distance)

        # if all nodes were already visited, im trapped so just leave
        if (lowest_f_node == None):
            print 'blocked off'
            print closest_h_node
            return self._find_path(closedict, closest_h_node[0])

        # update g for the next call
        g = g + lowest_f_node[2]

        # recursion!!!!
        return self._path(lowest_f_node[1], end, g, opendict, closedict)

    def path(self, start, end, func=None):


        global closest_h_node


        currentpath = list()
        start_node = self.convert_coord_to_node(start)
        currentpath.append(start_node)

        opendict = {start_node : (0, None)}
        closedict = {}

        # RECURSIVE PATHFINDING AHHHHHH
        result = self._path(start_node, self.convert_coord_to_node(end), 0, opendict, closedict)

        # change everything back to coordinates
        if result != None:
            result = map(self.convert_node_to_coord, result)

        return result

    def h(self, current, end):
        """right now just calculates the euclidean heuristic"""

        # calculate the absolute distance to where we want to go
        start_point = self.convert_node_to_coord(current)
        end_point = self.convert_node_to_coord(end)
        return np.sqrt((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2)

    
    def _add_obstacle(self, current, discrete_radius):
        if discrete_radius < 0:
            return
        adjacent = self.get_adjacent_nodes(current)
        for n in adjacent:
            self.distances[(current, n)] = inf
            self.distances[(n, current)] = inf
            self._add_obstacle(n, discrete_radius-1)





    def add_obstacle(self, center, discrete_radius):

        current = self.convert_coord_to_node(center)
        self._add_obstacle(current, discrete_radius)
        


    def _remove_obstacle(self, current, discrete_radius):
        if discrete_radius < 0:
            return
        adjacent = self.get_adjacent_nodes(current)
        for n in adjacent:
            if n/self.n != current/self.n and n%self.n != current%self.n:
                self.distances[(current, n)] = self.diag_distance
                self.distances[(n, current)] = self.diag_distance
            else:
                self.distances[(current, n)] = self.edge_distance
                self.distances[(n, current)] = self.edge_distance
            self._remove_obstacle(n, discrete_radius-1)

    def remove_obstacle(self, center, discrete_radius):

        current = self.convert_coord_to_node(center)
        self._remove_obstacle(current, discrete_radius)





def init_graph(length_field, width_field, distance_between_points=.01, edge_distance=10, diag_distance=14):
    """creates a grid with the above parameters for A*"""

    numofxpoints = int(np.floor(length_field/distance_between_points));
    numofypoints = int(np.floor(width_field/distance_between_points));

    graph = Graph(numofxpoints, numofypoints, edge_value=edge_distance, diag_value=diag_distance)

    for i in xrange(numofxpoints):
        for j in xrange(numofypoints):
            graph.add_node((i,j))

    for i in xrange(numofxpoints):
        for j in xrange(numofypoints):
            graph.add_edges((i,j),edge_distance, diag_distance)

    # for i in xrange(numofxpoints):
    #     for j in xrange(numofypoints):
    #         print (graph.get_node((i,j)))

    # print graph.path((40,3), (99,99))

    return graph