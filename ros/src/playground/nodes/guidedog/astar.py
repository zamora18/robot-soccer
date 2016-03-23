from collections import defaultdict
import numpy as np


class Graph:
    def __init__(self, n, m):
        """inits the graph so you can setup the grid"""

        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}
        self.n = n
        self.m = m



    def get_adjacent_edges(self, node):
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

        result = coord[1] + coord[0] * self.n
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


    def find_f(self, coord, end, g):
        pass


    def _find_path(self, closedict, end):
        path = list()
        currentkey = end
        current = closedict[currentkey]
        while current[1] != None:
            path.insert(0,currentkey)

            currentkey = current[1]
            current = closedict[currentkey]
        return path

    def _path(self, current, end, g, opendict, closedict, func=None):

        closedict[current] = opendict[current];
        del opendict[current]

        if current == end:
            return  self._find_path(closedict, end)

        adjacent = self.get_adjacent_edges(current)

        lowest_f_node = None
        for n in adjacent:
            if (closedict.get(n) == None):
                node = opendict.get(n)
                distance = self.distances[(current, n)]
                f = (distance + g) + self.h(n, end)
                if node != None:
                    if (distance + g) < node[0]:
                        node = ((distance + g), current)
                        opendict[n] = node
                else:
                    opendict[n] = ((distance + g), current)


                if lowest_f_node == None or f < lowest_f_node[0]:
                    lowest_f_node = (f, n, distance)

        if (lowest_f_node == None):
            return None

        g = g + lowest_f_node[2]

        return self._path(lowest_f_node[1], end, g, opendict, closedict)

    def path(self, start, end, func=None):

        currentpath = list()
        start_node = self.convert_coord_to_node(start)
        currentpath.append(start_node)

        opendict = {start_node : (0, None)}
        closedict = {}

        # RECURSIVE PATHFINDING AHHHHHH
        result = self._path(start_node, self.convert_coord_to_node(end), 0, opendict, closedict)

        # change everything back to coordinates
        result = map(self.convert_node_to_coord, result)

        return result

    def h(self, current, end):
        """right now just calculates the euclidean heuristic"""

        start_point = self.convert_node_to_coord(current)
        end_point = self.convert_node_to_coord(end)
        return np.sqrt((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2)





def init_graph(length_field, width_field, distance_between_points=.01, edge_distance=10, diag_distance=14):
    """creates a grid with the above parameters for A*"""

    numofxpoints = int(np.floor(length_field/distance_between_points));
    numofypoints = int(np.floor(width_field/distance_between_points));

    graph = Graph(numofxpoints, numofypoints)

    for i in xrange(numofxpoints):
        for j in xrange(numofypoints):
            graph.add_node((i,j))

    for i in xrange(numofxpoints):
        for j in xrange(numofypoints):
            graph.add_edges((i,j),edge_distance, diag_distance)

    # for i in xrange(numofxpoints):
    #     for j in xrange(numofypoints):
    #         print (graph.get_node((i,j)))

    print graph.path((0,0), (9,9))

    return graph