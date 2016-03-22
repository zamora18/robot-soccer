from collections import defaultdict
import numpy as np


class Graph:
    def __init__(self, n, m):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}
        self.n = n
        self.m = m

    def add_node(self, coord):
        value = self.convert_coord_to_node(coord)
        self.nodes.add(value)

    def add_edge(self, from_coord, to_coord, weight):
        from_node = self.convert_coord_to_node(from_coord)
        to_node = self.convert_coord_to_node(to_coord)
        if from_node == None or to_node == None:
            return
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = weight

    def convert_coord_to_node(self, coord):
        result = coord[1] + coord[0] * self.n
        if result < 0 or result >= self.n * self.m:
            return None
        return result

    def convert_node_to_coord(self, node):
        i = node / self.n
        j = node % self.n
        return (i,j)

    def add_edges(self,coord, weight, diag_weight):
        self.add_edge(coord, (coord[0]-1, coord[1]), weight)
        self.add_edge(coord, (coord[0]+1, coord[1]), weight)
        self.add_edge(coord, (coord[0], coord[1]-1), weight)
        self.add_edge(coord, (coord[0], coord[1]+1), weight)
        self.add_edge(coord, (coord[0]-1, coord[1]-1), diag_weight)
        self.add_edge(coord, (coord[0]-1, coord[1]+1), diag_weight)
        self.add_edge(coord, (coord[0]+1, coord[1]-1), diag_weight)
        self.add_edge(coord, (coord[0]+1, coord[1]+1), diag_weight)

    def get_node(self, coord):
        return self.convert_coord_to_node(coord)




def init_grid(length_field, width_field, distance_between_points=.01, edge_weight=1, diag_weight=1.4):
    """creates a grid with the above parameters for A*"""

    numofxpoints = int(np.floor(length_field/distance_between_points));
    numofypoints = int(np.floor(width_field/distance_between_points));

    graph = Graph(numofxpoints, numofypoints)

    for i in xrange(numofxpoints):
        for j in xrange(numofypoints):
            graph.add_node((i,j))

    for i in xrange(numofxpoints):
        for j in xrange(numofypoints):
            graph.add_edges((i,j),edge_weight, diag_weight)

    # for i in xrange(numofxpoints):
    #     for j in xrange(numofypoints):
    #         print (graph.get_node((i,j)))

    return graph
