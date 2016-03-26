import astar as a


graph = None
field_length = 0
field_width = 0
distance_between_discrete_points = 0


def init_graph(length_field, width_field, distance_between_points=.01, edge_distance=10, diag_distance=14):
	global graph, field_width, field_length, distance_between_discrete_points

	field_length = length_field
	field_width = width_field
	distance_between_discrete_points = distance_between_points


	graph = a.init_graph(length_field, width_field, distance_between_points, edge_distance, diag_distance)




def add_obstacle(x, y, radius):
	discrete_radius = int(radius/distance_between_discrete_points)

