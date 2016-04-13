import astar as a
import copy

original_graph = None
modified_graph = None
field_length = 0
field_width = 0
distance_between_discrete_points = 0
edge_value = 0
diag_value = 0
obstacles = None


def init_graph(length_field, width_field, distance_between_points=.05, edge_distance=10, diag_distance=14):
	global original_graph, field_width, field_length, distance_between_discrete_points, edge_value, diag_value, obstacles

	field_length = length_field
	field_width = width_field
	distance_between_discrete_points = distance_between_points
	edge_value = edge_distance
	diag_value = diag_distance
	obstacles = list()

	original_graph = a.init_graph(length_field, width_field, distance_between_points, edge_distance, diag_distance)
	# modified_graph = copy.deepcopy(original_graph)

	# print (original_graph)
	# print (modified_graph)

def _convert_location_to_node(location):
	global distance_between_discrete_points, field_width, field_length

	x = int((location[0] + field_length/2.0)/distance_between_discrete_points)
	y = int((location[1] + field_width/2.0)/distance_between_discrete_points)

	return (x,y)

def _convert_node_to_location(node):
	global distance_between_discrete_points, field_width, field_length

	x = round((node[0]*distance_between_discrete_points) - field_length/2, 3)
	y = round((node[1]*distance_between_discrete_points) - field_width/2, 3)

	return (x,y)


def add_obstacle(location, radius=.2):
	global distance_between_discrete_points, field_length, field_width, original_graph

	discrete_radius = int(radius/distance_between_discrete_points)

	# print discrete_radius

	node = _convert_location_to_node(location)

	if node[0] > original_graph.n or node[0] < 0 or node[1] < 0 or node[1] > original_graph.m:
		print ("{} was not a valid point".format(node))
		return

	original_graph.add_obstacle(node, discrete_radius)
	obstacles.append((node, discrete_radius))

def _fix_node(node):
	global field_length, field_width, distance_between_discrete_points

	x_max = int(field_length/distance_between_discrete_points)
	y_max = int(field_width/distance_between_discrete_points)

	x = node[0]
	y = node[1]

	if (x < 0):
		x = 0;
	elif x > x_max:
		x = x_max

	if y < 0:
		y = 0
	elif y > y_max:
		y = y_max

	return (x,y)

def get_path(start, end):
	global original_graph, field_width, field_length, distance_between_discrete_points, edge_value, diag_value

	start = _convert_location_to_node(start)
	end = _convert_location_to_node(end)

	start = _fix_node(start)
	end = _fix_node(end)

	# print ("start = {}, end = {}".format(start, end))

	path = original_graph.path(start, end)

	for obstacle in obstacles:
		original_graph.remove_obstacle(obstacle[0],obstacle[1])

	world_path = map(_convert_node_to_location, path)

	return world_path



def main():
	init_graph(3.68, 2)

	add_obstacle(1,1)
	add_obstacle(-1,1)
	add_obstacle(1,-1)
	add_obstacle(-1,-1)
	add_obstacle(0,0)








if __name__ == "__main__":
    main()