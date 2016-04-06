import Constants as C
from GameObjects import Robot, Ball
from pathplanning import init_graph, add_obstacle, get_path


def initialize_world():
    init_graph(C.field_length, C.field_width, distance_between_points=0.067)


def plan(x_c, y_c, _me, _ally, _opp1, _opp2):
    # Add all obstacles every iteration
    add_obstacle(_ally.get_2d_location())
    add_obstacle(_opp1.get_2d_location())
    add_obstacle(_opp2.get_2d_location())

    # Create start and end points
    start = _me.get_2d_location()
    end = (x_c, y_c)

    # Plan that path! Returns a list of tuples
    path = get_path(start, end)

    return path[2]
