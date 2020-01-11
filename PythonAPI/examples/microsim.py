import glob
import math
import os
import sys
import random

sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

import carla
import svgwrite
import numpy as np

def normalize(angle):
    if angle >= np.deg2rad(180):
        return angle - np.deg2rad(360)
    elif angle < -180:
        return angle + np.deg2rad(360)
    else:
        return angle

if __name__ == '__main__':

    print('Loading resources...')
    network = carla.SumoNetwork.load('../../Data/meskel_square.net.xml')
    network_occupancy_map = network.create_occupancy_map()
    sidewalk = network_occupancy_map.create_sidewalk(1.5)
    print('Resources loaded.')
    
    path = [network.get_nearest_route_point(carla.Vector2D(465, 480))]
    for _ in range(500):
        path.append(random.choice(network.get_next_route_points(path[-1], 1.0)))

    simulator = carla.Simulator(network, sidewalk, 
            carla.VehicleAgent(network, 
                carla.Vector2D(-1, -1), carla.Vector2D(1, 3), 
                1.8, 6.0, 
                carla.Vector2D(465, 480), carla.Vector2D(0, -1),
                path))


    dwg = svgwrite.Drawing('map.svg', profile='full')
    dwg.add(dwg.rect(size=('100%', '100%'), fill='white'))

    def to_svg(coord):
        return [coord.x, coord.y]

    def add_arrowed_line(start, end, **args):
        direction = (end - start).make_unit_vector()
        normal = direction.rotate(math.pi / 2)
        mid = (start + end) / 2 
        dwg.add(dwg.line(
            to_svg(start), 
            to_svg(end), 
            **args))
        dwg.add(dwg.line(
            to_svg(mid - 0.75 * direction + 0.75 * normal),
            to_svg(mid + 0.75 * direction),
            **args))
        dwg.add(dwg.line(
            to_svg(mid - 0.75 * direction - 0.75 * normal),
            to_svg(mid + 0.75 * direction),
            **args))

    def add_line(start, end, **args):
        dwg.add(dwg.line(
            to_svg(start), 
            to_svg(end), 
            **args))

    def add_circle(pos, radius, **args):
        dwg.add(dwg.circle(
            to_svg(pos), 
            radius, 
            **args))

    def add_triangle(triangle, **args):
        dwg.add(dwg.polygon(
            [to_svg(triangle.v0), to_svg(triangle.v1), to_svg(triangle.v2)],
            **args))

    lanes_with_connections = set()
    lanes_with_connections.update(network.edges[c.from_edge].lanes[c.from_lane].id for c in network.connections)
    
    # Draw SUMO network.
    for entry in network.edges:
        edge = entry.data()
        for lane in edge.lanes:
            for i in range(len(lane.shape) - 1):
                add_arrowed_line(
                        lane.shape[i], 
                        lane.shape[i + 1],
                        stroke='grey',
                        stroke_width=0.25)

    # Draw sidewalk.
    for polygon in sidewalk.polygons:
        for i in range(len(polygon) - 1):
            add_line(
                    polygon[i], 
                    polygon[i + 1], 
                    stroke='red', 
                    stroke_width=0.25)
    
    for _ in range(300):
        add_circle(simulator.ego_agent.position, 0.5, stroke='blue', stroke_width=0.5)
        
        target = network.get_route_point_position(simulator.ego_agent.path[1])
        offset = target - simulator.ego_agent.position
        theta = normalize(math.atan2(offset.y, offset.x) - math.atan2(simulator.ego_agent.heading.y, simulator.ego_agent.heading.x))
        
        simulator = simulator.step(1, 6.0, 3.0 * theta)

    dwg.save()
