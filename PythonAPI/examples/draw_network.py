import glob
import math
import os
import sys

sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

import carla

import svgwrite

def get_spawn_occupancy_map(center_pos, spawn_size_min, spawn_size_max):
    return carla.OccupancyMap(
            carla.Vector2D(center_pos.x - spawn_size_max, center_pos.y - spawn_size_max),
            carla.Vector2D(center_pos.x + spawn_size_max, center_pos.y + spawn_size_max)) \
        .difference(carla.OccupancyMap(
            carla.Vector2D(center_pos.x - spawn_size_min, center_pos.y - spawn_size_min),
            carla.Vector2D(center_pos.x + spawn_size_min, center_pos.y + spawn_size_min)))

if __name__ == '__main__':

    print('Loading map...')
    network = carla.SumoNetwork.load('../../Data/meskel_square.net.xml')
    network_occupancy_map = network.create_occupancy_map()
    sidewalk = network_occupancy_map.create_sidewalk(1.5)
    sidewalk_occupancy_map = sidewalk.create_occupancy_map(3)
    landmarks = carla.Landmark.load(
            '../../Data/meskel_square.osm',
            network.offset)
    landmarks = [l.difference(network_occupancy_map).difference(sidewalk_occupancy_map) for l in landmarks]
    landmarks = [l for l in landmarks if not l.is_empty]

    segment_map = network.create_segment_map()
    segment_map = segment_map.intersection(get_spawn_occupancy_map(carla.Vector2D(450, 400), 50, 150))

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
                stroke = 'black'
                stroke_width = 0.25
                '''
                if edge.function != carla.Function.Normal:
                    stroke = 'blue'
                    stroke_width = 0.25
                if i == len(lane.shape) - 2 and lane.id not in lanes_with_connections:
                    stroke = 'red'
                    stroke_width = 1.0
                '''
                add_arrowed_line(
                        lane.shape[i], 
                        lane.shape[i + 1],
                        stroke=stroke,
                        stroke_width=stroke_width)

    # Draw sidewalk.
    '''
    for polygon in sidewalk.polygons:
        for i in range(len(polygon) - 1):
            add_line(
                    polygon[i], 
                    polygon[i + 1], 
                    stroke=svgwrite.rgb(231,76,60), 
                    stroke_width=1.0)
    '''

    # Draw landmarks.
    '''
    for landmark in landmarks:
        for triangle in landmark.get_triangles():
            add_triangle(
                    triangle,
                    fill=svgwrite.rgb(149,165,166),
                    stroke=svgwrite.rgb(149,165,166),
                    stroke_width=0.25)
    '''
        
    # Sample segments.
    for _ in range(1000):
        add_circle(segment_map.rand_point(), 1.0, 
                fill='red',
                stroke='black',
                stroke_width=0.5)

    dwg.save()
