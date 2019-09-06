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

if __name__ == '__main__':

    with open('../../Data/map.net.xml', 'r') as file:
        data = file.read()

    print('Loading map...')
    network = carla.SumoNetwork.load(data)
    network_occupancy_map = network.create_occupancy_map()
    sidewalk = network_occupancy_map.create_sidewalk(1.5)
    sidewalk_occupancy_map = sidewalk.create_occupancy_map(3)
    landmarks = carla.Landmark.load(
            '../../Data/map.osm',
            network.offset)
    landmarks = [l.difference(network_occupancy_map).difference(sidewalk_occupancy_map) for l in landmarks]
    landmarks = [l for l in landmarks if not l.is_empty]
    
    dwg = svgwrite.Drawing('map.svg', profile='full')
    dwg.add(dwg.rect(size=('100%', '100%'), fill='white'))

    def to_svg(coord):
        return [coord.y - network.bounds_min.y, network.bounds_max.x - coord.x]

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
                #if edge.function != carla.Function.Normal:
                #    stroke = 'blue'
                #if i == len(lane.shape) - 2 and lane.id not in lanes_with_connections:
                #    stroke = 'red'
                #    stroke_width = 1.0
                add_arrowed_line(
                        lane.shape[i], 
                        lane.shape[i + 1],
                        stroke=svgwrite.rgb(52,152,219),
                        stroke_width=1.0)

    # Draw sidewalk.
    for polygon in sidewalk.polygons:
        for i in range(len(polygon) - 1):
            add_line(
                    polygon[i], 
                    polygon[i + 1], 
                    stroke=svgwrite.rgb(231,76,60), 
                    stroke_width=1.0)

    # Draw landmarks.
    for landmark in landmarks:
        for triangle in landmark.get_triangles():
            add_triangle(
                    triangle,
                    fill=svgwrite.rgb(149,165,166),
                    stroke=svgwrite.rgb(149,165,166),
                    stroke_width=0.25)
        
    dwg.save()
