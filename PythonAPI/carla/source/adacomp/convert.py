import numpy as np
import lane_network
import lane_network_solver
import svgwrite
from util import *
import math
from collections import defaultdict

network = lane_network.LaneNetwork.from_osm('/map.osm')

max_x = max(node.pos[0] for node in network.nodes.values())
min_x = min(node.pos[0] for node in network.nodes.values())
max_y = max(node.pos[1] for node in network.nodes.values())
min_y = min(node.pos[1] for node in network.nodes.values())

def tsp(pos):
    return (pos[0] - min_x, max_y - pos[1])

dwg = svgwrite.Drawing(
        'map.svg', 
        size=(max_x - min_x, max_y - min_y),
        profile='full')
dwg.add(dwg.rect(
    insert=(0, 0), 
    size=('100%', '100%'), 
    rx=None, 
    ry=None, 
    fill='white'))

def add_line(start, end, **args):
    dwg.add(dwg.line(
        tsp(start),
        tsp(end),
        **args))

def add_arrowed_line(start, end, **args):
    direction = end - start
    direction /= np.linalg.norm(direction)
    normal = rotate(direction, math.pi / 2)
    mid = (start + end) / 2
    dwg.add(dwg.line(tsp(start), tsp(end), **args))
    dwg.add(dwg.line(
        tsp(mid - 0.2 * direction + 0.2 * normal),
        tsp(mid + 0.2 * direction),
        **args))
    dwg.add(dwg.line(
        tsp(mid - 0.2 * direction - 0.2 * normal),
        tsp(mid + 0.2 * direction),
        **args))

def add_circle(point, r, **args):
    dwg.add(dwg.circle(tsp(point), r, **args))


# Draw roads.
for road in network.roads.values():
    node_a = network.nodes[road.source_node_id]
    node_b = network.nodes[road.destination_node_id]
    add_arrowed_line(node_a.pos, node_b.pos, stroke='lightgray', stroke_width=0.5)



# Solve network.
lane_network_solver.solve(network)



# Draw lane connections.
for lane_connection in network.lane_connections.values():
    start = network.get_lane_end(
        network.lanes[lane_connection.source_lane_id], 
        lane_connection.source_offset)
    end = network.get_lane_start(
        network.lanes[lane_connection.destination_lane_id],
        lane_connection.destination_offset)
    if np.linalg.norm(end - start) > 0.1:
        add_arrowed_line(start, end, stroke='green', stroke_width=0.1)
    else:
        add_circle((start + end) / 2, 0.2, stroke='green', stroke_width=0.1)



# Draw lanes.
# Fast lookup table.
lane_connections_table = defaultdict(list)
for lane_connection in network.lane_connections.values():
    lane_connections_table[lane_connection.source_lane_id].append(lane_connection)
    lane_connections_table[lane_connection.destination_lane_id].append(lane_connection)

for lane in network.lanes.values():
    incoming_offset = None
    outgoing_offset = None
    for lane_connection in lane_connections_table[lane.id]:
        if lane_connection.source_lane_id == lane.id:
            if outgoing_offset == None or outgoing_offset > lane_connection.source_offset:
                outgoing_offset = lane_connection.source_offset
        if lane_connection.destination_lane_id == lane.id:
            if incoming_offset == None or incoming_offset > lane_connection.destination_offset:
                incoming_offset = lane_connection.destination_offset
    
    start = network.get_lane_start(
        lane,
        0 if incoming_offset == None else incoming_offset)
    end = network.get_lane_end(
        lane,
        0 if outgoing_offset == None else outgoing_offset)
    color = 'red' if incoming_offset == None or outgoing_offset == None else 'blue'
    
    if np.linalg.norm(end - start) > 0.1:
        add_arrowed_line(start, end, stroke=color, stroke_width=0.1)
    else:
        add_line(start, end, stroke=color, stroke_width=0.1)

print('Saving network...')
network.save('./network.ln')    
print('Loading network...')
lane_network.LaneNetwork.load('./network.ln')

dwg.save()
exit()
