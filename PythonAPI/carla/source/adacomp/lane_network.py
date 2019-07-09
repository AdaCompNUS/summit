import osm
import math
import numpy as np
import itertools
from util import *
import struct

class Node:
    def __init__(self, id, pos):
        self.id = id
        self.pos = pos

class Road:
    def __init__(self, id, source_node_id, destination_node_id, forward_lane_ids=None, backward_lane_ids=None):
        self.id = id
        self.source_node_id = source_node_id
        self.destination_node_id = destination_node_id
        self.forward_lane_ids = [] if forward_lane_ids is None else forward_lane_ids
        self.backward_lane_ids = [] if backward_lane_ids is None else backward_lane_ids

class Lane:
    def __init__(self, id, road_id, is_forward, index):
        self.id = id
        self.road_id = road_id
        self.is_forward = is_forward
        self.index = index

class LaneConnection:
    def __init__(self, id, source_lane_id, destination_lane_id, source_offset=0.0, destination_offset=0.0):
        self.id = id
        self.source_lane_id = source_lane_id
        self.destination_lane_id = destination_lane_id
        self.source_offset = source_offset
        self.destination_offset = destination_offset

class LaneNetwork:
    def __init__(self, lane_width=3.0):
        self.lane_width = lane_width
        self.nodes = dict()
        self.roads = dict()
        self.lanes = dict()
        self.lane_connections = dict()
    
    def get_road_length(self, road):
        return np.linalg.norm(self.nodes[road.destination_node_id].pos - self.nodes[road.source_node_id].pos)


    def get_road_direction(self, road):
        direction = self.nodes[road.destination_node_id].pos - self.nodes[road.source_node_id].pos
        return direction / np.linalg.norm(direction)
    
    def get_lane_direction(self, lane):
        direction = self.get_road_direction(self.roads[lane.road_id])
        if not lane.is_forward:
            direction = -direction
        return direction

    def get_lane_start(self, lane, offset=0):
        road = self.roads[lane.road_id]
        direction = self.get_road_direction(road)
        normal = rotate(direction, math.pi / 2)

        if lane.is_forward:
            center = self.nodes[road.source_node_id].pos + offset * direction
            index = lane.index + len(road.backward_lane_ids)
        else:
            center = self.nodes[road.destination_node_id].pos - offset * direction
            index = len(road.backward_lane_ids) - 1 - lane.index
        
        num_lanes = len(road.forward_lane_ids) + len(road.backward_lane_ids)

        return center + self.lane_width * (index - 0.5 * (num_lanes - 1)) * normal
    
    def get_lane_end(self, lane, offset=0):
        road = self.roads[lane.road_id]
        direction = self.get_road_direction(road)
        normal = rotate(direction, math.pi / 2)

        if lane.is_forward:
            center = self.nodes[road.destination_node_id].pos - offset * direction
            index = lane.index + len(road.backward_lane_ids)
        else:
            center = self.nodes[road.source_node_id].pos + offset * direction
            index = len(road.backward_lane_ids) - 1 - lane.index
        
        num_lanes = len(road.forward_lane_ids) + len(road.backward_lane_ids)

        return center + self.lane_width * (index - 0.5 * (num_lanes - 1)) * normal
        

    def save(self, path):
        with open(path, 'wb') as f:
            f.write(struct.pack('>d', self.lane_width))
            for node in sorted(self.nodes.values(), key=lambda x:x.id):
                f.write(struct.pack('>bqdd', 1, node.id, node.pos[0], node.pos[1]))
            for road in sorted(self.roads.values(), key=lambda x:x.id):
                f.write(struct.pack(
                    '>bqqq', 
                    2, road.id, road.source_node_id, road.destination_node_id))
                f.write(struct.pack('>i', len(road.forward_lane_ids)))
                for forward_lane_id in road.forward_lane_ids:
                    f.write(struct.pack('>q', forward_lane_id))
                f.write(struct.pack('>i', len(road.backward_Lane_ids)))
                for backward_lane_id in road.backward_lane_ids:
                    f.write(struct.pack('>q', backward_lane_id))
            for lane in sorted(self.lanes.values(), key=lambda x:x.id):
                f.write(struct.pack('>bqq?i', 3, lane.id, lane.road_id, lane.is_forward, lane.index))
            for lane_connection in sorted(self.lane_connections.values(), key=lambda x:x.id):
                f.write(struct.pack(
                    '>bqqqdd', 
                    4, 
                    lane_connection.id,
                    lane_connection.source_lane_id,
                    lane_connection.destination_lane_id,
                    lane_connection.source_offset,
                    lane_connection.destination_offset))

    @staticmethod
    def load(path):
        with open(path, 'rb') as f:
            lane_width = struct.unpack('>d', f.read(8))[0]
            network = LaneNetwork(lane_width)
            
            element_type = f.read(1)
            while element_type:
                element_type = struct.unpack('>b', element_type)[0]

                if element_type == 1:
                    data = struct.unpack('>qdd', f.read(8 + 8 + 8))
                    node = Node(data[0], np.array([data[1], data[2]]))
                    network.nodes[node.id] = node
                elif element_type == 2:
                    data = struct.unpack('>qqq', f.read(8 + 8 + 8))
                    num_forward_lanes = struct.unpack('>i', f.read(4))[0]
                    forward_lane_ids = struct.unpack(
                            '>' + 'q' * num_forward_lanes, 
                            f.read(8 * num_forward_lanes))
                    num_backward_lanes = struct.unpack('>i', f.read(4))[0]
                    backward_lane_ids = struct.unpack(
                            '>' + 'q' * num_backward_lanes,
                            f.read(8 * num_backward_lanes))
                    road = Road(data[0], data[1], data[2], forward_lane_ids, backward_lane_ids)
                    network.roads[road.id] = road
                elif element_type == 3:
                    data = struct.unpack('>qq?i', f.read(8 + 8 + 1 + 4))
                    lane = Lane(*data)
                    network.lanes[lane.id] = lane
                elif element_type == 4:
                    data = struct.unpack('qqqdd', f.read(8 + 8 + 8 + 8 + 8))
                    lane_connection = LaneConnection(data[0], data[1], data[2])
                    lane_connection.source_offset = data[3]
                    lane_connection.destination_offset = data[4]
                    network.lane_connections[lane_connection.id] = lane_connection

                element_type = f.read(1)

        return network

    @staticmethod
    def from_osm(osm_path, lane_width=3.0):

        # Load osm map.
        osm_map = osm.Map('map.osm')
        
        # Determine osm roads for use.
        osm_road_ids = set()
        osm_node_ids = set()
        for osm_way in osm_map.ways.values():
            if 'highway' in osm_way.tags and osm_way.tags['highway'] != 'footway':
                osm_road_ids.add(osm_way.id)
                osm_node_ids.update(osm_way.nodes)

        # Calculate nodes.
        nodes = []
        for osm_node_id in osm_node_ids:
            osm_node = osm_map.nodes[osm_node_id]
            pos = np.array(osm_map.loc_to_cart(osm_node.lat, osm_node.lon))
            nodes.append(Node(osm_node_id, pos))

        # Calculate roads.
        roads = []
        road_id = 0
        lanes = []
        lane_id = 0
        for osm_road_id in osm_road_ids:
            # Calculate number of lanes per direction.
            osm_road = osm_map.ways[osm_road_id]
            num_lanes = 1
            if 'lanes' in osm_road.tags:
                num_lanes = int(osm_road.tags['lanes'])
            num_forward_lanes = num_lanes
            num_backward_lanes = 0
            if 'lanes:forward' in osm_road.tags:
                num_forward_lanes = int(osm_road.tags['lanes:forward'])
            num_backward_lanes = num_lanes - num_forward_lanes
            if 'lanes:backward' in osm_road.tags:
                if num_backward_lanes != int(osm_road.tags['lanes:backward']):
                    print('Warning: Lanes do not add up.')

            # Create roads between each node.
            for i in range(len(osm_road.nodes) - 1):
                node_a_id = osm_road.nodes[i]
                node_b_id = osm_road.nodes[i + 1]

                road = Road(road_id, node_a_id, node_b_id)
                road_id += 1
            
                # Create lanes along each direction on road.
                for j in range(num_forward_lanes):
                    lane = Lane(lane_id, road.id, True, j)
                    lane_id += 1
                    road.forward_lane_ids.append(lane.id)
                    lanes.append(lane)
                
                for j in range(num_backward_lanes):
                    lane = Lane(lane_id, road.id, False, j)
                    lane_id += 1
                    road.backward_lane_ids.append(lane.id)
                    lanes.append(lane)

                roads.append(road)
        
        # Create lane network.
        network = LaneNetwork(lane_width)
        network.nodes = dict((node.id, node) for node in nodes)
        network.roads = dict((road.id, road) for road in roads)
        network.lanes = dict((lane.id, lane) for lane in lanes)

        return network
