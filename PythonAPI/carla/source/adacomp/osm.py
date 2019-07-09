import osmium
import math

class Node:
    def __init__(self, n_osm):
        self.id = n_osm.id
        self.lat = n_osm.location.lat
        self.lon = n_osm.location.lon
        self.tags = dict((p.k, p.v) for p in n_osm.tags)

    def __str__(self):
        return str(self.lat) + ',' + \
               str(self.lon) + '/' + \
               ','.join(k + '=' + v for (k, v) in self.tags.items())

class Way:
    def __init__(self, w_osm):
        self.id = w_osm.id
        self.nodes = [n.ref for n in w_osm.nodes]
        self.tags = dict((p.k, p.v) for p in w_osm.tags)

    def __str__(self):
        return ','.join(str(n) for n in self.nodes) + '/' + \
               ','.join(k + '=' + v for (k, v) in self.tags.items())

class RelationMember:
    def __init__(self, rm_osm):
        self.id = rm_osm.ref
        self.role = rm_osm.role
        self.type = rm_osm.type

    def __str__(self):
        return '(' + str(self.id) + ',' + self.role + ',' + self.type + ')'

class Relation:
    def __init__(self, r_osm):
        self.id = r_osm.id
        self.members = [RelationMember(m) for m in r_osm.members]
        self.tags = dict((p.k, p.v) for p in r_osm.tags)

    def __str__(self):
        return ','.join(str(m) for m in self.members) + '/' + \
               ','.join(k + '=' + v for (k, v) in self.tags.items())

class OsmiumHandler(osmium.SimpleHandler):
    def __init__(self):
        osmium.SimpleHandler.__init__(self)
        self.nodes = dict()
        self.ways = dict()
        self.relations = dict()

    def node(self, n):
        self.nodes[n.id] = Node(n)

    def way(self, w):
        self.ways[w.id] = Way(w)

    def relation(self, r):
        self.relations[r.id] = Relation(r)

    def result(self):
        return (self.nodes, self.ways, self.relations)

class Map:
    def __init__(self, path):
        self.earth_circumference = 40075036.0
        self.lat_lon_scale = self.earth_circumference / 360.0 # meters per degree
        
        h = OsmiumHandler()
        h.apply_file(path)
        (self.nodes, self.ways, self.relations) = h.result()

        self.center_lat = 0
        self.center_lon = 0

        for n in self.nodes.values():
            self.center_lat += n.lat
            self.center_lon += n.lon
        self.center_lat /= len(self.nodes)
        self.center_lon /= len(self.nodes)

    def lat_to_m(self, lat):
        return lat * self.lat_lon_scale

    def lon_to_m(self, lon, lat):
        return lon * self.lat_lon_scale * math.cos(math.radians(lat))

    def loc_to_cart(self, lat, lon):
        return (self.lon_to_m(lon, lat) - self.lon_to_m(self.center_lon, lat),
                self.lat_to_m(lat) - self.lat_to_m(self.center_lat))
