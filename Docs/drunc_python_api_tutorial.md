<h1>DRUNC Python API Tutorial</h1>
#### Overview

The DRUNC project extends CARLA with additional functionality to support research in autonomous driving in urban crowds. This document serves as a tutorial for DRUNC's Python API to access these features.

##### DRUNC Features in LibCarla/PythonAPI 

A few concepts are central to DRUNC, and are integrated as a part of LibCarla accessible through bindings in the Python API:

  - **Lane Network:** A representation of the topology of a driving network at the fidelity of lanes. Lane networks holds topological and geometrical information of the lanes and lane connections.
  - **Route Map:** An interface for generate routes, in particular functions to get nearest route point and to get next possible route points.
  - **Occupancy Map:** A representation for some covered area in 2D space, using a bounding volume hierarchy containing of mesh triangles.
  - **Occupancy Grid:**  A representation for some covered area in 2D space partitioned up to some resolution, using a 2D lookup table.
  - **Polygon Table:** A representation for polygons in 2D space partition up to some cell-size, using a 2D lookup table.
  - **GAMMA:** Motion prediction model for heterogeneous traffic agents. [Insert citation]

These features do not require a CARLA simulation to be running to work, since they are independent of any simulator instance.

##### Simulator/LibCarla/PythonAPI Extensions

Additionally, we have also added functionality to support our extensions:

- **Occupancy Map Spawning:** Spawns a given occupancy map as a mesh in a CARLA simulation.



#### Loading and spawning lane network

The typical first step is to load a lane network from an existing network file

```py
lane_network = carla.LaneNetwork.load("network.ln")
```

Next, an occupancy map can be created from the loaded lane network and spawned in a CARLA simulation

```py
occupancy_map = lane_network.create_occupancy_map()
client = carla.Client('127.0.0.1', 2000)
world = client.get_world()
world.spawn_occupancy_map(occupancy_map)
```



#### Creating route map and walking along random routes

A route map can be created from an existing lane network

```pyth
lane_network = carla.LaneNetwork.load("network.ln")

route_map = carla.RouteMap(lane_network)
```

Walkers can be spawn at random positions using the route map

```py
position = carla.Vector2D(random.uniform(-500, 500), random.uniform(-500, 500))
route_point = route_map.get_nearest_route_point(position)
position = route_map.get_position(route_point)
actor = world.spawn_actor(
    blueprint, 
    carla.Transform(carla.Location(position.x, position.y, 2.0), carla.Rotation()))
```

The route map can also be used to generate routes for walkers

```py
path = [route_point]
for _ in range(20):
    # Find all possible next route points 1.0 meters ahead.
    possible_next_route_points = route_map.get_next_route_points(route_point[-1], 1.0)
    # Randomly select next route point.
    next_route_point = random.choice(possible_next_route_points)
    path.append(next_route_point)
```



#### Fetching occupancy grid from occupancy map

Occupancy maps can be rasterized to produce an occupancy grid with some resolution bounded by some area

```Py
lane_network = carla.LaneNetwork.load("network.ln")
occupancy_map = lane_network.create_occupancy_map()

occupancy_grid = occupancy_map.create_occupancy_grid(
    carla.Vector2D(-500, -500), # Area lower bound.
    carla.Vector2D(500, 500), # Area upper bound
    0.1) # Resolution
```

The data of the occupancy grid is stored in an underlying  8-bit 2D array flattened in row-major order into a contiguous block. It can be retrieved directly as a NumPy array for use

```python
import cv2

...
cv2.imshow('test', occupancy_grid.data)
```

Note that the returned NumPy array points directly to the underlying memory of the occupancy grid. No copying of the data is done at all, making such usage extremely efficient. Changes to the NumPy array changes the original data directly.



#### Fetching polygon table from occupancy map

Polygon tables can be retrieved from occupancy map for the fast lookup of polygons making up the unoccupied areas

```py
lane_network = carla.LaneNetwork.load("network.ln")
occupancy_map = lane_network.create_occupancy_map()

polygon_table = occupancy_map.create_polygon_table(
    carla.Vector2D(-500, -500), # Area lower bound.
    carla.Vector2D(500, 500), # Area upper bound
    100, # Table cell size
    0.1) # Resolution
```

The table cell size argument determines the size of cells used to partition the occupancy map. On the other hand, the resolution is used for a different purpose - DRUNC calculates the polygon against a rasterization of the occupancy map, and the resolution argument is specified for this rasterization process.

The polygon table can then be used accordingly. For example, to draw out all polygons in every cell,

```py
img = np.zeroes((1000, 1000, 3), dtype=np.uint8)
for r in range(polygon_table.rows):
    for c in range(polygon_table.columns):
        for p in polygon_table.get(r, c): 
            for i in range(len(p) - 1): 
                v1 = p[i]
                v2 = p[i + 1]
                cv2.arrowedLine(img, 
                    (int((v1.y - (-500)) / 0.1), int((500 - v1.x) / 0.1)),
                    (int((v2.y - (-500)) / 0.1), int((500 - v2.x) / 0.1)),
                    (0, 0, 255),
                    1) 
```

