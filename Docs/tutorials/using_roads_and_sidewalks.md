# Spawning on roads
On roads, route points, stored as `carla.SumoNetworkRoutePoint` objects, hold semantic information such the point's road, lane, and offset along lane. The SUMO network, stored as a `carla.SumoNetwork`, is used to traverse these route points spatially and topologically.

One way to spawn points is to first lookup the route point on the road nearest to an arbitrary position.

```python
# Load SUMO network.
sumo_network = carla.SumoNetwork.load(PATH_TO_SUMO_NETWORK_FILE)

# Get arbitrary position.
position = carla.Vector(100, 100)

# Get nearest route point on SUMO network.
route_point = sumo_network.get_nearest_route_point(position)
```

Since the route point holds only semantic information and not the actual position, a translational query is required to determine the actual position to spawn the agent:

```python
# Get route point position.
route_point_position = sumo_network.get_route_point_position(route_point)

# Get route point transform
route_point_transform = carla.Transform()
route_point_transform.x = route_point_position.x
route_point_transform.y = route_point_position.y
route_point_transform.z = 0.5 # Spawn at a height of 0.5 meters.

# Spawn actor. See CARLA's documentation.
world.spawn_actor(BLUEPRINT, route_point_transform)
```

Alternatively, you can use the `carla.SegmentMap` class to generate route points uniformly distributed along the lanes of the road. The `carla.SegmentMap` class is a data structure to work with line segments, optimized for uniform sampling of points over stored line segments. 

```python
# Get segments of SUMO network.
sumo_network_segments = sumo_network.create_segment_map()

# Randmly pick spawn point. Note that this returns a position directly, not a route point.
route_point_position = sumo_network_segments.rand_point()
```

It is also possible to bound the spawn segments to a certain region, for example a rectangular bounding box:

```python
# Get segments of SUMO network.
sumo_network_segments = sumo_network.create_segment_map()

# Define bounding box.
bounds_min = carla.Vector2D(-50, -50)
bounds_max = carla.Vector2D(100, 300)
bounds_occupancy = carla.OccupancyMap(bounds_min, bounds_max)

# Calculate intersection of line segments with bounding box.
spawn_segments = sumo_network_segments.intersection(bounds_occupancy)
```

!!! note
    In SUMMIT, we strive to a support a wide range of geometric manipulations. As such, you can go all funky and do stuff like:

        # Arbitrary polygon.
        polygon = carla.OccupancyMap([carla.Vector2D(-50, -50),
            carla.Vector2D(-50, 100), carla.Vector2D(30, 200),
            carla.Vector2D(70, 70), carla.Vector2D(100, -70)])
        # Arbitrary rectangle.
        rectangle1 = carla.OccupancyMap(carla.Vector2D(-30, -30), carla.Vector2D(200, 300))
        # Another arbitrary rectangle.
        rectangle2 = carla.OccupancyMap(carla.Vector2D(-70, 30), carla.Vector2D(100, 200))
        # Some combination of areas.
        spawn_occupancy = polygon.union(rectangle1).difference(rectangle2)
        # Crop line segments.
        spawn_segments = sumo_network_segments.intersection(spawn_occupancy)

# Navigation on roads
On roads, the SUMO network can be used to traverse the route points spatially and topologically.

The below example fetches the nearest route point given the agent's current position, and randomly selects from topologically possible next route points a set distance ahead.

```python3
# Get 2D position of actor.
location = actor.get_location()
position2d = carla.Vector2D(location.x, location.y)

# Lookup nearest route point.
route_point = sumo_network.get_nearest_route_point(position2d)

# Get all route points 1 meter ahead.
next_route_points = sumo_network.get_next_route_points(route_point, 1.0)

# Randomly select next route point.
next_route_point = random.choice(next_route_points)
```

Since the route point only holds semantic information and not the actual position, a translational query is required to determine the actual position:

```python
# Get next route point's position.
position = sumo_network.get_route_point_position(next_route_point)
```

which can then be used for required tasks, such as for feedback to some steering controller for vehicles.

# Spawning on sidewalks
On sidewalks, things work very similar to roads. Sidewalks also have route points, stored as `carla.SidewalkRoutePoint` objects, which hold semantic information such as the point's polygon, segment, and offset along segment.  The sidewalk, stored as a `carla.Sidewalk`, is used to traverse these route points spatially and topologically.

One way to spawn points is to first lookup the route point on the road nearest to an arbitrary position.

```python
# Load SUMO network.
sumo_network = carla.SumoNetwork.load(PATH_TO_SUMO_NETWORK_FILE)

# Calculate sidewalk 1.5 meters from road's mesh.
sumo_network_occupancy = sumo_network.create_occupancy_map()
sidewalk = sumo_network_occupancy.create_sidewalk(1.5)

# Get arbitrary position.
position = carla.Vector(100, 100)

# Get nearest route point on sidewalk.
route_point = sidewalk.get_nearest_route_point(position)
```

Since the route point holds only semantic information and not the actual position, a translational query is required to determine the actual position to spawn the agent:

```python
# Get route point position.
route_point_position = sidewalk.get_route_point_position(route_point)

# Get route point transform
route_point_transform = carla.Transform()
route_point_transform.x = route_point_position.x
route_point_transform.y = route_point_position.y
route_point_transform.z = 0.5 # Spawn at a height of 0.5 meters.

# Spawn actor. See CARLA's documentation.
world.spawn_actor(BLUEPRINT, route_point_transform)
```

Alternatively, you can use the `carla.SegmentMap` class to generate route points uniformly distributed along the segments of the sidewalk. The `carla.SegmentMap` class is a data structure to work with line segments, optimized for uniform sampling of points over stored line segments. 

```python
# Get segments of sidewalk.
sidewalk_segments = sidewalk.create_segment_map()

# Randmly pick spawn point. Note that this returns a position directly, not a route point.
route_point_position = sidewalk_segments.rand_point()
```

It is also possible to bound the spawn segments to a certain region, for example a rectangular bounding box:

```python
# Get segments of sidewalk.
sidewalk_segments = sidewalk.create_segment_map()

# Define bounding box.
bounds_min = carla.Vector2D(-50, -50)
bounds_max = carla.Vector2D(100, 300)
bounds_occupancy = carla.OccupancyMap(bounds_min, bounds_max)

# Calculate intersection of line segments with bounding box.
spawn_segments = sidewalk_segments.intersection(bounds_occupancy)
```

!!! note
    In SUMMIT, we strive to a support a wide range of geometric manipulations. As such, you can go all funky and do stuff like:

        # Arbitrary polygon.
        polygon = carla.OccupancyMap([carla.Vector2D(-50, -50),
            carla.Vector2D(-50, 100), carla.Vector2D(30, 200),
            carla.Vector2D(70, 70), carla.Vector2D(100, -70)])
        # Arbitrary rectangle.
        rectangle1 = carla.OccupancyMap(carla.Vector2D(-30, -30), carla.Vector2D(200, 300))
        # Another arbitrary rectangle.
        rectangle2 = carla.OccupancyMap(carla.Vector2D(-70, 30), carla.Vector2D(100, 200))
        # Some combination of areas.
        spawn_occupancy = polygon.union(rectangle1).difference(rectangle2)
        # Crop line segments.
        spawn_segments = sidewalk_segments.intersection(spawn_occupancy)

# Navigating sidewalks
On sidewalks, the sidewalk can be used to traverse the route points spatially and topologically.

The below example fetches the nearest route point given the agent's current position, and selects the next route point a set distance anticlockwise along the route point's polygon. 

!!! note
    Sidewalks are structured using closed polygons, so there will always be exactly one next route point. Additionally, sidewalks in SUMMIT are always oriented anticlockwise, so that the next route point always goes anticlockwise along the current route point's polygon. 

```python
# Get 2D position of actor.
location = actor.get_location()
position2d = carla.Vector2D(location.x, location.y)

# Lookup nearest route point.
route_point = sidewalk.get_nearest_route_point(position2d)

# Get route point 1 meter anticlockwise along route point's polygon.
next_route_point = sidewalk.get_next_route_point(route_point, 1.0)
```

To get the next route point clockwise (instead of anticlockwise) along the current route point's polygon, use the `get_previous_route_point` method instead:

```python
# Get route point 1 meter clockwise along route point's polygon.
next_route_point = sidewalk.get_previous_route_point(route_point, 1.0)
```

You can also fetch the nearest point on an adjacent sidewalk polygon (i.e. on the other side of the road):

```python
# Get nearest adjacent route point, at most 50 meters away.
# If such a route point exists, this returns a list of exactly one item.
# If not, this returns an empty list.
next_route_points = sidewalk.get_adjacent_route_points(route_point, 50.0)
```

Since the route point only holds semantic information and not the actual position, a translational query is required to determine the actual position:

```python
# Get next route point's position.
position = sidewalk.get_route_point_position(next_route_point)
```

which can then be used for required tasks, such as for feedback to some heading controller for pedestrians.
