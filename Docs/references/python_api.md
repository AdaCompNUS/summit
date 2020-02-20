!!! important
    SUMMIT's Python API is an extension of CARLA's [Python API](https://carla.readthedocs.io/en/latest/python_api/). All classes and methods appearing in CARLA's Python API are accessible in SUMMIT. 
    
    However, since some components were designed for different use cases, not all components from CARLA may work with SUMMIT components, and vice versa.

## carla.Actor<a name="carla.Actor"></a>
Only additional features introduced in SUMMIT to `carla.Actor` are listed here. Please refer to [CARLA's implementation](https://carla.readthedocs.io/en/latest/python_api/#carlaactor) for the remaining original features, all of which are also accessible.

<h3>Methods</h3>

- <a name="carla.actor.set_collision_enabled"></a>**<font color="#7fb800">set_collision_enabled</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**enabled**</font>)   
Enable or disable collision for this instance.
    - **Parameters:**
        - `enabled`: _bool_

---

## carla.AABB2D<a name="carla.AABB2D"></a>
2D axis aligned bounding box helper class.

<h3>Instance Variables</h3>

- <a name="carla.AABB2D.bounds_min"></a>**<font color="#f8805a">bounds_min</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)

- <a name="carla.AABB2D.bounds_max"></a>**<font color="#f8805a">bounds_max</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)

<h3>Methods</h3>

- <a name="carla.AABB2D.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)
    - **Parameters:**
        - `other` (_[carla.AABB2D](#carla.AABB2D)_)

- <a name="carla.AABB2D.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)
    - **Parameters:**
        - `other` (_[carla.AABB2D](#carla.AABB2D)_)

---

## carla.AABBMap<a name="carla.AABBMap"></a>
Data structure optimized for efficient collision checking of 2D axis aligned bounding boxes (AABBs).

<h3>Methods</h3>

- <a name="carla.AABBMap.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>)  
Constructs an empty instance.

- <a name="carla.AABBMap.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
Returns the number of AABBs contained in this instance.

- <a name="carla.AABBMap.insert"></a>**<font color="#7fb800">insert</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**aabb**</font>)  
Inserts an AABB into this instance.
    - **Parameters:**
        - `aabb` (_[carla.AABB2D](#carla.AABB2D)_) 

- <a name="carla.AABBMap.intersects"></a>**<font color="#7fb800">intersects</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**aabb**</font>)  
Checks if an AABB intersects with any contained in this instance.
    - **Parameters:**
        - `aabb` (_[carla.AABB2D](#carla.AABB2D)_) 
    - **Return:** _bool_

---

## carla.Landmark<a name="carla.Landmark"></a>
Helper class to load landmark meshes from OSM files.

- <a name="carla.Landmark.load"></a>**<font color="#7fb800">load</font>**(<font color="#00a6ed">**filename**</font>, <font color="#00a6ed">**offset**=carla.Vector2D(0, 0)</font>)  
Loads all landmark meshes from an OSM file, applying an optional offset.
    - **Parameters:**
        - `filename` (_str_) 
        - `offset` (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)
    - **Return:** _list([carla.OccupancyMap](#carla.OccupancyMap))_

---

## carla.OccupancyMap<a name="carla.OccupancyMap"></a>
Data structure to manipulate 2D areas of occupancy.

<h3>Instance Variables</h3>

- <a name="carla.OccupancyMap.is_empty"></a>**<font color="#f8805a">is_empty</font>** (_bool_)   
Returns whether this instance represents an empty area.

<h3>Methods</h3>

- <a name="carla.OccupancyMap.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>)  
Constructs an instance representing an empty area.

- <a name="carla.OccupancyMap.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**line**</font>, <font color="#00a6ed">**width**</font>)  
Constructs an instance representing a line buffered by some width.
    - **Parameters:**
        - `line` (_list([carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d))_)
        - `width` (_float_)
    - **Note:** <font color="#8E8E8E">_The buffered line has a width of `width`. Its ends are also approximately buffered round with a diameter of `width`._</font> 

- <a name="carla.OccupancyMap.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**polygon**</font>)  
Constructs an instance representing a closed polygon.
    - **Parameters:**
        - `polygon` (_list([carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d))_)
    - **Note:** <font color="#8E8E8E">_Polygon vertices can be given in any orientation. Additionally, the last vertex **should not** be repeated. An n-gon will require only each of the n vertices._</font>

- <a name="carla.OccupancyMap.save"></a>**<font color="#7fb800">load</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**filename**</font>)  
Saves this instance to a file.
    - **Parameters:**
        - `filename` (_str_)

- <a name="carla.OccupancyMap.load"></a>**<font color="#7fb800">load</font>**(<font color="#00a6ed">**filename**</font>)  
Loads an instance from a file.
    - **Parameters:**
        - `filename` (_str_)

- <a name="carla.OccupancyMap.union"></a>**<font color="#7fb800">union</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)  
Returns the union of this instance's area and that of another instance.
    - **Parameters:**
        - `other` (_[carla.OccupancyMap](#carla.OccupancyMap)_)
    - **Return:** _[carla.OccupancyMap](#carla.OccupancyMap)_

- <a name="carla.OccupancyMap.difference"></a>**<font color="#7fb800">difference</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)  
Returns the difference of this instance's area and that of another instance.
    - **Parameters:**
        - `other` (_[carla.OccupancyMap](#carla.OccupancyMap)_)
    - **Return:** _[carla.OccupancyMap](#carla.OccupancyMap)_
    - **Note:** <font color="#8E8E8E">_The difference returned is the area covered by this instance that is not covered by the other instance._</font>

- <a name="carla.OccupancyMap.intersection"></a>**<font color="#7fb800">intersection</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)  
Returns the intersection of this instance's area and that of another instance.
    - **Parameters:**
        - `other` (_[carla.OccupancyMap](#carla.OccupancyMap)_)
    - **Return:** _[carla.OccupancyMap](#carla.OccupancyMap)_

- <a name="carla.OccupancyMap.buffer"></a>**<font color="#7fb800">buffer</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**width**</font>)  
Returns the area formed by buffering the outlines of this instance's area by some width.
    - **Parameters:**
        - `width` (_float_)
    - **Return:** _[carla.OccupancyMap](#carla.OccupancyMap)_
    - **Note:** <font color="#8E8E8E">_The buffered lines have a width of `width`. Ends are also approximately buffered round with a diameter of `width`._</font>

- <a name="carla.OccupancyMap.contains"></a>**<font color="#7fb800">contains</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**point**</font>)  
Checks if a point is contained in this instance's area.
    - **Parameters:**
        - `point` (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)
    - **Return:** _bool_

- <a name="carla.OccupancyMap.create_sidewalk"></a>**<font color="#7fb800">create_sidewalk</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**distance**</font>)  
Creates a sidewalk along the outlines of this instance's area, with some distance away from the outlines.
    - **Parameters:**
        - `distance` (_float_)
    - **Return:** _[carla.Sidewalk](#carla.Sidewalk)_

- <a name="carla.OccupancyMap.get_triangles"></a>**<font color="#7fb800">get_triangles</font>**(<font color="#00a6ed">**self**</font>)   
Returns the triangles in the triangulation of this instance's area.
    - **Return:** _list([carla.Triangle2D](#carla.Triangle2D))_

- <a name="carla.OccupancyMap.get_mesh_triangles"></a>**<font color="#7fb800">get_mesh_triangles</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**offset**=0.0</font>)   
Returns the triangles, arranged in a list of 3D vertices, in the triangulation of this instance's area with some optional added height offset.
    - **Parameters:**
        - `offset` (_float_)
    - **Return:** _list([carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d))_
    - **Note:** <font color="#8E8E8E">_Both upward and downward facing triangles are produced._</font>

- <a name="carla.OccupancyMap.get_wall_mesh_triangles"></a>**<font color="#7fb800">get_wall_mesh_triangles</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**height**=0.0</font>)   
Returns the triangles, arranged in a list of 3D vertices, in the triangulation of the area formed by sweeping this instance's area vertically by some height.
    - **Parameters:**
        - `height` (_float_)
    - **Return:** _list([carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d))_
    - **Note:** <font color="#8E8E8E">_Both inward and outward facing triangles are produced._</font>

---

## carla.Segment2D<a name="carla.Segment2D"></a>
2D line segment helper class.

<h3>Instance Variables</h3>

- <a name="carla.Segment2D.start"></a>**<font color="#f8805a">start</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)

- <a name="carla.Segment2D.end"></a>**<font color="#f8805a">end</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)

<h3>Methods</h3>

- <a name="carla.Segment2D.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)
    - **Parameters:**
        - `other` (_[carla.Segment2D](#carla.Segment2D)_)

- <a name="carla.Segment2D.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)
    - **Parameters:**
        - `other` (_[carla.Segment2D](#carla.Segment2D)_)

---

## carla.SegmentMap<a name="carla.SegmentMap"></a>
Represents a collection of line segments. Mainly used for efficiently sampling uniformly from collections of line segments.

<h3>Methods</h3>

- <a name="carla.SegmentMap.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**segments**</font>)  
Constructs an instance from a collection of line segments.  
    - **Parameters:**
        - `segments` (_list([carla.Segment2D](#carla.Segment2D))_)

- <a name="carla.OccupancyMap.get_segments"></a>**<font color="#7fb800">get_segments</font>**(<font color="#00a6ed">**self**</font>)  
Returns a list of all segments stored in this instance.
    - **Return:** _list([carla.Segment2D](#carla.Segment2D))_
    - **Note:** <font color="#8E8E8E">_This operation can be time-consuming, as each segment is converted from an internal representation into a _[carla.Segment2D](#carla.Segment2D)_._</font>

- <a name="carla.SegmentMap.seed_rand"></a>**<font color="#7fb800">seed_rand</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**seed**</font>)  
Resets this instance's internal random number generator with a seed value.
    - **Parameters:**
        - `seed` (_int_)

- <a name="carla.SegmentMap.rand_point"></a>**<font color="#7fb800">rand_point</font>**(<font color="#00a6ed">**self**</font>)  
Sample a random point uniformly from this instance's segments.
    - **Return:** _[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_
    - **Note:** <font color="#8E8E8E">_Firstly, a line segment is sampled with probability proportional to its length. Then, a point is uniformly picked along the selected line segment._</font>

- <a name="carla.SegmentMap.union"></a>**<font color="#7fb800">union</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)  
Returns a union of this instance's segments and that of another instance.
    - **Parameters:**
        - `other` (_[carla.SegmentMap](#carla.SegmentMap)_)
    - **Return:** _[carla.SegmentMap](#carla.SegmentMap)_

- <a name="carla.SegmentMap.difference"></a>**<font color="#7fb800">difference</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)  
Returns the difference of this instance's segments and an occupancy map's area.
    - **Parameters:**
        - `other` (_[carla.OccupancyMap](#carla.OccupancyMap)_)
    - **Return:** _[carla.SegmentMap](#carla.SegmentMap)_
    - **Note:** <font color="#8E8E8E">_The difference returned are the segments covered by this instance, cut appropriately, that are not contained in the occupancy maps' area._</font>

- <a name="carla.SegmentMap.intersection"></a>**<font color="#7fb800">intersection</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)  
Returns the intersection of this instance's segments and an occupancy map's area.
    - **Parameters:**
        - `other` (_[carla.OccupancyMap](#carla.OccupancyMap)_)
    - **Return:** _[carla.SegmentMap](#carla.SegmentMap)_

---

## carla.Sidewalk<a name="carla.Sidewalk"></a>
Represents a sidewalk.

<h3>Methods</h3>

- <a name="carla.Sidewalk.get_route_point_position"></a>**<font color="#7fb800">get_route_point_position</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**route_point**</font>)  
Converts a sidewalk route point into the corresponding actual position.
    - **Parameters:**
        - `route_point` (_[carla.SidewalkRoutePoint](#carla.SidewalkRoutePoint)_)
    - **Return:** _[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_

- <a name="carla.Sidewalk.get_nearest_route_point"></a>**<font color="#7fb800">get_nearest_route_point</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**position**</font>)  
Determines the sidewalk route point closest to a position.
    - **Parameters:**
        - `position` (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)
    - **Return:** _[carla.SidewalkRoutePoint](#carla.SidewalkRoutePoint)_

- <a name="carla.Sidewalk.get_next_route_point"></a>**<font color="#7fb800">get_next_route_point</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**route_point**</font>, <font color="#00a6ed">**distance**</font>)  
Gets the next route point succeeding some route point by some distance.
    - **Parameters:**
        - `route_point` (_[carla.SidewalkRoutePoint](#carla.SidewalkRoutePoint)_)
        - `distance` (_float_)
    - **Return:** _[carla.SidewalkRoutePoint](#carla.SidewalkRoutePoint)_
    - **Note:** <font color="#8E8E8E">_The returned route point is ahead of the given route point anticlockwise along the route point's polygon._</font>

- <a name="carla.Sidewalk.get_previous_route_point"></a>**<font color="#7fb800">get_previous_route_point</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**route_point**</font>, <font color="#00a6ed">**distance**</font>)  
Gets the next route point preceding some route point by some distance.
    - **Parameters:**
        - `route_point` (_[carla.SidewalkRoutePoint](#carla.SidewalkRoutePoint)_)
        - `distance` (_float_)
    - **Return:** _[carla.SidewalkRoutePoint](#carla.SidewalkRoutePoint)_
    - **Note:** <font color="#8E8E8E">_The returned route point is ahead of the given route point clockwise along the route point's polygon._</font>

- <a name="carla.Sidewalk.create_occupancy_map"></a>**<font color="#7fb800">create_occupancy_map</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**width**</font>)  
Creates an occupancy map by buffering the paths in this instance by some width.
    - **Parameters:**
        - `width` (_float_)
    - **Return:** _[carla.OccupancyMap](#carla.OccupancyMap)_
    - **Note:** <font color="#8E8E8E">_The buffered lines have a width of `width`. Ends are also approximately buffered round with a diameter of `width`._</font>

- <a name="carla.Sidewalk.create_segment_map"></a>**<font color="#7fb800">create_segment_map</font>**(<font color="#00a6ed">**self**</font>)  
Creates a `carla.SegmentMap` using the paths found in this instance.
    - **Return:** _[carla.SegmentMap](#carla.SegmentMap)_

- <a name="carla.Sidewalk.intersects"></a>**<font color="#7fb800">intersects</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**segment**</font>)  
Checks if a line segment intersects with the paths in this instance.
    - **Parameters:**
        - `segment` (_[carla.Segment2D](#carla.Segment2D)_)
    - **Return:** _bool_

---

## carla.SidewalkRoutePoint<a name="carla.SidewalkRoutePoint"></a>
Represents a sidewalk route point. 

<h3>Instance Variables</h3>

- <a name="carla.SidewalkRoutePoint.polygon_id"></a>**<font color="#f8805a">polygon_id</font>** (_int_)

- <a name="carla.SidewalkRoutePoint.segment_id"></a>**<font color="#f8805a">segment_id</font>** (_int_)

- <a name="carla.SidewalkRoutePoint.offset"></a>**<font color="#f8805a">offset</font>** (_float_)


<h3>Methods</h3>
- <a name="carla.SidewalkRoutePoint.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.SumoNetwork<a name="carla.SumoNetwork"></a>
Represents a SUMO network.

<h3>Instance Variables</h3>

- <a name="carla.SumoNetwork.bounds_min"></a>**<font color="#f8805a">bounds_min</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)    
Gets the minimum point of this instance's bounds.

- <a name="carla.SumoNetwork.bounds_max"></a>**<font color="#f8805a">bounds_max</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)    
Gets the maximum point of this instance's bounds.

- <a name="carla.SumoNetwork.original_bounds_min"></a>**<font color="#f8805a">original_bounds_min</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)    
Gets the minimum point (in LatLon) of this instance's geographic bounds.

- <a name="carla.SumoNetwork.original_bounds_max"></a>**<font color="#f8805a">original_bounds_max</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)    
Gets the maximum point (in LatLon) of this instance's geographic bounds.

- <a name="carla.SumoNetwork.offset"></a>**<font color="#f8805a">offset</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)     
Gets the offset applied to the SUMO network to move it to the origin.

<h3>Methods</h3>

- <a name="carla.SumoNetwork.get_route_point_position"></a>**<font color="#7fb800">get_route_point_position</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**route_point**</font>)  
Converts a SUMO network route point into the corresponding actual position.
    - **Parameters:**
        - `route_point` (_[carla.SumoNetworkRoutePoint](#carla.SumoNetworkRoutePoint)_)
    - **Return:** _[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_

- <a name="carla.SumoNetwork.get_nearest_route_point"></a>**<font color="#7fb800">get_nearest_route_point</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**position**</font>)  
Determines the SUMO network route point closest to a position.
    - **Parameters:**
        - `position` (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)
    - **Return:** _[carla.SumoNetworkRoutePoint](#carla.SumoNetworkRoutePoint)_

- <a name="carla.SumoNetwork.get_next_route_points"></a>**<font color="#7fb800">get_next_route_points</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**route_point**</font>, <font color="#00a6ed">**distance**</font>)  
Gets the list of possible next route points succeeding some route point by some distance.
    - **Parameters:**
        - `route_point` (_[carla.SumoNetworkRoutePoint](#carla.SumoNetworkRoutePoint)_)
        - `distance` (_float_)
    - **Return:** _list([carla.SumoNetworkRoutePoint](#carla.SumoNetworkRoutePoint))_

- <a name="carla.SumoNetwork.get_next_route_paths"></a>**<font color="#7fb800">get_next_route_paths</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**route_point**</font>, <font color="#00a6ed">**num_points**</font>, <font color="#00a6ed">**interval**</font>)  
Gets the list of possible paths succeeding some route point. Each path spans a specified number of points with some given interval.
    - **Parameters:**
        - `route_point` (_[carla.SumoNetworkRoutePoint](#carla.SumoNetworkRoutePoint)_)
        - `num_points` (_int_)
        - `interval` (_float_)
    - **Return:** _list(list([carla.SumoNetworkRoutePoint](#carla.SumoNetworkRoutePoint)))_

- <a name="carla.SumoNetwork.create_occupancy_map"></a>**<font color="#7fb800">create_occupancy_map</font>**(<font color="#00a6ed">**self**</font>)  
Creates the occupancy map for this instance.
    - **Return:** _[carla.OccupancyMap](#carla.OccupancyMap)_

- <a name="carla.SumoNetwork.create_roadmark_occupancy_map"></a>**<font color="#7fb800">create_roadmark_occupancy_map</font>**(<font color="#00a6ed">**self**</font>)  
Creates the occupancy map for this instance's roadmarks.
    - **Return:** _[carla.OccupancyMap](#carla.OccupancyMap)_

- <a name="carla.SumoNetwork.create_segment_map"></a>**<font color="#7fb800">create_segment_map</font>**(<font color="#00a6ed">**self**</font>)  
Creates a `carla.SegmentMap` using the lanes found in this instance.
    - **Return:** _[carla.SegmentMap](#carla.SegmentMap)_


---

## carla.SumoNetworkRoutePoint<a name="carla.SumoNetworkRoutePoint"></a>
Represents a SUMO network route point. 


<h3>Instance Variables</h3>

- <a name="carla.SumoNetworkRoutePoint.edge"></a>**<font color="#f8805a">edge</font>** (_str_)
- <a name="carla.SumoNetworkRoutePoint.lane"></a>**<font color="#f8805a">lane</font>** (_int_)
- <a name="carla.SumoNetworkRoutePoint.segment"></a>**<font color="#f8805a">segment</font>** (_int_)
- <a name="carla.SumoNetworkRoutePoint.offset"></a>**<font color="#f8805a">offset</font>** (_float_)


<h3>Methods</h3>
- <a name="carla.SumoNetworkRoutePoint.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>)  
    
---

## carla.Triangle2D<a name="carla.Triangle2D"></a>
2D triangle helper class.

<h3>Instance Variables</h3>

- <a name="carla.Triangle2D.v0"></a>**<font color="#f8805a">v0</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)

- <a name="carla.Triangle2D.v1"></a>**<font color="#f8805a">v1</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)

- <a name="carla.Triangle2D.v2"></a>**<font color="#f8805a">v2</font>** (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)

<h3>Methods</h3>

- <a name="carla.Triangle2D.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)
    - **Parameters:**
        - `other` (_[carla.Triangle2D](#carla.Triangle2D)_)

- <a name="carla.Triangle2D.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)
    - **Parameters:**
        - `other` (_[carla.Triangle2D](#carla.Triangle2D)_)

---

## carla.Vector2D<a name="carla.Vector2D"></a>
Only additional features introduced in SUMMIT to `carla.Vector2D` are listed here. Please refer to [CARLA's implementation](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d) for the remaining original features, all of which are also accessible.

<h3>Methods</h3>

- <a name="carla.Triangle2D.squared_length"></a>**<font color="#7fb800">squared_length</font>**(<font color="#00a6ed">**self**</font>)    
Returns the squared length of this instance.
    - **Return:** _float_

- <a name="carla.Triangle2D.length"></a>**<font color="#7fb800">length</font>**(<font color="#00a6ed">**self**</font>)    
Returns the length of this instance.
    - **Return:** _float_

- <a name="carla.Triangle2D.make_unit_vector"></a>**<font color="#7fb800">make_unit_vector</font>**(<font color="#00a6ed">**self**</font>)    
Returns the unit vector of this instance.
    - **Return:** _[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_

- <a name="carla.Triangle2D.rotate"></a>**<font color="#7fb800">rotate</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**angle**</font>)    
Returns the result of rotating this instance by an angle.
    - **Parameters:**
        - `angle`: (_float_) (in radians)
    - **Return:** _[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_

- <a name="carla.Triangle2D.dot_product"></a>**<font color="#7fb800">dot_product</font>**(<font color="#00a6ed">**vector1**</font>, <font color="#00a6ed">**vector2**</font>)    
Returns the dot product of two vectors.
    - **Parameters:**
        - `vector1`: (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)
        - `vector2`: (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)
    - **Return:** _float_

---

## carla.World<a name="carla.World"></a>
Only additional features introduced in SUMMIT to `carla.World` are listed here. Please refer to [CARLA's implementation](https://carla.readthedocs.io/en/latest/python_api/#carlaworld) for the remaining original features, all of which are also accessible.

<h3>Methods</h3>

- <a name="carla.world.spawn_dynamic_mesh"></a>**<font color="#7fb800">spawn_dynamic_mesh</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**triangles**</font>, <font color="#00a6ed">**material**</font>, <font color="#00a6ed">**segmentation_tag**</font>)    
Spawns a dynamic mesh using mesh triangles, together with a specified material and [segmentation tag](https://carla.readthedocs.io/en/latest/cameras_and_sensors/#sensorcamerasemantic_segmentation). Returns the id of the spawned dynamic mesh (note: these ids are **not** the same as actor ids).
    - **Parameters:**
        - `triangles`: _list([carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d))_
        - `material`: _str_ 
        - `segmentation_tag`: _int_
    - **Return:** _int_

- <a name="carla.world.spawn_dynamic_tile_mesh"></a>**<font color="#7fb800">spawn_dynamic_tile_mesh</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**bounds_min**</font>, <font color="#00a6ed">**bounds_max**</font>, <font color="#00a6ed">**data**</font>, <font color="#00a6ed">**segmentation_tag**</font>)    
Spawns a rectangular dynamic mesh of a JPEG image [segmentation tag](https://carla.readthedocs.io/en/latest/cameras_and_sensors/#sensorcamerasemantic_segmentation). Returns the id of the spawned dynamic mesh (note: these ids are **not** the same as actor ids).
    - **Parameters:**
        - `bounds_min`: _[carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d)_
        - `bounds_max`: _[carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d)_
        - `data`: _list(int)_ - The JPEG data, as an array of bytes. 
        - `segmentation_tag`: _int_
    - **Return:** _int_

- <a name="carla.world.destroy_dynamic_mesh"></a>**<font color="#7fb800">destroy_dynamic_mesh</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**id**</font>)    
Destroys a dynamic mesh given its id. Returns whether the operation succeeded.
    - **Parameters:**
        - `id`: _int_
    - **Return:** _bool_

---

## command.SpawnDynamicMesh<a name="command.SpawnDynamicMesh"></a>
Spawns a dynamic mesh out of mesh triangles.

<h3>Instance Variables</h3>
- <a name="command.SpawnDynamicMesh.triangles"></a>**<font color="#f8805a">triangles</font>** (_list([carla.Triangle2D](#carla.Triangle2D))_)
- <a name="command.SpawnDynamicMesh.material"></a>**<font color="#f8805a">material</font>** (_str_)
- <a name="command.SpawnDynamicMesh.semantic_tag"></a>**<font color="#f8805a">semantic_tag</font>** (_int_)

<h3>Methods</h3>

- <a name="command.SpawnDynamicMesh.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**triangles**</font>, <font color="#00a6ed">**material**</font>, <font color="#00a6ed">**segmentation_tag**</font>)    
    - **Parameters:**
        - `triangles`: _list([carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d))_
        - `material`: _str_ 
        - `segmentation_tag`: _int_

---

## command.DestroyDynamicMesh<a name="command.DestroyDynamicMesh"></a>
Destroys a dynamic mesh given its id.

<h3>Instance Variables</h3>
- <a name="command.DestroyDynamicMesh.id"></a>**<font color="#f8805a">id</font>** (_int_)

<h3>Methods</h3>

- <a name="command.DestroyDynamicMesh.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**id**</font>)    
    - **Parameters:**
        - `id`: _int_

