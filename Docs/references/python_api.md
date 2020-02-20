!!! important
    SUMMIT's Python API is an extension of CARLA's [Python API](https://carla.readthedocs.io/en/latest/python_api/). All classes and methods appearing in CARLA's Python API are accessible in SUMMIT. 
    
    However, since some components were designed for different use cases, not all components from CARLA may work with SUMMIT components, and vice versa.

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
- <a name="carla.AABB2D.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  


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


## carla.Landmark<a name="carla.Landmark"></a>
Helper class to load landmark meshes from OSM files.

- <a name="carla.Landmark.load"></a>**<font color="#7fb800">load</font>**(<font color="#00a6ed">**filename**</font>, <font color="#00a6ed">**offset**=carla.Vector2D(0, 0)</font>)  
Loads all landmark meshes from an OSM file, applying an optional offset.
    - **Parameters:**
        - `filename` (_str_) 
        - `offset` (_[carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d)_)
    - **Return:** _list([carla.OccupancyMap](#carla.OccupancyMap))_

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
    - **Note:** The buffered line has a width of `width`. Its ends are also approximately buffered round with a diameter of `width`.
- <a name="carla.OccupancyMap.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**polygon**</font>)  
Constructs an instance representing a closed polygon.
    - **Parameters:**
        - `polygon` (_list([carla.Vector2D](https://carla.readthedocs.io/en/latest/python_api/#carlavector2d))_)
    - **Note:** Polygon vertices can be given in any orientation. Additionally, the last vertex **should not** be repeated. An n-gon will require only each of the n vertices.
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
    - **Note:** The difference return is the area covered by this instance that is not covered by the other instance. See [here](https://en.wikipedia.org/wiki/Complement_(set_theory)#Relative_complement) for more details.
- <a name="carla.OccupancyMap.intersection"></a>**<font color="#7fb800">intersection</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)  
Returns the intersection of this instance's area and that of another instance.
    - **Parameters:**
        - `other` (_[carla.OccupancyMap](#carla.OccupancyMap)_)
    - **Return:** _[carla.OccupancyMap](#carla.OccupancyMap)_
- <a name="carla.OccupancyMap.buffer"></a>**<font color="#7fb800">buffer</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**width**</font>)  
Returns the area formed by buffering the outlines of this instance's area by some width.
    - **Parameters:**
        - `other` (_float_)
    - **Return:** _[carla.OccupancyMap](#carla.OccupancyMap)_
    - **Note:** The buffered lines have a width of `width`. Ends are also approximately buffered round with a diameter of `width`.
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
    - **Return:** _list([carla.Vector3D](#carla.Vector3D))_
    - **Note:** Both upward and downward facing triangles are produced.
- <a name="carla.OccupancyMap.get_wall_mesh_triangles"></a>**<font color="#7fb800">get_wall_mesh_triangles</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**height**=0.0</font>)   
Returns the triangles, arranged in a list of 3D vertices, in the triangulation of the area formed by sweeping this instance's area vertically by some height.
    - **Parameters:**
        - `offset` (_float_)
    - **Return:** _list([carla.Vector3D](#carla.Vector3D))_
    - **Note:** Both inward and outward facing triangles are produced.

## carla.Segment2D<a name="carla.Segment2D"></a>
## carla.Segments<a name="carla.Segments"></a>
## carla.Sidewalk<a name="carla.Sidewalk"></a>
## carla.SidewalkRoutePoint<a name="carla.SidewalkRoutePoint"></a>
## carla.SumoNetwork<a name="carla.SumoNetwork"></a>
## carla.SumoNetworkRoutePoint<a name="carla.SumoNetworkRoutePoint"></a>
## carla.Triangle2D<a name="carla.Triangle2D"></a>
