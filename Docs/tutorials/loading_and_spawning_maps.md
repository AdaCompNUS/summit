!!! Important
    SUMMIT comes with scripts that spawn all relevant map objects covered in this tutorial. You may use it directly them `<summit_root>/PythonAPI/examples/spawn_meshes.py` and `<summit_root>/PythonAPI/examples/spawn_imagery.py`, without going through this tutorial.
   
    Note that these scripts require that you have already [cached the map object meshes](/tutorials/preparing_maps/#optional-downloading-satellite-imagery) and [downloaded the satellite imagery](/tutorials/preparing_maps/#optional-caching-map-object-meshes). For built-in SUMMIT maps, these have already been done for you, so you may go ahead with using the scripts directly, skipping this tutorial.

# Connecting to the simulator
The steps to connect to the simulator [derives from CARLA](https://carla.readthedocs.io/en/latest/python_api_tutorial/#connecting-and-retrieving-the-world). A client object is created from which the world is received:

```python
client = carla.Client('localhost', 2000)
world = client.get_world()
```

# Roads and roadmarks
Roads are represented using SUMO networks, which are loaded into memory via a SUMO network file:

```python
sumo_network = carla.SumoNetwork.load(PATH_TO_SUMO_NETWORK_FILE)
```

The occupancy map (i.e. mesh) for the road can be produced by calling:

```python
sumo_network_occupancy = sumo_network.create_occupancy_map()
```

If you required roadmarks (e.g. lane dividers), SUMMIT provides a fully automated way to generate the roadmarks' mesh:
```python
roadmark_occupancy = sumo_network.create_roadmark_occupancy_map()
```

The road and roadmarks' meshes are then sent to the simulator and spawned dynamically:

```python
# Get mesh triangles, and defines material and segmentation tag for road.
# Roadmarks' mesh is subtracted from the road's mesh to prevent flickering due to overlaps.
road_triangles = sumo_network_occupancy.difference(roadmark_occupancy).get_mesh_triangles()
road_material = '/Game/Carla/Static/GenericMaterials/Masters/LowComplexity/M_Road1'
road_segmentation = 7 # Road (defined by CARLA).

# Get mesh triangles, and defines material and segmentation tag for roadmarks.
roadmark_triangles = roadmark_occupancy.get_mesh_triangles()
roadmark_material = '/Game/Carla/Static/GenericMaterials/LaneMarking/M_MarkingLane_W'
roadmark_segmentation = 6 # Road line (defined by CARLA).

# Spawn road and roadmarks dynamically in simulator.
world.spawn_dynamic_mesh(road_triangles, road_material, road_segmentation)
world.spawn_dynamic_mesh(roadmark_triangles, roadmark_material, roadmark_segmentation)
```

# Sidewalks
Sidewalks are represented using a set of oriented polygons with holes using `carla.Sidewalk` objects. 
In SUMMIT, sidewalks are automatically calculated as boundaries along road meshes:

```python
# Create sidewalk 1.5 meters from road's mesh.
sidewalk = sumo_network_occupancy.create_sidewalk(1.5)

# Create sidewalk's mesh for a sidewalk width of 3.0 meters.
sidewalk_occupancy = sidewalk.create_occupancy_map(3.0)

# Get mesh triangles, and defines sidewalk material and segmentation tag for sidewalk.
sidewalk_triangles = sidewalk_occupancy.get_mesh_triangles()
sidewalk_material = '/Game/Carla/Static/GenericMaterials/Ground/GroundWheatField_Mat'
sidewalk_segmentation = 8 # Sidewalk (defined by CARLA).

# Spawn sidewalk dynamically in simulator.
world.spawn_dynamic_mesh(sidewalk_triangles, sidewalk_material, sidewalk_segmentation)
```


# Landmarks
Landmarks can be loaded from the map's OSM file:

```python
landmark_occupancies = carla.Landmark.load(PATH_TO_OSM_FILE, sumo_network.offset)
```

!!! note
    The `sumo_network.offset` argument applies an offset to each landmark equal to the offset applied to the SUMO network. This is required because the SUMO network itself is offset (by `sumo_network.offset`) such that its bounds are aligned to the origin in the simulator. On the other hand, the landmarks maintain their global geographic position. Without applying the correct offset, the landmarks may end up at positions far away from the SUMO network.

For each landmark, we can retrieve the ground, ceiling, and wall meshes to be spawned in the simulator:

```python
landmark_material = '/Game/Carla/Static/Buildings/aa_wall_mat'
landmark_segmentation = 1 # Building (defined by CARLA)

for landmark_occupancy in landmark_occupancies:
    # Get ground mesh, applying a vertical offset of 0 meters.
    ground_mesh_triangles = landmark_occupancy.get_mesh_triangles(0.0)

    # Get ceiling mesh triangles, with a vertical offset of 20.0 meters.
    ceiling_mesh_triangles = landmark_occupancy.get_mesh_triangles(20.0)

    # Get vertically rising mesh triangles, with a height of 20.0 meters.
    wall_mesh_triangles = landmark_occupancy.get_wall_mesh_triangles(20.0)
    
    # Spawn meshes dynamically in simulator.
    world.spawn_dynamic_mesh(ground_mesh_triangles, landmark_material, landmark_segmentation)
    world.spawn_dynamic_mesh(ceiling_mesh_triangles, landmark_material, landmark_segmentation)
    world.spawn_dynamic_mesh(wall_mesh_triangles, landmark_material, landmark_segmentation)
```

The landmarks may sometime overlap with the roads and sidewalks. You may wish to do some preprocessing to remove these overlaps:

```python
sumo_network_occupancy = ... # Get road mesh.
sidewalk_occupancy = ... # Get sidewalk mesh.
landmark_occupancies = ... # Get landmark meshes.

# Subtract road mesh and sidewalk mesh from landmark meshes.
landmark_occupancies = [
        l.difference(sumo_network_occupancy).difference(sidewalk_occupancy) 
        for l in landmark_occupancies]

# Filter empty landmark meshes.
landmark_occupancies = [l for l in landmark_occupancies if not l.is_empty]
```

# Satellite/general imagery
To spawn satellite imagery in SUMMIT, we provide a utility script at `<summit_root>/PythonAPI/examples/spawn_imagery.py`. It assumes that you have already [downloadeded the satellite imagery](/tutorials/preparing_maps/#optional-downloading-satellite-imagery). To use, run

```python
spawn_imagery.py --dataset <map_name>
```

!!! note
    Spawning satellite imagery involves a tedious amount of work (such as looking up the map tile indices from geographical coordinates), and we highly recommend using the provided script which already deals with these efforts.

The dynamic image tiles that SUMMIT expose to spawn satellite imagery can also be used to spawn any JPEG image in general. The below example spawns a an arbitrary image on a square region that is slanted along the z axis.

```python
with open(JPEG_PATH, 'rb') as f:
    # NOTE: Python 2 reads the file as a string instead of as an
    # array of bytes. ord(b) checks for this and converts accordingly.
    data = [ord(b) if isinstance(b, str) else b for b in f.read()]

segmentation = 9 # Vegetation (defined by CARLA)
bounds_min = carla.Vector3D(0, 0, -10)
bounds_max = carla.Vector3D(100, 100, 10)

world.spawn_dynamic_tile_mesh(bounds_min, bounds_max, data, segmentation)
```

# Saving and loading meshes
You may wish to save meshes onto the disk, and reload them for future use, speeding up loading times by elimiating the unnecessary recomputations:

```python
# Saves mesh to the disk at SAVE_PATH.
sumo_network_occupancy.save(SAVE_PATH)

# Reload mesh from the disk.
sumo_network_occupancy = carla.OccupancyMap.load(SAVE_PATH)

# Spawn dynamically in simulator.
world.spawn_dynamic_mesh(sumo_network_occupancy.get_mesh_triangles(), road_material, road_segmentation)
```

This example works for any carla.OccupancyMap instance, not just for roads.
