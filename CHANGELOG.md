## CARLA 0.9.6

  * Upgraded to Unreal Engine 4.22
  * Added Vulkan support, if installed, CARLA will use Vulkan, use `-opengl` flag to launch with OpenGL
  * The simulator is now compiled in "Shipping" mode, faster but it accepts less command-line arguments
  * Pedestrians are back:
    - Spawn pedestrians that will roam randomly on sidewalks
    - The script 'spawn_npc.py' spawns now pedestrians, adjust the number with the flag `-w`
    - Added navigation meshes for each maps for pedestrian navigation
  * Allow adding custom props (FBX) to CARLA Blueprint library so they are spawnable
  * Simplified pipeline for importing and packaging maps and custom props
  * Vehicle physics:
    - Added access to vehicle transmission details
    - Added access to vehicle physics brake values
    - Added tire friction trigger boxes for simulating slippery surfaces
  * Added camera gamma correction as command-line argument to manual_control.py
  * Added ability to set motion blur settings for RGB camera in sensor python blueprint
  * Added C++ client example using LibCarla
  * Added PythonAPI documentation generator, we documented in detail all the Python reference
  * Added a new Python script config.py that allows the user to configure the simulator from the command-line
  * New recorder features:
    - Documented recorded system and binary file
    - Added optional parameter to show more details about a recorder file (related to `show_recorder_file_info.py`)
    - Added playback speed (slow/fast motion) to the replayer
    - Allow custom paths for saving the recorded files
    - More data is now recorded to replay animations:
      + Wheels of vehicles are animated (steering, throttle, handbrake), also bikes and motorbikes
      + Walker animations are simulated (through speed of walker)
  * New high quality pedestrians: female, girl and boy; improved meshes and textures
  * More color and texture variations for each pedestrian
  * New vehicle Audi Etron: 25.000 tris and LODs
  * New material for Mustang, new system that will allow us to improve all the vehicle materials
  * Improved vehicle Tesla
  * New high-quality "Default" weather tailor-made for each map
  * Improved the rest of weather profiles too
  * RGB camera improvements:
    - Enabled temporal antialiasing and motion blur
    - Added gamma value and motion blur as a blueprint attributes
    - Enabled texture streaming for scene captures
  * API changes:
    - Renamed `frame_count` and `frame_number` as `frame`, old members are kept as deprecated
    - `world.wait_for_tick()` now returns a `carla.WorldSnapshot`
    - The callback of `world.on_tick(callback)` now receives a `carla.WorldSnapshot`
    - Deprecated waypoint's `is_intersection`, use `is_junction` instead
  * API extensions:
    - Added attachment type "SpringArm" for cinematic cameras
    - Added waypoint's `junction_id` that returns de OpenDrive identifier of the current junction
    - Added `world.get_actor(id)` to find a single actor by id
    - Added `carla.WeatherParameters.Default` for the default (tailor-made for each town) weather profile
    - Added `WorldSnapshot` that contains a list of `ActorSnapshot`, allows capturings a "still image" of the world at a single frame
    - Added `world.tick()` now synchronizes with the simulator and returns the id of the newly started frame
    - Added `world.apply_settings(settings)` now synchronizes with the simulator and returns the id of the frame when the settings took effect
    - Added `world.remove_on_tick(id)` to allow removing on tick callbacks
    - Added allow setting fixed frame-rate from client-side, now is part of `carla.WorldSettings`
    - Added `is_invincible` to walkers
  * Several optimizations to the RPC server, now supports a bigger load of async messages
  * Updated DebugHelper to render on Shipping packages, it has also better performance
  * Updated OpenDriveActor to use the new Waypoint API
  * Removed deprecated code and content
  * Exposed waypoints and OpenDrive map to UE4 Blueprints
  * Change the weight of cars. All cars have been compared with the real to have a feedback more real
  * Recorder fixes:
    - When a recorded session finish replaying, all vehicles will continue in autopilot, and all pedestrians will stop
    - Fixed a possible crash if an actor is respawned before the episode is ready when a new map is loaded automatically
    - Actors at start of playback could interpolate positions from its current position instead than the recorded position
    - Camera following in playback was not working if a new map was needed to load
    - API function 'show_recorder_file_info' was showing the wrong parent id
    - Script 'start_recording.py' now properly saves destruction of actors at stop
    - Problem when vehicles enable autopilot after a replayer, now it works better
  * Fixed dead-lock when loading a new map in synchronous mode
  * Fixed get_actors may produce actors without parent
  * Fixed std::bad_cast when importing other libraries, like tensorflow, before carla
  * Fixed latitude in WGS84 reprojection code such that Latitudes increase as one move north in CARLA worlds
  * Fixed walking animations, the animations now go at the same speed as the game
  * Fixed loading and reloading world not using the timeout
  * Fixed XODR files can be found now anywhere in content
  * Fixed bug related with Pygame error of surface too large, added sidewalks and improved lane markings in `no_rendering_mode.py`
  * Fixed Lidar effectiveness bug in manual_control.py
  * Fixed wrong units in VehiclePhysicsControl's center of mass
  * Fixed semantic segmentation of bike riders
  * Fixed inconsistent streetlights in Town03
  * Fixed incorrect vehicle bounds

## CARLA 0.9.5

  * Added `client_bounding_boxes.py` to show bounding boxes client-side
  * New Town07, rural environment with narrow roads
  * Reworked OpenDRIVE parser and waypoints API
    - Fixed several situations in which the XODR was incorrectly parsed
    - Exposed more information: lane marking, lane type, lane section id, s
    - API change: waypoint's `lane_type` is now an enum, `carla.LaneType`
    - API change: `carla.LaneMarking` is not an enum anymore, extended with color, type, lane change, and width
    - API extension: `map.get_waypoint` accepts an extra optional flag argument `lane_type` for filtering lane types
    - API extension: `carla.Map` can be constructed off-line out of XODR files, `carla.Map(town_name, xodr_content)`
    - API extension: `id` property to waypoints, uniquely identifying waypoints up to half centimetre precision
  * API change: Renamed "lane_invasion" to "lane_detector", added too its server-side sensor to be visible to other clients
  * API extension: new carla.command.SpawnActor to spawn actors in batch
  * API extension: `map.transform_to_geolocation` to transform Location to GNSS GeoLocation
  * API extension: added timestamp (elapsed simulation seconds) to SensorData
  * API extension: method `client.apply_batch_sync` that sends commands in batch and waits for server response
  * API extension: optional argument "actor_ids" to world.get_actors to request only the actors with the ids provided
  * Migrated Content to AWS
  * Updated `spawn_npc.py` to spawn vehicles in batch
  * Added --rolename to "manual_control.py"
  * Added options to "no_rendering_mode.py" to draw extra road information
  * Added "scene_layout.py" to retrieve the whole information in the scene as Python dict
  * Basic agent integrated with global router
  * Allow usage of hostname for carla::Client and resolve them to IP addresses
  * Added new pack of assets
    - Windmill, different farm houses, silo
    - Plants corn, dandelion, poppy, and grass
    - Yield traffic sign
  * Added modular buildings New York style
  * Added marking lanes in Town03
  * Added command-line arguments to simulator to disable rendering and set the server timeout
  * Improved performance in Town01 and Town02
  * Changed yellow marking lane from Town01 and Town02 to dashed yellow marking lane
  * Improved lane cross detection to use the new Waypoint API
  * Enhanced stop triggers options
  * Fixed semantic segmentation tags in Town04, Town05, Town06
  * Fixed tree collision in Town01
  * Fixed VehicleSpawnPoint out of the road in Town01
  * Fixed geo-reference of Town01 and Town07
  * Fixed floating pillars in Town04
  * Fixed floating building in Town03
  * Fixed vehicles missing the route if autopilot enabled too late
  * Fixed division by zero in is_within_distance_ahead()
  * Fixed local planner to avoid premature route pruning at path overlaps
  * Fixed global router behavior to be consistent with new Waypoint API
  * Fixed clean up of local_planner when used by other modules
  * Fixed python client DLL error on Windows
  * Fixed wrong type returned by `ActorList.Filter(...)`
  * Fixed wheel's tire friction affecting all vehicles from physics control parameters
  * Fixed obstacle detector not working
  * Fixed small float bug in misc.py


## CARLA 0.9.4

  * Added recording and playback functionality
  * Added synchronous mode, simulator waits until a client sends a "tick" cue, `client.tick()`
  * Allow changing map from client-side, added `client.load_world(name)`, `client.reload_world()`, and `client.get_available_maps()`
  * Added scripts and tools to import maps directly from .fbx and .xodr files into the simulator
  * Exposed minimum physics control parameters for vehicles' engine and wheels
  * Allow controlling multiple actors in "batch mode"
  * New Town06, featuring a "Michigan left" intersection including:
    - Connection ramp between two highways
    - Incorporation to a highway requiring changing several lanes to take another exit
    - Junctions supporting different scenarios
  * New traffic signs assets: one-way, no-turn, more speed limits, do not enter, arrow floors, Michigan left, and lane end
  * New pedestrian texture to add more variations
  * New road PBR material
  * Extended the waypoint API with `lane_change`, `lane_type`, `get_right_lane()` and `get_left_lane()`
  * Added world settings for changing no-rendering mode and synchronous mode at run-time
  * Added methods to acquire a traffic light's pole index and all traffic lights in it's group
  * Added performance benchmark script to measure the simulator's rendering performance
  * Added `manual_control_steeringwheel.py` to control agents using Logitech G29 steering wheels (and maybe others)
  * Added movable props present in the map (e.g. chairs and tables) as actors so they can be controlled from Python
  * Added recording and playback bindings to `manual_control.py` script
  * Removed `world.map_name` from API, use `world.get_map().name` instead
  * Refactored `no_rendering_mode.py` to improve performance and interface
  * Several improvements to the build system for Windows
  * Expose traffic sign's trigger volumes on Python API
  * Improved export/import map tools
  * Simplify Dockerfile halving Carla Docker image size
  * Episodes have now a random unique id to avoid collisions between runs
  * Reduced overhead of many RPC calls by sending only actor IDs (instead of serializing all the actor attributes every time)
  * Added priority system for vehicle control input (internal, not exposed in API)
  * Removed "Example.CarlaSettings.ini", you can still use it, but it's no longer necessary
  * Improved time-out related error messages
  * Fixed Town01 placed 38 meters above the zero
  * Fixed parsing of OpenDrive geo-reference exported by RoadRunner
  * Fixed issue of retrieving an empty list when calling `world.get_actors()` right after creating the world
  * Fixed a few synchronization issues related to changing the world at runtime
  * Fixed traffic light when it gets illuminated by the hero vehicle in `no_rendering_mode.py`
  * Fixed `manual_control.py` and `no_rendering_mode.py` to prevent crashes when used in "no rendering mode"
  * Fixed traffic signs having the trigger box rotated
  * Fixed female walk animation
  * Fixed BP_MultipleFloor, tweaked offset in BaseFloor to adjust meshes between them
  * Fixed static objects present in the map were marked as "movable"

## CARLA 0.9.3

  * Upgraded to Unreal Engine 4.21
  * Upgraded Boost to 1.69.0
  * New Town04 (biggest so far), includes a freeway, new bridge and road barrier, a nicer landscape based on height-map, and new street props
  * New Town05, adding more variety of intersections for the scenario runner
  * Redesigned pedestrian models and animations (walk and idle) for male and female characters
  * Added sensor for detecting obstacles (ray-cast based)
  * Added sensor GNSS (GPS)
  * Basic agent integrated with global router
  * Added a few methods to manage an actor:
    - set_velocity: for setting the linear velocity
    - set_angular_velocity: for setting the angular velocity
    - get_angular_velocity: for getting the angular velocity
    - add_impulse: for applying an impulse (in world axis)
  * Renamed vehicle.get_vehicle_control() to vehicle.get_control() to be consistent with walkers
  * Added new mesh for traffic lights
  * Added new pine tree assets, with their LODs finely tuned for performance
  * Added point transformation functionality for LibCarla and PythonAPI
  * Added "sensor_tick" attribute to sensors (cameras and lidars) to specify the capture rate in seconds
  * Added Export/Import map tools
  * Added "get_forward_vector()" to rotation and transform, retrieves the unit vector on the rotation's X-axis
  * Added support for Deepin in PythonAPI's setup.py
  * Added support for spawning and controlling walkers (pedestrians)
  * Updated BasicAgent to allow setting target_speed and handle US-style traffic lights properly
  * OpenDriveActor has been rewritten using the Waypoint API, this has fixed some bugs
  * Remove crash reporter from packaged build
  * Improved simulator fatal error handling, now uses UE4 fatal error system
  * LibCarla server pipeline now compiles with exceptions disabled for better performance and compatibility with UE4
  * Fixed TCP accept error, too many open files while creating and destroying a lot of sensors
  * Fixed lost error messages in client-side, now when a request fails it reports the reason
  * Fixed global route planner to handle round about turns and made the code consistent with local planner
  * Fixed local planner to avoid premature route pruning at path overlaps
  * Fixed autopilot direction not properly initialized that interfered with the initial raycast direction
  * Fixed crash when an actor was destroyed but not de-registered, e.g. falling out of world bounds

## CARLA 0.9.2

  * Updated ROS bridge for CARLA 0.9.X (moved to its own repository)
  * Added Python API "agents" extension, includes
    - Global route planner based on the Waypoints API (compatible with OpenDrive)
    - BasicAgent: new client agent that can drive to a given coordinate of the map using the waypoint API and PID controllers, attending to other vehicles and traffic lights
    - RoamingAgent: new client agent that can drive at different speeds following waypoints based on PID controllers, attending to other vehicles and traffic lights
    - LocalPlanner functionality to navigate waypoints using PID controllers
    - LateralControl and LongitudinalControl PIDs
  * Added support for manual gear shifting
  * Added "role_name" attribute to actors to easily identify the "hero" vehicle
  * Changed traffic lights in Town03 to American style
  * Added new junction types with only stop signs
  * Updates to documentation and tutorials
  * Simulator now starts by default in windowed mode
  * CMake version required downgraded to 3.5 for better compatibility
  * Fixed waypoints height were all placed at zero height
  * Fixed actors in world.get_actors() missing parent actor
  * Fixed some vehicles losing their wheels after calling set_simulate_physics
  * Fixed bounding box of Lincoln MkZ
  * Several fixes and improvements to OpenDriveActor

## CARLA 0.9.1

  * New town: Town03
    - Created with Vector Zero's RoadRunner (including OpenDrive information of the road layout)
    - Bigger and more diverse
    - More road variety: multiple lanes and lane markings, curves at different angles, roundabout, elevation, tunnel
  * Lots of improvements to the Python API
    - Support for Python 3
    - Support for retrieving and changing lighting and weather conditions
    - Migrated Lidar sensor
    - Migrated image converter methods: Depth, LogarithmicDepth, and CityScapesPalette
    - Migrated IO methods for sensor data, "save_to_disk" available for PNG, JPEG, TIFF, and PLY
    - Added support for requesting the list of all the actors alive in the current world, `world.get_actors()`
    - `world.get_actors()` returns an `ActorList` object with `filter` functionality and lazy initialization of actors
    - Added collision event sensor, "sensor.other.collision", that triggers a callback on each collision to the actor it is attached to
    - Added lane detector sensor, "sensor.other.lane_detector", that detects lane invasion events
    - Added `carla.Map` and `carla.Waypoint` classes for querying info about the road layout
      - Added methods for converting and saving the map as OpenDrive format
      - Added `map.get_spawn_points()` to retrieve the recommended spawn points for vehicles
      - Added `map.get_waypoint(location)` to query the nearest waypoint
      - Added `map.generate_waypoints(distance)` to generate waypoints all over the map at an approximated distance
      - Added `map.get_topology()` for getting a list the tuples of waypoints that define the edges of the road graph
      - Added `waypoint.next(distance)` to retrieve the list of the waypoints at a distance that can be driven from this waypoint
    - Added `parent` attributes to actors, not None if the actor is attached to another actor
    - Added `semantic_tags` to actors containing the list of tags of all of its components
    - Added methods for retrieving velocity and acceleration of actors
    - Added function to enable/disable simulating physics on an actor, `actor.set_simulate_physics(enabled=True)`
    - Added bounding boxes to vehicles, `vehicle.bounding_box` property
    - Exposed last control applied to vehicles, `vehicle.get_vehicle_control()`
    - Added a "tick" message containing info of all the actors in the scene
      - Executed in the background and cached
      - Added `world.wait_for_tick()` for blocking the current thread until a "tick" message is received
      - Added `world.on_tick(callback)` for executing a callback asynchronously each time a "tick" message is received
      - These methods return/pass a `carla.Timestamp` object containing, frame count, delta time of last tick, global simulation time, and OS timestamp
      - Methods retrieving actor's info, e.g. `actor.get_transform()`, don't need to connect with the simulator, which makes these calls quite cheap
    - Allow drawing debug shapes from Python: points, lines, arrows, boxes, and strings (`world.debug.draw_*`)
    - Added id (id of current episode) and map name to `carla.World`
    - Exposed traffic lights and signs as actors. Traffic lights have a specialized actor class that has the traffic light state (red, green, yellow) as property
    - Added methods for accessing and modifying individual items in `carla.Image` (pixels) and `carla.LidarMeasurement` (locations)
    - Added `carla.Vector3D` for (x, y, z) objects that are not a `carla.Location`
    - Removed `client.ping()`, `client.get_server_version()` accomplishes the same
    - Renamed `contains_X()` methods to `has_X()`
    - Changed `client.set_timeout(seconds)` to use seconds (float) instead of milliseconds
    - Allow iterating attributes of an Actor's Blueprint
    - Fixed wildcard filtering issues, now "vehicle.*" or "*bmw*" patterns work too
    - Fixed `actor.set_transform()` broken for attached actors
  * More Python example scripts and improved the present ones
    - Now all the scripts use the list of recommended spawn points for each map
    - Renamed "example.py" to "tutorial.py", and updated it with latest changes in API
    - Added timeout to the examples
    - "manual_control.py" performance has been improved while having more measurements
    - "manual_control.py" now has options to change camera type and position
    - "manual_control.py" now has options to iterate weather presets
    - "manual_control.py" now has a fancier HUD with lots of info, and F1 key binding to remove it
    - Added "dynamic_weather.py" to change the weather in real-time (the one used in the video)
    - Added "spawn_npc.py" to quickly add a lot of NPC vehicles to the simulator
    - Added "spawn_npc.py --safe" to only add non-problematic vehicles
    - "vehicle_gallery.py" also got some small fixes
  * Asset and content improvements
    - New vehicle: Lincoln MKZ 2017
    - Refactored weather system, parametrized to make it easier to use
    - Improved control of bikes and motorbikes, still not perfect but causes less accidents
    - Added building block generator system
    - Misc city assets: New building, tunnel columns, rail-road bridges, new textures, new urban props
    - Adjusted vehicle physics and center of mass
    - Adjusted the maximum distance culling for foliage
    - Adjusted pedestrian animations and scale issues (not yet available with new API though)
    - Improved map building blueprints, spline based asset repeaters, and wall building tools
    - Replaced uses of Unreal's Foliage system with standard static meshes to work around a visual bug in Linux systems
    - Fixed filenames too long when packing the project on Windows
    - Fixed "SplineMeshRepeater" loses its collider mesh from time to time
    - Standardized asset nomenclature
  * New system for road information based on OpenDrive format
    - Added new map classes for querying info about the road layout and topology
    - Added methods for finding closest point on the road
    - Added methods for generating and iterating waypoints based on the road layout
    - Added OpenDrive parser to convert OpenDrive files to our map data structures
  * Other miscellaneous improvements and fixes
    - Fixed single channel Lidar crash (by @cwecht)
    - Fixed command-line argument `-carla-settings` fails to load absolute paths (by @harlowja)
    - Added an option to command-line to change quality level when launching the simulator, `-quality-level=Low`
    - Added ROS bridge odometry message (by @ShepelIlya)
    - New lens distortion shader, sadly not yet integrated with our cameras :(
    - New Docker tutorial
    - Disabled texture streaming to avoid issue of textures not loading in scene captures
    - Adjusted scene capture camera gamma to 2.4
    - Fixed leaking objects in simulation when despawning a vehicle. Now Pawn's controller is destroyed too if necessary when destroying an Actor
    - Fixed overflow on platform time-stamp, now it uses `double`
    - Upgraded @rpclib to fix crash when client exits too fast (rpclib/PR#167)
    - Moved "PythonClient" inside deprecated folder to avoid confusion
    - Refactored sensor related code
      - New plugin system for sensors that simplifies adding sensors, mini-tutorial at #830
      - Compile-time dispatcher for sensors and serializers
  * Improvements to the streaming library
    - Added multi-streams for streaming simultaneously to multiple clients (used by the "tick" message)
    - Messages re-use allocated memory when possible
    - Allows unsubscribing from a stream
    - Fixed client receives interleaved sensor messages, some messages can be discarded if connection is too slow though
    - Fixed streaming client fails to connect in Windows
    - Fixed streaming client keeps trying to reconnect after destroying a sensor
  * Refactored client C++ API
    - Python GIL is released whenever possible to avoid blocking
    - Fixed deadlock when closing the simulator while a client is connected
    - Fixed crash on simulator shutdown if a client has connected at some point
    - Set methods are now sent async which greatly improves performance in the client-side
    - Vehicle control is cached and not sent if haven't changed
    - Suppressed exceptions in destructors
  * Other development improvements
    - Improved Linux Makefile, fine-grained targets to reduce compilation times in development
    - Workaround for "setup.py" to link against "libcarla_client.a" again (Linux only)
    - Added support for ".gtest" file, each line of this file is passed to GTest executables as arguments when running `make check` targets
    - Python eggs are also archived on Jenkins to easily get them without downloading the full package
    - Added uncrustify config file for formatting UE4 C++ code

## CARLA 0.9.0

  * Upgraded to Unreal Engine 4.19
  * Redesign of the networking architecture
    - Allows any number of clients to connect simultaneously
    - Now is possible to add and remove at any time any vehicle or camera
    - Now is possible to control any vehicle or camera
    - Now is possible to place cameras anywhere
    - Reduced to two ports instead of three
    - First port uses an RPC protocol based on [rpclib](http://rpclib.net/)
    - Second port is for the streaming of the sensor data
  * Redesign of the Python API
    - Actors and sensors are now exposed in the API and can be independently controlled
    - The Python module is built in C++, with significant performance gain in some operations
    - Many functionality haven't been ported yet, so expect a lot of things missing
  * Redesign of the build system to accommodate the changes in dependencies
    - Everything can be done now with the Makefile
    - For the moment only Linux is supported, sorry
  * Massive clean up of all unused assets
  * Some aesthetic fixes to the vehicles

## CARLA 0.8.4

  * Community contribution: ROS bridge by @laurent-george
  * New vehicle: Tesla Model 3
  * Added an option to _"CarlaSettings.ini"_ to disable bikes and motorbikes
  * Fixed missing collision of vehicles introduced in 0.8.3
  * Improved stability of bikes and motorbikes
  * Improved autopilot turning behaviour at intersections, now using front wheels positions as reference
  * Temporarily removed Kawasaki Ninja motorbikes because the model was having some stability issues

## CARLA 0.8.3

  * Added two-wheeled vehicles, 3 bicycles and 4 motorbikes
  * Several art optimizations (CARLA is now about 10% faster)
    - Improved the performance of vegetation assets, adjusted LOD and culling distance, set billboards where possible
    - Drastically reduced the number of polygons of the landscape while keeping the original shape
    - Removed some high-cost unnecessary assets
    - Remodelled Mustang and NissanMicra, now with less polygons and materials, better textures and LOD
    - Remodelled building SM_TerracedHouse_01, now with more polygons but less materials and better textures
  * CARLA releases include now a Dockerfile for building docker images
  * Change in HUD: replace "FPS" by "Simulation Step"
  * The current map name is now included in the scene description message sent to the client
  * Adapted "manual_control.py" and "view_start_positions.py" to use the map name sent by the simulator
  * Improved the vehicle spawning algorithm, now it tries to spawn as much cars as possible even if there are not enough spawn points
  * "Setup.sh" is now faster and accepts an argument to run multiple jobs in parallel
  * Fixed foliage distance culling using wrong distance in "Low Mode"
  * Fixed NissanMicra slightly turning left when driving straight

## CARLA 0.8.2

  * Revamped driving benchmark
    - Changed name from benchmark to driving benchmark
    - Fully Redesigned the architecture of the module
    - Added a lot more documentation
    - Now you can stop and resume the benchmarks you run
  * Rolled back vehicle's location to the pivot of the mesh instead of the center of the bounding box
  * Added relative transform of the vehicle's bounding box to the measurements, player and non-players
  * Added "frame number" to each sensor measurement so it is possible to sync all the measurements based on the frame they are produced
  * Improved vehicle spawner to better handle spawning failures
  * Walkers use now a closer angle to detect vehicles, so they don't stop moving if a car passes nearby
  * Fixed lighting artefact causing the road to change its brightness depending on the distance to the camera
  * Fixed captured images overexposed in Low mode
  * Fixed illegal character in asset name
  * Fixed editing sun azimuth angle in CarlaWeadther.ini had no effect
  * Fixed crash when using a non-standard image size in DirectX (Windows)
  * Fixed issue with using multiple "SceneCaptureToDiskCamera"

## CARLA 0.8.1

  * New Python example for visualizing the player start positions
  * Fixed box extent of non-player agents was sent in centimeters instead of meters
  * Fixed speed limits were sent in km/h instead of m/s
  * Fixed issue in Volkswagen T2 wheels causing it to overturn

## CARLA 0.8.0

  * Upgraded to Unreal Engine 4.18
  * Created our own pedestrian 3D models free to use and distribute
  * Removed Epic's Automotive Materials dependencies
  * 360 Lidars support (similar to Velodyne HDL-32E or VLP-16) thanks to Anton Pechenko (Yandex)
    - Ray-cast based
    - Configurable settings
    - Added methods to save points to disk as PLY file
  * Added quality level settings
    - Low: low quality graphics, about 3 times faster with one camera
    - Epic: best quality (as before)
  * Measurements now use SI units
    - Locations:    m
    - Speed:        m/s
    - Acceleration: m/s^2
    - Collisions:   kg*m/s
    - Angles:       degrees
  * Added API methods to convert depth images to a point cloud
    - New method "image_converter.depth_to_local_point_cloud"
    - A supplementary image can be passed to attach colors to the points
    - New client example generates a point cloud in world coordinates
    - Added Transform class to Python API
  * Performance optimizations
    - Significant speed improvements in both Epic and Low modes
    - Fixed materials and improved shaders for roads, architecture, sidewalks, foliage, landscapes, cars, walkers, reflections, water
    - Execution of a set of Project and Engine parameters to improve performance (quality, vsync, AO, occlusion)
    - Generation of the road pieces using static meshes and actors instead of a single actor with instanced meshes
      - Improved performance since now is able to apply occlusion and draw distance
    - Images are captured asynchronously in the render thread
      - In asynchronous mode, images may arrive up to two frames later
      - In synchronous mode, game thread is blocked until images are ready
    - Blueprint code optimizations for vehicles, walkers, and splines
    - Added a way to configure different quality levels with culling distance and materials configuration
  * Refactored sensor related code to ease adding new sensors in the future
  * Added vehicle box extent to player measurements
  * Removed the player from the list of non-player agents
  * Adjusted bounding boxes to vehicles' height
  * Changed vehicles' center to match bounding box
  * Added autopilot mode to manual_control.py
  * Added quality level options to manual_control.py and client_example.py
  * Replaced background landscape and trees by a matte painting
  * Fixed road map generated some meshes twice
  * Small improvements to Windows support
    - Fixed issues with the Makefile
    - Fixed asset names too long or containing special characters

## CARLA 0.7.1

  * New Python API module: Benchmark
    - Defines a set of tasks and conditions to test a certain agent
    - Contains a starting benchmark, CoRL2017
    - Contains Agent Class: Interface for benchmarking AIs
  * New Python API module: Basic Planner (Temporary Hack)
    - Provide routes for the agent
    - Contains AStar module to find the shortest route
  * Other Python API improvements
    - Converter class to convert between Unreal world and map units
    - Metrics module to summarize benchmark results
  * Send vehicle's roll, pitch, and yaw to client (orientation is now deprecated)
  * New RoutePlanner class for assigning fixed routes to autopilot (IntersectionEntrance has been removed)
  * Create a random engine for each vehicle, which greatly improves repeatability
  * Add option to skip content download in Setup.sh
  * Few small fixes to the city assets

## CARLA 0.7.0

  * New Python client API
    - Cleaner and more robust
    - Compatible with Python 2 and 3
    - Improved exception handling
    - Improved examples
    - Included methods for parsing the images
    - Better documentation
    - Protocol: renamed "ai_control" to "autopilot_control"
    - Merged testing client
    - Added the maps for both cities, the client can now access the car position within the lane
  * Make CARLA start without client by default
  * Added wind effect to some trees and plants
  * Improvements to the existing weather presets
  * Build script: skip content download if up-to-date

## CARLA 0.6.0

  * Included Unreal project and reorganised folders
  * Enabled semantic segmentation by default
  * Added Felipe's Python client
  * New build system (Linux only)
  * Few fixes to city assets

## CARLA 0.5.4

  * Added command-line parameter -carla-no-hud
  * Remove override gamma from weather settings
  * Fixed issue road map generation hangs cooking command
  * Organise Python client and make sample script
  * Rename maps
    - CARLA_ORIGIN_0 --> Town02
    - CARLA_ORIGIN_1 --> Town01
  * Fixed Carla-Cola machine falling at begin play

## CARLA 0.5.3

  * Fixed issues with weather
  * Fixed missing building

## CARLA 0.5.2

  * Autopilot mode has been removed, now server sends AI control together with measurements every frame
  * State and position of traffic lights and signs are now included in the measurements too
  * Added a python console client
  * Fixed crash when client sends an invalid player start
  * Fixed some issues with the dynamic weather not looking as it used to do
  * Fixed some collision boxes missing

## CARLA 0.5.1

  * Fixed issue server was destroyed on every reset, closing the connection
  * Fixed issue agent servers connect too late
  * Improvements to the python client
  * Added python client test suite for testing the release
  * Added image converter
  * Fixed missing floor on CARLA_ORIGIN_0
  * Changed sidewalk texture
  * Improvements on the physics of some vehicles
  * More props and decals added to the cities

## CARLA 0.5.0

  * Upgraded to Unreal Engine 4.17
    - Fixes memory leaks
    - Fixes crashes with C++ std classes
  * Redesigned CarlaServer
    - Faster, avoids unnecessary copies
    - Sends images as raw data (no compression)
    - Supports synchronous and asynchronous mode
    - Networking operation have a time-out
    - Synchronous methods have a time-out
    - Pure C interface for better compatibility
    - Unit tests with GoogleTest
  * New server-client protocol
    - Upgraded to proto3
    - Supports repeated fields
    - Optionally send information about all dynamic agents in the scene
    - Now sends transforms instead of locations only
    - Autopilot mode added to control
  * New build system to avoid linkage issues
  * Added autopilot mode
  * Added an on-board camera to the car
  * Added traffic lights and speed limit to player state
  * Added player pawn selection to config file
  * Improved blueprint interface of the C++ classes
  * Some performance improvements to vehicle controllers
  * Fix issues with depth material in Windows
  * Fix issues with random engine not being available for vehicles
  * Fixed issue that compiling a release hang when saving the road map
  * Added more content; 7 vehicles, 30 pedestrians, many decals and props
  * Randomized pedestrian clothing
  * Many improvements and fixes to the city levels and assets
  * Added sub-surface scattering to vegetation
  * Added key binding to change weather during play
  * Added key binding to toggle autopilot mode
  * Added a second camera to the player

## CARLA 0.4.6

  * Add weather presets specific for each level
  * Some map fixes, adjust weather presets specific for each level
  * Fixed regression that some walkers may go at extremely slow and fast speeds

## CARLA 0.4.5

  * Add random seeds to config file
  * Improve logging
  * Removed rotation of map CARLA_ORIGIN_1

## CARLA 0.4.4

  * Fixed regression walkers despawning when stopping after seeing a car
  * Changed, collision is only registered if player moves faster than 1 km/h
  * Fixed issue walkers resume movement after sensing nothing, but the car is still there sometimes
  * Few improvements to the city assets

## CARLA 0.4.3

  * Fixed issue with reward, intersect other lane wasn't sent to the client
  * Improvements to the AI of other vehicles, and how they detect pedestrians
  * Improvements to the AI of the pedestrians, trying to avoid slightly better the cars
  * Made roads collision channel WorldStatic
  * Tune several vehicles' physics and engine
  * Fixed issue with vehicles bouncing back after hitting a pedestrian
  * Add bigger box to pedestrians to avoid accidents
  * Make vehicles spawn in order instead of randomly

## CARLA 0.4.2

  * Fixed issues with the server-client protocol
  * More improvements to the AI of other vehicles, now they barely crash
  * Improved the physics of some vehicles
  * Tweak the city for better AI of other vehicles

## CARLA 0.4.1

  * Improved AI of other vehicles, still needs some adjustment, but now they crash much less
  * Fixed wrong semantic segmentation label of the poles of traffic lights and signs
  * Added randomized vehicle license plates
  * Few improvements to the city assets

## CARLA 0.4.0

  * Made vehicle input more realistic, now reverse and brake use different input
  * Changed server-client protocol
    - CarlaSettings.ini is sent for every new episode
    - Control is extended with brake, reverse and handbrake
  * Set a clearer hierarchy for loading settings files
  * Made camera post-process settings able to change depending on the weather
  * Added basic functionality for NPC vehicles
  * Some improvements to the walker spawner
  * Generate road map metadata on save
  * Added command-line switch -carla-no-networking
  * Improved verbosity control of CarlaServer
  * Fixed issue with server that two threads used 100% CPU
  * Fixed issue with the attachment of the main camera to the player
  * Fixed issues with CarlaServer interface with Unreal, does not use STL containers anymore
  * Fixed issue with server not running below 30 fps at fixed frame rate, added physics sub-stepping
  * Fixed issues with some weather settings
  * Added randomized pedestrians with their AI and animations
  * Added other vehicles with their AI and physics
  * Added traffic lights and signs
  * Tweaked capture image to look similar to main camera
  * Changed car input to match settings in plugin
  * General improvements to levels and assets

## CARLA 0.3.0

  * Added basic dynamic weather functionality
    - Weather and sun light can be changed during game
    - Presets stored in config file CarlaWeather.ini
    - Added some presets for dynamic weather
  * Add basic functionality to spawn pedestrians
  * Split road meshes for intersections and turns for better precission of the road map
  * Better debug for road map
  * Implemented collision count for other cars and pedestrians
  * Command line argument -carla-settings now accepts relative paths
  * Improved performance when semantic segmentation is disabled
  * Improved tagger system
  * Implemented nav-mesh and spawn points for pedestrians
  * Added new cars
  * Added dynamic street lights
  * General improvements to levels and assets
  * Make the car jump

## CARLA 0.2.4

  * Fixed serialization of road map resulting in a huge map size
  * Some optimizations in the vegetation
  * Implemented more LODS

## CARLA 0.2.3

  * Fixed rounding errors in HUD (100% was shown as 99%, 30 FPS as 29 FPS)
  * Fixed crash when player goes out of road map
  * Fixed several issues related to the transform of the road map (wasn't working in CARLA_ORIGIN_1)
  * Make custom depth pass disable by default (semantic segmentation won't work by default)
  * Fixed road width in T-intersections
  * Implement road LOD
  * Fixed missing assets

## CARLA 0.2.2

  * Implemented signals for off-road and opposite lane invasion
  * Fixed linking issues (use Unreal's libpng)
  * Fixed memory leak in PNG compression
  * Added boundaries to the map
  * Several fixes in the map content

## CARLA 0.2.1

  * Fixed the memory leak related to protobuf issues
  * Fixed color shift in semantic segmentation and depth
  * Added in-game timestamp (now sending both OS and in-game)

## CARLA 0.2.0

  * Fixed Depth issues
  * Fixed random crash due to an invalid player start position
  * Added semantic segmentation
  * Changed codification to PNG
  * Camera configuration through config INI file

## CARLA 0.1.1

  * Added build system for Windows and Linux
  * Added more content

## CARLA 0.1.0

  * Added basic functionality
