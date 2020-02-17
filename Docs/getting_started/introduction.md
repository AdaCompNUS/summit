# SUMMIT
SUMMIT (**S**imulator for **U**rban Driving **i**n **M**assive **Mi**xed **T**raffic) is an open-source simulator with a focus on generating high-fidelity, interactive data for unregulated, dense urban traffic on complex real-world maps. It works with map data in the form of [OSM files](https://wiki.openstreetmap.org/wiki/OSM_file_formats) and [SUMO networks](https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html) to generate crowds of heterogeneous traffic agents with sophisticated and realistic unregulated behaviors. SUMMIT can work with map data fetched from online sources, providing a virtually unlimited source of complex environments.

SUMMIT additionally exposes interfaces to interact with the contextual information provided by the map data. It also provides a robust suite of geometric utilities for use by external programs. Through these, SUMMIT aims to enablie applications in a wide range of fields such as perception, vehicle control and planning, end-to-end learning, etc. 

SUMMIT was built upon the very successful [CARLA](http://carla.org/). Updates to CARLA are constantly merged into SUMMIT to ensure that users of SUMMIT have access to the high quality of work endowed by CARLA, such as its high-fidelity physics, rendering and sensors; however, it should be noted that not all components of SUMMIT work with those from CARLA, as they were designed for a different use case.

# The simulator
SUMMIT adds on top of CARLA a set of capabilities to enable the simulation of sophisticated traffic behaviors on real-world maps:

  * **Road contexts: ** In SUMMIT, the `SumoNetwork` interface is exposed to allow for easy interaction with road contexts, which are represented using [SUMO networks](https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html). It is optimized for fast spatial and topological queries.
  * **Sidewalk contexts: ** Sidewalk contexts are represented using a collection of oriented polygons. SUMMIT exposes the `Sidewalk` interface to easily interact with sidewalk contexts. It is optimized for fast spatial and topological queries.
  * **Geometric utilities: ** SUMMIT provides a range of utility classes to help with various geometric operations.
    * `OccupancyMap`: Manipulation for general 2D areas. Useful for exact collision detection. 
    * `AABBMap`: Manipulation for axis aligned bounding boxes. Useful for fast approximate collision detection. 
    * `SegmentsMap`: Maipulation for collections of 2D line segments. Primarily used to sample high quality uniformly distributed spawn points against road and sidewalk contexts.
  * **Procedural simulation:** Spawning a scenario is entirely procedural, and no recompilation the simulator is required for simulation on a new map. Mesh information for map objects such as roads, sidewalks, landmarks, and satellite imagery are sent from the client to the simulation server, where the objects are dynamically spawned.
  * **Crowd simulator:** SUMMIT provides a client side Python script that simulates unregulated traffic in the simulation. It is capable of generating dense crowds, involving vehicles on the road and pedestrians on the sidewalk, where agents operate interactively with one another. [GAMMA](https://arxiv.org/abs/1906.01566), a state-of-the-art traffic motion prediction model, is used to produce sophisticated and realistic behaviors in the simulated crowd.
