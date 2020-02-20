!!! important
    SUMMIT comes with a [set of maps ready for use](/references/summit_map_library.md). This page is required only if you wish to use additional maps.

# Map data sources
For a given map, SUMMIT requires both a corresponding OSM file and SUMO network, which provide the respective information:

  * OSM file: Landmark data for spawning landmark objects, and geographical bounds for satellite imagery.
  * SUMO network: Road contexts.

Note that sidewalk information is not required at all, since SUMMIT automatically calculates the sidewalks as boundaries along roads.

!!! note
    When working with real-world maps, the SUMO networks are typically generated from the OSM file. It is thus common that both the OSM file and SUMO network contains the same road information. The road information in the OSM file is simply unused after producing the SUMO network.

    While it is possible to use the road information in the OSM file directly, we choose to use a SUMO network as it provides a much more detailed representation to work with. In addition, SUMO comes with a flexible suite of tools to aid making fine adjustments to the SUMO networks.

!!! important
    SUMMIT uses the convention of storing all map data inside the `<summit-root>/Data/` folder. All data files pertaining to the same map should be given the same name with different extensions (e.g. `meskel_square.osm`, `meskel_square.net.xml`). In addition, underscores should be in the file name to separate words in the map name.


# Obtaining OSM files
To obtain high-quality real-world OSM files, we recommend using [OpenStreetMap's database](https://www.openstreetmap.org/). Simply zoom into the are of interest, and export the map to `<summit_root>/Data/<map_name>.osm`, giving your map a reasonable name.

You may wish to preprocess the OSM files using [JOSM](https://josm.openstreetmap.de/) to remove road information unwanted from the simulation, such as service roads and footbridges.

!!! note
    In SUMMIT, we do not impose any restrictions on the source of OSM files. You are free to use any OSM file, even those produced by yourself.

# Obtaining SUMO networks
A SUMO network can be automatically obtained from an OSM file by using SUMO's [NETCONVERT utility](https://sumo.dlr.de/docs/NETCONVERT.html). This assumes that you have already setup the SUMO tools, as recommended in the [requirements section](/getting_started/quick_start/#requirements). The SUMO network should then be stored at `<summit_root>/Data/<map_name>.net.xml`, where `<map_name>` follows from that of the OSM file.

We provide an example script at `<summit_root>/Scripts/osm2sumo.sh` to convert an OSM file into a SUMO network. 

You are recommended to postprocess the SUMO network using [SUMO's NETEDIT](https://sumo.dlr.de/docs/NETEDIT.html) after the conversion. The conversion process is not perfect, and we highly recommend postprocessing the SUMO network to eliminate any ambiguities.

!!! important
    Similar to the OSM files, you are free to use any source of SUMO networks, including those produce by yourself. There are some restrictions however:
      
      * All lanes in SUMMIT are assumed to have a fixed width of 4.0 meters.
      * Bounds for OSM file and corresponding SUMO network must line up, if not landmarks and satellite imagery may spawn incorrectly.

!!! note
    For SUMMIT's built in maps, we have done some postprocessing after the conversion, so it will be different from what is produced by simply calling the script.

# (Optional) Specifying simulation bounds
For a given map, SUMMIT's traffic simulation script simulates traffic in a user defined bounds, specified in `<summit_root>/Data/<map_name>.sim_bounds`. We recommend specifying a suitable bounds if you are using a custom map, in order to use SUMMIT's built in traffic simulation script.

Bounds in a `.sim_bounds` file are specified with the `min_x,min_y` on the first line, and `max_x,max_y` on the second line. For example, in `<summit_root>/Data/meskel_square.sim_bounds`:

    350,300
    550,500

These are in terms of CARLA coordinates. To find a suitable bounds for your map, you can use SUMO's NETEDIT tool as follows:

  1. Open the map's SUMO network in SUMO's NETEDIT tool.
  2. Find a suitable rectangular bound and note down the corner coordinates given by NETEDIT. These are in SUMO coordinates.
  3. Convert each SUMO coordinates into CARLA coordinates by swapping the x and y values.
  4. Take the minimum and maximum of x and y among the converted CARLA coordinates.

# (Optional) Caching map object meshes
Often in SUMMIT, these mesh of map objects are calculated in order to spawn them dynamically in the simulator. The meshes can be calculated and saved beforehand to speed this process up.

We provide a utility script at `<summit_root>/Scripts/extract_meshes.py` that calculates the meshes of various map objects for a given map, assuming that both the OSM file and SUMO network have been prepared. The meshes are stored at `<summit_root>/Data/` with the respective names.

As an example, running the following:

```bash
extract_meshes.py --dataset meskel_square
```
    
uses `meskel_square.osm` (OSM file) and `meskel_square.net.xml` (SUMO network) to produce the following files in `<summit_root>/Data/`:

  * `meskel_square.network.wkt`: Road mesh.
  * `meskel_square.roadmark.wkt`: Roadmarks mesh.
  * `meskel_square.sidewalk.wkt`: Sidewalk mesh.
  * `meskel_square.landmarks/*.landmark.wkt`: Landmarks' meshes.

# (Optional) Downloading satellite imagery
To download satellite imagery for your map, ensure that you have your map's OSM file located at `<summit_root>/Data/<map_name>.osm`, so that SUMMIT can locate the geographic bounds. Then, run the following utility script:

```bash
<summit_root>/Scripts/download_imagery.py --dataset <map_name>
```

This downloads imagery that stored as tiles in `<summit_root/Data/imagery/` in [Slippy Map format](https://wiki.openstreetmap.org/wiki/Slippy_Map).
