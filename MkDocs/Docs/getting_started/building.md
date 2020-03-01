!!! important
    Building SUMMIT is only necessary if you wish to edit SUMMIT. If you only wish to use the simulator, the [setup steps](/getting_started/setting_up) are sufficient.

<h1>Building SUMMIT</h1>

The build instructions for SUMMIT are exactly the same as those in CARLA. 

To build SUMMIT, follow exactly the [build instructions for CARLA](https://carla.readthedocs.io/en/latest/how_to_build_on_linux/), with some additional steps:

  * Clone and use the [SUMMIT repository](https://github.com/AdaCompNUS/summit) instead of the CARLA repository.
  * **Before building**, install [ccache](https://ccache.dev/): `sudo apt install ccache`.
  * **After getting assets**, copy the following SUMMIT specific assets into the assets folder, overriding any existing files:
    * Copy `<summit_root>/CustomAssets/EmptyMap.umap` to `<summit_root>/Unreal/CarlaUE4/Content/Carla/Maps/TestMaps/EmptyMap.umap`
    * Copy `<summit_root>/CustomAssets/EmptyMap_BuiltData.umap` to `<summit_root>/Unreal/CarlaUE4/Content/Carla/Maps/TestMaps/EmptyMap_BuiltData.umap`
    * Copy `<summit_root>/CustomAssets/M_Tile.uasset` to `<summit_root>/Unreal/CarlaUE4/Content/Carla/Static/GenericMaterials/Ground/M_Tile.uasset`
  

