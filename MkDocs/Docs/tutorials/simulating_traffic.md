SUMMIT comes with a built in script to simulate traffic on a map, located at `<summit_root>/PythonAPI/examples/gamma_crowd.py`. It makes use of the recent [GAMMA](https://arxiv.org/abs/1906.01566) prediction model to simulate heterogeneous agent behavior with sophisticated and unregulated behaviors.

To use the script, ensure that you have done the following:

  * [Specified simulation bounds](/tutorials/preparing_maps/#optional-specifying-simulation-bounds). This is required, since the script needs to know where to bound the agents. It only spawns agents within the specified bounds, and actively remove agents that go out of bounds.
  * [Spawned the map objects](/tutorials/loading_and_spawning_maps). 

You can then call `gamma_crowd.py --dataset <map_name>` to simulate the traffic. A list of all arguments are available with the `--help` flag, and it is recommended to read through the options as you may find them useful for your needs.

