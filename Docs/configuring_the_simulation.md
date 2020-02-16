<h1>Configuring the simulation</h1>

Before you start running your own experiments there are few details to take into
account at the time of configuring your simulation. In this document we cover
the most important ones.

Changing the map
----------------

The map can be changed from the Python API with

```py
world = client.load_world('Town01')
```

this creates an empty world with default settings. The list of currently
available maps can be retrieved with

```py
print(client.get_available_maps())
```

To reload the world using the current active map, use

```py
world = client.reload_world()
```

Graphics Quality
----------------

<h4>Vulkan vs OpenGL</h4>

Vulkan _(if installed)_ is the default graphics API used by Unreal Engine and CARLA on Linux.  
It consumes more memory but performs faster.  
On the other hand, OpenGL is less memory consuming but performs slower than Vulkan.

!!! note
    Vulkan is an experimental build so it may have some bugs when running the simulator.

OpenGL API can be selected with the flag `-opengl`.

```sh
> ./CarlaUE4.sh -opengl
```

<h4>Quality levels</h4>

Currently, there are two levels of quality, `Low` and `Epic` _(default)_. The image below shows
how the simulator has to be started with the appropiate flag in order to set a quality level
and the difference between qualities.

![](img/epic_quality_capture.png)  |  ![](img/low_quality_capture.png)
:-------------------------:|:-------------------------:
`./CarlaUE4.sh -quality-level=Epic`  |  `./CarlaUE4.sh -quality-level=Low`

**Low mode runs significantly faster**, ideal for users that don't rely on quality precision.

!!! important
    The issue that made quality levels show an abnormal whiteness has been fixed. If the problem persists delete `GameUserSettings.ini` as it is saving the previous settings. It will be generated again in the next run. __Ubuntu path:__ `  ~/.config/Epic/CarlaUE4/Saved/Config/LinuxNoEditor/` __Windows path:__ `<Package folder>\WindowsNoEditor\CarlaUE4\Saved\Config\WindowsNoEditor\`

Running off-screen
------------------

In Linux, you can force the simulator to run off-screen by setting the
environment variable `DISPLAY` to empty

!!! important
    **DISPLAY= only works with OpenGL**<br>
    Unreal Engine currently crashes when Vulkan is used when running
    off-screen. Therefore the `-opengl` flag must be added to force the engine to
    use OpenGL instead. We hope that this issue is addressed by Epic in the near
    future.

```sh
# Linux
DISPLAY= ./CarlaUE4.sh -opengl
```

This launches the simulator without simulator window, of course you can still
connect to it normally and run the example scripts. Note that with this method,
in multi-GPU environments, it's not possible to select the GPU that the
simulator will use for rendering. To do so, follow the instruction in
[Running without display and selecting GPUs](carla_headless.md).

No-rendering mode
-----------------

It is possible to completely disable rendering in the simulator by enabling
_no-rendering mode_ in the world settings. This way is possible to simulate
traffic and road behaviours at very high frequencies without the rendering
overhead. Note that in this mode, cameras and other GPU-based sensors return
empty data.

```py
settings = world.get_settings()
settings.no_rendering_mode = True
world.apply_settings(settings)
```

Fixed time-step
---------------

The time-step is the _simulation-time_ elapsed between two steps of the
simulation. In video-games, this _simulation-time_ is almost always adjusted to
real time for better realism. This is achieved by having a **variable
time-step** that adjusts the simulation to keep up with real-time. In
simulations however, it is better to detach the _simulation-time_ from
real-time, and let the simulation run as fast as possible using a **fixed
time-step**. Doing so, we are not only able to simulate longer periods in less
time, but also gain repeatability by reducing the float-point arithmetic errors
that a variable time-step introduces.

CARLA can be run in both modes.

<h4>Variable time-step</h4>

The simulation tries to keep up with real-time. To do so, the time-step is
slightly adjusted each update. Simulations are not repeatable. By default, the
simulator starts in this mode, but it can be re-enabled if changed with

```py
settings = world.get_settings()
settings.fixed_delta_seconds = None
world.apply_settings(settings)
```

<h4>Fixed time-step</h4>

The simulation runs as fast as possible, simulating the same time increment on
each step. To enable this mode set a fixed delta seconds in the world settings.
For instance, to run the simulation at a fixed time-step of 0.05 seconds (20
FPS) apply the following settings

```py
settings = world.get_settings()
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
```

!!! important
    **Do not decrease the frame-rate below 10 FPS.**<br>
    Our settings are adjusted to clamp the physics engine to a minimum of 10
    FPS. If the game tick falls below this, the physics engine will still
    simulate 10 FPS. In that case, things dependent on the game's delta time are
    no longer in sync with the physics engine.
    Ref. [#695](https://github.com/carla-simulator/carla/issues/695)

Synchronous mode
----------------

!!! important
    **Always run the simulator at fixed time-step when using the synchronous
    mode**. Otherwise the physics engine will try to recompute at once all the
    time spent waiting for the client, this usually results in inconsistent or
    not very realistic physics.

The client-simulator communication can be synchronized by using the _synchronous
mode_. When the synchronous mode is enabled, the simulation is halted each
update until a _tick_ message is received.

This is very useful when dealing with slow client applications, as the simulator
waits until the client is ready to continue. This mode can also be used to
synchronize data among sensors by waiting until all the data is received. Note
that data coming from GPU-based sensors (cameras) is usually generated with a
delay of a couple of frames respect to data coming from CPU-based sensors.

The synchronous mode can be enabled at any time in the world settings.

```py
# Example: Synchronizing a camera with synchronous mode.

settings = world.get_settings()
settings.synchronous_mode = True
world.apply_settings(settings)

camera = world.spawn_actor(blueprint, transform)
image_queue = queue.Queue()
camera.listen(image_queue.put)

while True:
    world.tick()
    image = image_queue.get()
```

For a more complex scenario synchronizing data from several sensors, take a look
at the example [synchronous_mode.py][syncmodelink].

[syncmodelink]: https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/synchronous_mode.py

Command-line options
--------------------------

!!! important
    Some of the command-line options are not available in `Linux` due to the "Shipping" build.
    Therefore, the use of [`config.py`][configlink] script is needed to configure the simulation.

[configlink]: https://github.com/carla-simulator/carla/blob/master/PythonAPI/util/config.py

Some configuration examples:

```sh
> ./config.py --no-rendering      # Disable rendering
> ./config.py --map Town05        # Change map
> ./config.py --weather ClearNoon # Change weather
...
```

To check all the available configurations, run the following command:

```sh
> ./config.py --help
```

Commands directly available:

  * `-carla-rpc-port=N` Listen for client connections at port N, streaming port is set to N+1 by default.
  * `-carla-streaming-port=N` Specify the port for sensor data streaming, use 0 to get a random unused port.
  * `-quality-level={Low,Epic}` Change graphics quality level.
  * [Full list of UE4 command-line arguments][ue4clilink] (note that many of these won't work in the release version).

Example:

```sh
> ./CarlaUE4.sh -carla-rpc-port=3000
```

[ue4clilink]: https://docs.unrealengine.com/en-US/Programming/Basics/CommandLineArguments
