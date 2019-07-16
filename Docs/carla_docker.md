<h1>Running CARLA in a Docker </h1>



This tutorial is designed for:

  * People that want to run CARLA without needing to install all dependencies.
  * Recommended solution to run multiple CARLA servers and perform GPU mapping

This tutorial was tested in Ubuntu 16.04 and using NVIDIA 396.37 drivers.
This method requires a version of NVIDIA drivers >=390.


## Docker Installation

!!! note
    Docker requires sudo to run. Follow this guide to add users to the docker sudo
    group <https://docs.docker.com/install/linux/linux-postinstall/>

### Docker CE

For our tests we used the Docker CE version.
To install Docker CE we recommend using [this tutorial](https://docs.docker.com/install/linux/docker-ce/ubuntu/#extra-steps-for-aufs)

### NVIDIA-Docker2

To install nvidia-docker-2 we recommend using the "Quick Start"
section from the [nvidia-dockers github](https://github.com/NVIDIA/nvidia-docker).




## Getting it Running


Pull the CARLA image.

    docker pull carlasim/carla:version

For selecting a version, for instance, version 0.8.2 (stable), do:

    docker pull carlasim/carla:0.8.2



Running CARLA under docker:

    docker run -p 2000-2002:2000-2002 --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 carlasim/carla:0.8.4

The "-p 2000-2002:2000-2002" argument is to redirect host ports for the docker container.
Use "NVIDIA_VISIBLE_DEVICES=<gpu_number>" to select the GPU.

You can also pass parameters to the CARLA executable. With this you can chose the town and
select the port that is going to be used:

    docker run -p 2000-2002:2000-2002 --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 carlasim/carla:0.8.4 /bin/bash CarlaUE4.sh  < Your list of parameters >

At the list of parameters do not forget to add "-world-port=<port_number>" so that CARLA runs on server mode
listening to the "<port_number>"


