# MAAV Mission 9 Repo - 2021

## Introduction
This README document is a work-in-progress. We will populate this more later.

## Building Docker Image
We will be using Docker to simulate the environment that will be on the
vehicle's computing hardware. To setup the Docker image (a "snapshot" of the
computer at a specific time and state) for this repo, follow these
instructions.

Install [Docker Desktop](https://docs.docker.com/get-docker/)
(which should be compatible with all systems) using the
normal install instructions, and then setup the Docker image using the
instructions below.

(Note for Windows users: We recommend using WSL with Ubuntu to run the Docker
commands for developing in this repo. The command syntax is more compatible
this way.)

(Let MAAV Software leadership know if you have trouble with Docker install.)

### Option 1: Build the Docker image from source
Clone this repository, and run this command:

```
$ docker build -t maav-mission9 . # Note the '.' at the end!
```

Once this is complete, you should see the Docker image in your images list:

```
$ docker images
REPOSITORY          TAG             IMAGE ID            CREATED         SIZE
maav-mission9       latest          <hex-hash>          30 minutes ago  1.5GB
```

You can now spin up a docker container as follows:

```
$ docker-compose run --rm maav-mission9
```

### Option 2: Download a Docker image file and load it
TODO. Once the Docker testng enironment is setup, we will have a download link.

Download the Docker image above, and load it using this command:
```
$ docker load -i maav-mission9-2-6-2021.img
```

### Run the MAAV Docker Container
This is an alternative to spinning up the Docker container (though `docker-compose` detailed above is preferred). To run it, us this command:

TODO: There may be other flags/arguments to add later.
```
$ docker run --rm -it maav-mission9
# --rm: Deletes the Docker container upon exit
# -it: Interactive mode with a cleaner look
```

### Clean up MAAV Docker Artifacts
If you need to clean up your Docker environment from any MAAV artifacts, here
are some useful commands to do so:

List all Docker images on your system:
```
$ docker images
```
Delete a Docker image:
```
$ docker image rm <REPOSITORY|IMAGE ID>
```
List all Docker containers on your system:
```
$ docker ps -a
```
Delete a Docker container:
```
$ docker rm <CONTAINER ID|NAME>
```
