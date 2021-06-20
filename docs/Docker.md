# Docker Additional Information / Commands
This page will tell you how to get the MAAV Software image, start a Docker
container, and clean up any artifacts from Docker (leftover images, containers).

## Setup the Docker Image

You can either build the image from source (recommended), or download an image
file and import it into Docker.

Once this is complete, you should see the Docker image in your images list:

```
$ docker images
REPOSITORY          TAG             IMAGE ID            CREATED         SIZE
maav-mission9       latest          <hex-hash>          30 minutes ago  1.5GB
```

### Option 1: Build the Docker image from source (recommended)

Run this command:
```
docker build -t maav-mission9 .
```
**Note the '.' at the end!**

### Option 2: Download the Docker image file and load it

TODO. Once the Docker testing enironment is setup, we will have a download link.

Download the Docker image above, and load it using this command:
```
docker load -i maav-mission9-2-6-2021.img
```

## Start a Container with the MAAV Docker Image

Depending on your host operating system, run this command:
```
docker-compose run --rm <wsl|mac|linux>
```

The Docker container will automatically delete itself after you exit. Any changes
you make OUTSIDE of the `/maav-mission9` folder will be discarded. This is because
the MAAV software folder on your host operating system is configured to exist inside
the Docker container as an "external drive".

## Clean up MAAV Docker Artifacts
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

