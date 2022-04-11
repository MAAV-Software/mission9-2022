# Setup MAAV Software on Mac

By the end of this tutorial, you should have the software repository on your system, and be able to start a virtual system (Docker container) to run and simulate the vehicle behavior.

## Install Required Software

You will need Docker, which is used to create images or "snapshots" of virtual systems and run them in containers. Install Docker using by following the documentation given at the following link:

[Docker Installation Instructions](https://runnable.com/docker/install-docker-on-macos)

You can test if Docker is setup correctly by running this command in your mac terminal:

```
docker
```

## Clone the Software Code

In your terminal, navigate to a folder you want to put the code. Run the following command to download the code from Github:

```
git clone <URL copied from Github>
```

You can either clone via HTTPS (no login required for `clone`, `fetch`, or `pull` but must get a [personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) for `push`), or [SSH](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) (more secure, uses private SSH key created on your system that matches with your public key on your Github account, can have an optional password, and is checked for any remote Git operation). We recommend SSH, but you can use whichever is easier for you.

## Build the Docker Image

For more information about using Docker, [see the Docker doc page.](./Docker.md)

Inside the root folder of the software code you just cloned, run this command:

```
sudo docker build -f mac.Dockerfile -t maav-mission9 .
```

**Note the '.' at the end!**

The Docker image should build successfully, but let us know if there's any issues or bugs you run into.

## Start the Docker Container and Gazebo

Start the Docker container using this command:

```
sudo docker run -p 6080:80 -v $PWD:/mission9-2021:rw maav-mission9
```

After you started the docker container, open a web browser and type [127.0.0.1:6080](http://127.0.0.1:6080) into the search bar and hit enter. If everything works correctly, you should see a linux desktop. This will be your way of interacting with the docker container!

Once inside the Docker container, we need to install a bunch of dependencies. This step takes a while and is a hastle. Eventually this step will be done automatically when the docker container is built for the first time. For now... Run this command:

```
/scripts/commands.sh
```

After everything gets installed, test if everything installed properly by running:

```
cd /px4_sitl/PX4-Autopilot && make px4_sitl -j1 gazebo
```

Note that this will compile some necessary binaries (which only happens one time until the Docker container is shut down) before starting PX4-Autopilot and Gazebo.

You can also debug opening Gazebo without PX4 running (which is useful for testing GUI settings) by using this command:

```
gazebo --verbose
```

Happy coding!
