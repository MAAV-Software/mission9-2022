# Setup MAAV Software on Ubuntu / Native Linux

By the end of this tutorial, you should have the software repository on your system, and be able to start a virtual system (Docker container) to run and simulate the vehicle behavior.

## Install Required Software

You will need Docker, which is used to create images or "snapshots" of virtual systems and run them in containers. Install Docker using this command:
```
sudo apt-get install docker
```

You can test if Docker is setup correctly by running this command:
```
docker
```

## Clone the Software Code

Use this command in WSL:
```
git clone <URL copied from Github>
```

You can either clone via HTTPS (no login required for `clone`, `fetch`, or `pull` but must login with username/password for `push`), or [SSH](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) (more secure, uses private SSH key created on your system that matches with your public key on your Github account, can have an optional password, and is checked for any remote Git operation). We recommend SSH, but you can use whichever is easier for you.

## Build the Docker Image

Inside the root folder of the software code you just cloned, run this command:
```
docker build -t maav-mission9 .
```
**Note the '.' at the end!**

The Docker image should build successfully, but let us know if there's any issues or bugs you run into.

TODO DISPLAY ENV VAR?
```
export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0"
```

## Start the Docker Container and Gazebo

In order to fix the Linux GUI for Gazebo, run this command:
```
xhost +
```

Start the Docker container using this command:
```
docker-compose run --rm linux
```

Once inside the Docker container, you can run a test to see if PX4 and Gazebo will start and display the simulator, as expected. Run this command:
```
cd /px4_sitl/PX4-Autopilot && make px4_sitl -j1 gazebo
```
Note that this will compile some necessary binaries (which only happens one time until the Docker container is shut down) before starting PX4-Autopilot and Gazebo.

You can also debug opening Gazebo without PX4 running (which is useful for testing GUI settings) by using this command:
```
gazebo --verbose
```
