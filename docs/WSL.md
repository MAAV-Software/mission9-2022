# Setup MAAV Software on Windows / WSL

By the end of this tutorial, you should have the software repository on your system, and be able to start a virtual system (Docker container) to run and simulate the vehicle behavior.

## Install Required Software

If you don't have WSL (Windows Subsystem for Linux) already installed on your machine, [follow these instructions.](https://eecs280staff.github.io/p1-stats/setup_wsl.html)

You will need vcXsrv, a tool to show "remote displays" in Windows. [Download vcXsrv here.](https://sourceforge.net/projects/vcxsrv/)

You will also need Docker Desktop, which is used to create images or "snapshots" of virtual systems and run them in containers. [Download Docker Desktop here.](https://docs.docker.com/get-docker/) There will be some additional settings config to make Docker work with WSL, but you can test if Docker is setup correctly by running this command in WSL:
```
docker
```

If the command doesn't work, check your Docker Desktop settings or make sure Docker Desktop is still running.

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

## Setup vcXsrv for Gazebo Simulation

This step is NOT a one-time setup. You will need to do this everytime you start working on MAAV software simulation.

Open vcXsrv, and start it with these settings (in order of the pages seen in the startup wizard):
1. Leave all settings as-is (Multiple Windows, Display Number -1)
2. Leave all settings as-is (Start no client)
3. Uncheck Native opengl, Check Disable access control (leave other settings as-is)
4. Click Finish

Once vcXsrv is running, run this command in WSL:
```
export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0"
```

## Start the Docker Container and Gazebo

Start the Docker container using this command in WSL:
```
docker-compose run --rm wsl
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
