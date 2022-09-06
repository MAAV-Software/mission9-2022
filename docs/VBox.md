# Setup MAAV Software on Windows / Mac (Intel)

By the end of this tutorial, you should have the software repository on your system, and be able to start a virtual linux operating system to run and simulate the vehicle behavior.

## Install Required Software

The simulation and development environment for our vehicle requires the use of a linux operating system. There a couple ways that we can do this (flashing an old computer of yours with a new operating system, "dual-booting" your computer with two seperate operating systems, and setting up a virtual machine). This doc goes over how to set up a virtual machine using [Virtualbox](https://www.virtualbox.org/) because it is by far the least invasive.

You will need to download the newest version of Virtualbox for your current operating system: [Virtualbox Download](https://www.virtualbox.org/wiki/Downloads). Please note that Virtualbox is not configured to work on macs with M1 processors.

Next, you will need to download the this linux distrobution: [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/20.04/). Make sure to dowload the "Desktop Image" and store the image in a safe place once it's installed. 

Finally, setup the virtual environment by going through the steps in [this tutorial](https://medium.com/tech-lounge/how-to-install-ubuntu-on-mac-using-virtualbox-3a26515aa869).

<ins>Recommended Settings:</ins>
- Virtual HD: 20 GB 
- RAM: 8000+ MB
- Processors: 2+
- Video Memory: 50+ MB

Now you should be able to spin up your very own Ubuntu desktop! For some mac users that are experiencing high latency and lag, I found [this page](https://mkyong.com/mac/virtualbox-running-slow-and-lag-on-macos-macbook-pro/) very helpful.


## Clone the Software Code

Open up a terminal in your new Ubuntu desktop and use this command to pull all of our software code from Github onto your computer.

```
$ git clone https://github.com/MAAV-Software/mission9-2022.git
```
You can either clone via HTTPS (no login required for `clone`, `fetch`, or `pull` but must login with username/password for `push`), or [SSH](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) (more secure, uses private SSH key created on your system that matches with your public key on your Github account, can have an optional password, and is checked for any remote Git operation). We recommend SSH, but you can use whichever is easier for you.

## Configure Your Ubuntu Environment
In this section we will download all necessary dependencies for simulating and developing software for the drone.

Go inside the software repo you just downloaded by typing the following after the `$`.
```
$ cd mission9-2022/scripts
```

Now run the installation script as an admin user by typing
```
$ sudo ./install_everything
```
**NOTE: THIS SHOULD FAIL**

The Docker image should build successfully, but let us know if there's any issues or bugs you run into.

## Start the Docker Container and Gazebo

Initialize the USB drivers and start the Docker container using these commands in WSL from the MAAV repo (you will need to do this every time you want to work in the Docker container):
```
$ sudo bash scripts/probe_drivers.sh
$ docker-compose up wsl
```
Navigate to [localhost:6080](http://localhost:6080) in your browser and you should see a desktop environment. Open a new terminal window.

You can test to see if USB passthrough and Realsense are working correctly by running the following command:
```
$ realsense-viewer
```

You can also run a test to see if PX4 and Gazebo will start and display the simulator, as expected. Using the number of threads on your computer from earlier in the tutorial, run this command:
```
$ cd /px4_sitl/PX4-Autopilot && make px4_sitl -j [threads] gazebo
```
Note that this will compile some necessary binaries (which only happens one time until the Docker container is shut down) before starting PX4-Autopilot and Gazebo.

You can also test to see if your

You can also debug opening Gazebo without PX4 running (which is useful for testing GUI settings) by using this command:
```
$ gazebo --verbose
```
