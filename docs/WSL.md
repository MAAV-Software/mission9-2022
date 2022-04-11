# Setup MAAV Software on Windows / WSL

By the end of this tutorial, you should have the software repository on your system, and be able to start a virtual system (Docker container) to run and simulate the vehicle behavior.

## Install Required Software

If you don't have WSL (Windows Subsystem for Linux) already installed on your machine, [follow these instructions.](https://eecs280staff.github.io/p1-stats/setup_wsl.html)

You will also need Docker Desktop, which is used to create images or "snapshots" of virtual systems and run them in containers. [Download Docker Desktop here.](https://docs.docker.com/get-docker/) There will be some additional settings config to make Docker work with WSL, but you can test if Docker is setup correctly by running this command in WSL:
```
$ docker
```

If the command doesn't work, check your Docker Desktop settings or make sure Docker Desktop is still running.

## Clone the Software Code

Use this command in WSL (take note of where you clone this to, you will need it for later):
```
$ git clone --branch wsl-realsense https://github.com/MAAV-Software/mission9-2021.git
```
You can either clone via HTTPS (no login required for `clone`, `fetch`, or `pull` but must login with username/password for `push`), or [SSH](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) (more secure, uses private SSH key created on your system that matches with your public key on your Github account, can have an optional password, and is checked for any remote Git operation). We recommend SSH, but you can use whichever is easier for you.

## Build Modified Linux Kernel and Install USB Packages

In order for WSL to be able to recognize and use the Realsense camera, you need to build your own custom kernel with the proper drivers enabled. First, clone the WSL kernel repo in WSL:
```
$ git clone https://github.com/microsoft/WSL2-Linux-Kernel.git
```
Next, enter the kernel directory and copy the `.config` file from the MAAV repository:
```
$ cd WSL2-Linux-Kernel
$ cp [path-to-MAAV-repo]/.config .config
```
Install build dependecies:
```
$ sudo apt install build-essential flex bison dwarves libssl-dev libelf-dev
```
Run the following command to see how many CPU threads you have:
```
$ getconf _NPROCESSORS_ONLN
```
Using the number of threads from the previous step to speed up execution, run the following commands to build the kernel and install the resulting modules.
```
$ sudo make -j [threads] && sudo make modules_install -j [threads] && sudo make install -j [threads]
```
Install USBIP tools in WSL:
```
$ sudo apt install linux-tools-5.4.0-77-generic hwdata
$ sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/5.4.0-77-generic/usbip 20
```
Move the custom kernel to somewhere on your Windows drive:
```
$ cp arch/x86/boot/bzImage /mnt/c/[wherever you want]/maavImage
```
Create a `.wslconfig` file and add the following lines to tell WSL to use your custom kernel (make sure to delimit path with `\\`, e.g. `c:\\Users\\<user>\\MAAV\\maavImage`):
```
$ touch /mnt/c/Users/<user>/.wslconfig
$ nano /mnt/c/Users/<user>/.wslconfig
```
`.wslconfig`
```
[wsl2]
kernel=c:[wherever you want]\\maavImage
```
Next, install USBIPD-WIN from [this link](https://github.com/dorssel/usbipd-win/releases) (download the `.msi` installer from the latest release), which provides the bridge from Windows to WSL for USB devices. Finally, close all your WSL shells and open a new command prompt (not WSL) as administrator. Run `wsl --shutdown` and open a new WSL window. Enter `uname -r` and you should see the line `[version-number]-maav-modified-WSL2+` if everything worked correctly. Finally, install the Realsense drivers:
```
$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
$ sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
$ sudo apt-get install librealsense2-dkms
$ sudo apt-get install librealsense2-utils
```
## Build the Docker Image

For more information about using Docker, [see the Docker doc page (not up to date).](./Docker.md)

Inside the root folder of the MAAV repository you cloned, run this command:
```
$ docker build -t maav-mission9 .
```
**Note the '.' at the end!**

The Docker image should build successfully, but let us know if there's any issues or bugs you run into.

## Pass Realsense camera to WSL

Open a new command prompt (not WSL) as administrator. Plug in a Realsense camera and enter the following command:
```
$ usbipd list
```
You should see a list of usb devices. Note the `BUSID` of your Realsense camera. Run the following command to attach your camera to WSL (you will have to do this every time you want to use the Realsense camera in WSL):
```
$ usbipd wsl attach --busid [BUSID]
```
You will now have access to the camera in WSL. If you want to detach the camera from WSL, run the following command:
```
$ usbipd wsl detach --busid [BUSID]
```

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
