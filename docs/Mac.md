# Setup MAAV Software on Mac (M1)

Okay bad news, I don't actually have an M1 mac lol so most of this is speculation and may not end up working. That being said...

By the end of this tutorial, you should have the software repository on your system, and be able to start a virtual environment (through UTM) to run and simulate the vehicle behavior.

## Install Required Software

The simulation and development environment for our vehicle requires the use of a linux operating system. There a couple ways that we can do this (flashing an old computer of yours with a new operating system, "dual-booting" your computer with two seperate operating systems, and setting up a virtual machine). **This doc goes over how to set up a virtual machine using [UTM](https://mac.getutm.app/) because it is by far the least invasive for an M1 mac.**

Please install UTM using the link above.

Next, you will need to download the this linux distrobution: [Ubuntu 20.04 (Focal Fossa)](https://cdimage.ubuntu.com/releases/20.04/release/). Make sure to dowload the "Desktop Image" and store the image in a safe place once it's installed. 

Finally, setup the virtual environment by going through the steps in [this tutorial](https://www.youtube.com/watch?v=MVLbb1aMk24&ab_channel=MoodyCodes).

<ins>Recommended Settings:</ins>
- Virtual HD: 25+ GB 
- RAM: 8000+ MB
- Processors: 2+
- Video Memory: 50+ MB

Let me (Drew) know if you have any issues with the installation process and we can troubleshoot any issues together!

Now you should be able to spin up your very own Ubuntu desktop!

## Clone the Software Code
Our team uses git version control to store copies of our code. Open up a terminal in your new Ubuntu desktop and install git onto your new virtual machine.
```
$ sudo apt-get install git
```
Then use these commands to pull all of our software code from Github onto your computer. Once complete, you should have a new folder named `mission9-2022` in your base directory (/).

```
$ cd /
$ sudo git clone https://github.com/MAAV-Software/mission9-2022.git
```
You can either clone via HTTPS (no login required for `clone`, `fetch`, or `pull` but must login with username/password for `push`), or [SSH](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) (more secure, uses private SSH key created on your system that matches with your public key on your Github account, can have an optional password, and is checked for any remote Git operation). We recommend SSH, but you can use whichever is easier for you.

## Configure Your Ubuntu Environment
In this section we will download all necessary dependencies for simulating and developing software for the drone.

Go inside the software repo you just downloaded by typing the following after the `$`.
```
$ cd /mission9-2022/scripts
```

Now switch to the admin user and run the installation script by running the following two commands:
```
$ sudo -i
$ cd /mission9-2022/scripts
$ chmod +x install_everything.sh
$ ./install_everything.sh
```

This should successfully build all the dependencies needed for basic software simulation with px4. To test if everything was set up properly, navigate to the px4_sitl/PX4_Autopilot directory then start up the simulation!
```
$  cd /px4_sitl/PX4-Autopilot/
$  make px4_sitl gazebo
```
If everything works you should see a small quadcopter in a simulated world! Note that this will compile some necessary binaries (which only happens one time until the virtual computer is shut down) before starting PX4-Autopilot and Gazebo.

You can also debug opening Gazebo without PX4 running (which is useful for testing GUI settings) by using this command:
```
$ gazebo --verbose
```

Happy Coding!
