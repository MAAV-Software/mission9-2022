# Setup Simulation Camera in Gazebo

By the end of this tutorial, you should have a simluated realsense camera in the gazebo simulation that publishes image data, depth data, and point-cloud data. This is currently a work in progress so don't get salty and ask Drew is you have questions. 

## Modify the quadcopter sdf file

Please make sure that a .sdf has been generated for the drone that you are simulating. By default the drone that Gazebo is simulating is called the `iris drone`. You can search the whole system for sdf files (robot description xml similar to `urdf`) using the following command. See [here](https://newscrewdriver.com/2018/07/31/ros-notes-urdf-vs-gazebo-sdf/#:~:text=My%20understanding%20can%20be%20boiled,can%20be%20represented%20in%20URDF.) for a description of the differences between sdf and urdf.

```
$ find / -name "*.sdf"          # all sdf files
$ find / -name "iris.sdf"       # for iris drone specifically
```

Edit the .sdf file using the code editor of your choice (e.g. vim, nano, etc).

```
$ vim iris.sdf
```

We are going to add our own plugin to the robot model! At the bottom of the file, add the following code between the last `</plugin>` tag and the `</model>` tag.

<ins>New Plugin XML:</ins>
```

```

Finally, setup the virtual environment by going through the steps in [this tutorial](https://medium.com/tech-lounge/how-to-install-ubuntu-on-mac-using-virtualbox-3a26515aa869).

<ins>Recommended Settings:</ins>
- Virtual HD: 25+ GB 
- RAM: 8000+ MB
- Processors: 2+
- Video Memory: 50+ MB

Now you should be able to spin up your very own Ubuntu desktop! For some mac users that are experiencing high latency and lag, I found [this page](https://mkyong.com/mac/virtualbox-running-slow-and-lag-on-macos-macbook-pro/) very helpful.


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
$  sudo -i
$ ./install_everything
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
