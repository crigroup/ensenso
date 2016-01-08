# ensenso

ROS package developed by the [Control Robotics Intelligence Group](http://www.ntu.edu.sg/home/cuong/) from the [Nanyang Technological University, Singapore](http://www.ntu.edu.sg). This package allows to receive in ROS the rigid bodies streamed by Motive 1.7.

The **master** branch is compatible with both ROS **Hydro** and **Indigo**.

### Our setup
  * Ensenso camera N35 (Ethernet version)
  * ROS Hydro (Ubuntu 12.04, 64 bits) or ROS Indigo (Ubuntu 14.04, 64 bits)

**Maintainer:** Francisco Suárez Ruiz, [fsuarez6.github.io](fsuarez6.github.io)

### Documentation
  * See the installation instructions below.
  * Throughout the various files in this repository.

### Build Status

TODO

Installation
===============

Ensenso Drivers
---------------

The Ensenso drivers are available for download here: <http://www.ensenso.com/support/sdk-download/>. You need to install 2 things:

-   uEye Driver
-   EnsensoSDK

**Note:** We have the **ethernet** version, so install the corresponding drivers.

Now, check your installation running the `nxView` command.

The first time you run it, the camera should appear under `Monocular` `Cameras`. Press the **Auto Config** button. This will move the camera to `Depth` `Cameras`. Select the camera and click **Open**.

In case you don't see the camera at all, check your internet connection, ping the camera or try to restart the ueye daemon: `sudo /etc/init.d/ueyeethdrc restart`

PCL 1.8
-------

You need at least **PCL 1.8.0** to be able to use the Ensenso cameras. To do so, you need to install PCL from source.

### Source Code

Clone the PCL repository:
```{bash}
$ cd ~/git
$ git clone https://github.com/PointCloudLibrary/pcl.git
```

### Supporting c++11 (optional)

Most likely, we will require `c++11` support. In the file `~/git/pcl/CMakeLists.txt` locate the line that contains this:

``` bash
SET(CMAKE_CXX_FLAGS "-Wall ...
```

Add the `c++11` flag:

``` bash
SET(CMAKE_CXX_FLAGS "-Wall -std=c++11 ...
```

### Compile and Install

Create a build directory, compile and install:

``` bash
$ cd ~/git/pcl
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make -j `nproc`
$ sudo make install
```

Repositories Installation
-----------------------

Go to your ROS working directory. e.g.
```{bash}
$ cd ~/catkin_ws/src
``` 

Clone the required repositories:
```{bash}
$ git clone https://github.com/ros-perception/perception_pcl.git -b $ROS_DISTRO-devel
$ git clone https://github.com/crigroup/ensenso.git
``` 

You need to make `ros_pcl` depend on **PCL 1.8.0**. In the file `pcl_ros/CMakeLists.txt` locate the line that contains this:
```{bash}
find_package(PCL REQUIRED)
```

And replace it by:
```{bash}
find_package(PCL 1.8.0 REQUIRED)
```

Install any missing dependencies using rosdep:
```
$ rosdep update
$ rosdep install --from-paths . --ignore-src -y
``` 

Now compile your ROS workspace. e.g.
```{bash}
$ cd ~/catkin_ws && catkin_make
```

Testing the Installation
------------------------

Be sure to always source the appropriate ROS setup file, e.g:
```
$ source ~/catkin_ws/devel/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try the following command (Be patient, the ensenso camera takes around 10 seconds to start):
```
$ roslaunch ensenso viewer.launch
``` 

Troubleshooting
===============

### Cannot connect to the camera

Try restarting the ueye daemon

``` bash
$ sudo /etc/init.d/ueyeethdrc restart
```

In case this doesn't work, try a forced stop before restarting:

``` bash
$ sudo /etc/init.d/ueyeethdrc force-stop
```

### Camera IP

You can change the camera IP using the `ueyesetip` command.
