# ensenso

ROS package developed by the [Control Robotics Intelligence Group](http://www.ntu.edu.sg/home/cuong/) from the [Nanyang Technological University, Singapore](http://www.ntu.edu.sg). This package allows to receive in ROS the rigid bodies streamed by Motive 1.7.

The **master** branch is compatible with both ROS **Hydro** and **Indigo**.

**Update (13-Jun-2016)**: On going development continues in the **kinetic-devel** branch.

### Our setup
  * Ensenso camera N35 (Ethernet version) (Tested with N35-802-16-BL and N35-804-16-BL models)
  * ROS Hydro (Ubuntu 12.04, 64 bits) or ROS Indigo (Ubuntu 14.04, 64 bits)

### Maintainers
  * [Francisco Suárez Ruiz](http://fsuarez6.github.io)

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

-   uEye Driver **4.72**
-   EnsensoSDK **1.3.180**

**Note:** We have the **ethernet** version, so install the corresponding drivers.

Now, check your installation running the `nxView` command.

The first time you run it, the camera should appear under `Monocular Cameras`. Press the **Auto Config** button. This will move the camera to `Depth Cameras`. Select the camera and click **Open**.

In case you don't see the camera at all, check your internet connection, ping the camera or try to restart the ueye daemon: `sudo /etc/init.d/ueyeethdrc restart`

PCL 1.8
-------

You need at least **PCL 1.8.0** to be able to use the Ensenso cameras. To do so, you need to install PCL from source.

### Dependencies
```{bash}
$ sudo apt-get install g++ libboost-all-dev libeigen3-dev libflann-dev libvtk5-dev python-vtk libvtk5-qt4-dev libvtk-java libqhull-dev libgomp1 libpcap-dev
```

### Source Code

Clone the PCL repository:
```{bash}
$ cd ~/git
$ git clone https://github.com/PointCloudLibrary/pcl.git
``` 

### Supporting C++11

We require `c++11` support. In the file `~/git/pcl/CMakeLists.txt` locate the line that contains this:

```{bash}
SET(CMAKE_CXX_FLAGS "-Wall ...
``` 

And add the `c++11` flag:

**Ubuntu 12.04**
```{bash}
SET(CMAKE_CXX_FLAGS "-Wall -std=c++0x ...
``` 

**Ubuntu 14.04**
```{bash}
SET(CMAKE_CXX_FLAGS "-Wall -std=c++11 ...
``` 

### Compile and Install

Create a build directory, compile and install:

```{bash}
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

You need to make `pcl_ros` depend on **PCL 1.8.0**. In the file `~/catkin_ws/src/perception_pcl/pcl_ros/CMakeLists.txt` locate the line that contains this:
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

### `OpenNI2` linking error

If you get errors related to `OpenNI2` or `boost version required 1.47` when building `pcl` set the `WITH_OPENNI2` compilation flag to `OFF`:
```{bash}
$ cmake -DCMAKE_BUILD_TYPE=Release -DWITH_OPENNI2=OFF .. 
``` 

### `VLPGrabber` Linking error

In the file `~/git/pcl/io/CMakeLists.txt` comment out the source file `src/vlp_grabber.cpp`.

Additionally, in the file `~/git/pcl/visualization/tools/CMakeLists.txt` comment out these two lines:
```{bash}
PCL_ADD_EXECUTABLE(pcl_vlp_viewer ${SUBSYS_NAME} vlp_viewer.cpp)
target_link_libraries(pcl_vlp_viewer pcl_io pcl_common pcl_visualization)
``` 

### Compile errors with Assembler messages

If you get errors like these while compiling PCL:
```{bash}
/tmp/ccRLy4Re.s:2488: Error: no such instruction: `vfmadd312ss (%r9),%xmm2,%xmm1'
/tmp/ccRLy4Re.s:2638: Error: no such instruction: `vfmadd312ss (%rdx),%xmm2,%xmm1'
/tmp/ccRLy4Re.s:3039: Error: no such instruction: `vfmadd312ss (%rax,%r11,4),%xmm5,%xmm1'
/tmp/ccRLy4Re.s:3402: Error: no such instruction: `vfmadd312ss (%rax,%r11,4),%xmm5,%xmm1'
/tmp/ccRLy4Re.s:3534: Error: no such instruction: `vfmadd312ss (%rax,%rdx,4),%xmm1,%xmm2'
``` 
Please refer to [these answers](http://stackoverflow.com/questions/17126593/compile-errors-with-assembler-messages). For the `dellstation` (Ubuntu 12.04), I had to use: `-march=native -mno-avx`.

### Cannot connect to the camera

Look for errors using the ueye camera manager:
```{bash}
$ ueyecameramanager
``` 

Try restarting the ueye daemon

```{bash}
$ sudo /etc/init.d/ueyeethdrc restart
``` 

In case this doesn't work, try a forced stop before restarting:

```{bash}
$ sudo /etc/init.d/ueyeethdrc force-stop
``` 

### Camera IP

You can change the camera IP using the `ueyesetip` command.
