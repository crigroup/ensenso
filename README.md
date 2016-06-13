# ensenso

ROS package developed by the [Control Robotics Intelligence Group](http://www.ntu.edu.sg/home/cuong/) from the [Nanyang Technological University, Singapore](http://www.ntu.edu.sg). This package acts as a ROS driver for the ensenso 3D cameras.

**Update (13-Jun-2016)**: On going development continues in the **kinetic-devel** branch.

For ROS **Hydro** or **Indigo** you can use the **master** branch.

### Our setup
* Ensenso camera N35 (Ethernet version) (Tested with N35-802-16-BL and N35-804-16-BL models)
* ROS Kinetic (Ubuntu 16.04, 64 bits)

### Maintainers
* [Francisco Suárez Ruiz](http://fsuarez6.github.io)

### Documentation
* See the installation instructions below.
* Throughout the various files in this repository.

### Build Status

*TODO*

(Waiting for travis-ci to give support to Ubuntu 16.04. See [this issue](https://github.com/travis-ci/travis-ci/issues/5821) for details)

## Installation

Go to your ROS working directory:
```{bash}
cd ~/catkin_ws/src
``` 

Clone this repository:
```{bash}
git clone https://github.com/crigroup/ensenso.git
``` 

Install any missing dependencies using rosdep:
```
rosdep update
rosdep install --from-paths . --ignore-src -y
``` 

Now compile your ROS workspace:
```{bash}
cd ~/catkin_ws && catkin_make

``` 

### Testing the Installation

Be sure to always source the appropriate ROS setup file, e.g:
```
source ~/catkin_ws/devel/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try the following command (Be patient, the ensenso camera takes around **10 seconds to start**):
```
roslaunch ensenso viewer.launch
``` 

## Troubleshooting

### Cannot connect to the camera

Look for errors using the ueye camera manager:
```{bash}
ueyecameramanager
``` 

Try restarting the ueye daemon

```{bash}
sudo /etc/init.d/ueyeethdrc restart
``` 

In case this doesn't work, try a forced stop before restarting:

```{bash}
sudo /etc/init.d/ueyeethdrc force-stop
``` 

### Camera IP

You can change the camera IP using the `ueyesetip` command.
