# ensenso
[![Build Status](https://travis-ci.org/crigroup/ensenso.svg?branch=kinetic-devel)](https://travis-ci.org/crigroup/ensenso)

ROS package developed by [CRI Group](http://www.ntu.edu.sg/home/cuong/), [Nanyang Technological University, Singapore](http://www.ntu.edu.sg). This package acts as a ROS driver for ensenso 3D cameras.

### Our setup
* Ensenso camera N35 (Ethernet version) (Tested with N35-802-16-BL and N35-804-16-BL models)
* ROS Kinetic (Ubuntu 16.04, 64 bits)

### Maintainer
* [Francisco Suárez Ruiz](http://fsuarez6.github.io)

### Documentation
* See the installation instructions below.
* Throughout the various files in this repository.

## Installation

### Requirements
Run ``./scripts/install_driver.sh` to install the necessary proprietary software.

As an alternative, manually download and install from [here](https://www.ensenso.com/support/sdk-download/):
* Wibu CodeMeter runtime (Tested version: 6.40.2402)
* EnsensoSDK (Tested version: 2.0.146)
* uEye Driver (Tested version: 4.81.1)

### ROS Package installation
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
