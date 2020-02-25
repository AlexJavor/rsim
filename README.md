# RSim


rsim is simple, 2D robotic simulator.
It is intended for testing implementations of trajectory tracking controllers.


While the simulator is built in Rust, it provides a simple C interface for implementing controllers and loading them in simulated robots.

## Installation

First, you should have a working rust toolchain : https://doc.rust-lang.org/cargo/getting-started/installation.html

    sudo apt install libasound2-dev libudev-dev   # install uncommon dependencies
    git clone https://github.com/arthur-bit-monnot/rsim.git
    cd rsim
    cargo build --release       # build (might be long)
    ln -s target/release/rsim . # link executable into the current directory
 
At this point you should have an executable called `rsim` in your current directory.    

## Running your first simulation

First you need to build your controller.

    cd controller/
    make
    
This should have created a shared library file `controller/libdummy_controller.so`. This implements a (not so smart) controller and provides it as a shared library.


Then you need your environment. An environment is a 2D image, where each pixel represents 10x10 centimeters.
A black pixel is interpreted as an obstacle. Look at the prexisting maps in `maps/` and choose one to use, for instance `maps/base.jpg`.


To run a simulation, you should provide rsim with a controller and a map. For instance:

    ./rsim --controller controller/libdummy_controller.so --map maps/base.jpg
    
The expected outcome is that the robot moves towards the target point (red cross). However, not being smart enough to avoid an obstacle it stops to avoid a collision.
After one minute, the simulation will timeout (you can also exit early by closing the window or pressing escape).

The purple points on the visualisation depicts obstacles that are seen by the robot's laser scan.

You can change the target point (represented as a red cross) by clicking on the map.

You can look at all the available options of `rsim` with:

    rsim -h

    

## Making your own controller     

To make you own controller you need to :

 - create a implementation of the controller's plugin interface. 
   This interface is defined `controller/controller.h` and you can find an example implementation in `controller/dummy_controller.c`.
 - you need to compile this controller as a shared library (a `.so` file on linux). 
   The `Makefile` in `controller/` shows how to do this for the dummy controller.
 - You need to provide it to RSIM as a command line argument `./rsim --controller path/to/controller.so`
 
 
The compiler interface defines common data types and a set of predefined functions that will be invoked when new sensor data is available.


## Making you own map

A map is simply an image where a pixel represents a 10cm by 10cm area. 
We consider that a pixel is occupied if it is black.
You can make your own maps and provide them to rsim with the `-m` option. 
Look into the `maps/` directory for two example maps (note that `empty.jpg` contains no obstacles).  


## Vehicles models

The simulator supports the simple car model and the differential drive model for vehicles. 
For simplicity the robot is represented as a disc both for visualisation and collision checking.

The pose of the robot is given by a tuple `(x, y, theta)` where `(x,y)` represent its absolute position in a 2D space and `theta` is its orientation in radians.
As command, the robots expects a tuple `(u_v, u_rot)` where `u_v` controls the velocity and `u_rot` controls the rotation.

`rsim` uses the simple car model as a default but the user can swith to a differential drive with `--in-place` command line option

### Dynamics of simple car model

    dx = u_v * cos(theta)
    dy = u_v * sin(theta)
    dtheta = u_v * u_theta  
    
Note that a simple car cannot turn in place.    
    
### Dynamics of differential drive model

    dx = u_v * cos(theta)
    dy = u_v * sin(theta)
    dtheta = u_theta
    
Unlike a simple car, a robot with differential drive can turn in place.    