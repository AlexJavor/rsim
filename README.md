# RSim


rsim is simple, 2D robotic simulator.
It is intended for testing implementations of trajectory tracking controllers.


While the simulator is built in Rust, it provides a simple C interface for implementing controllers and loading them in simulated robots.

# Installation

First, you should have a working rust toolchain : https://doc.rust-lang.org/cargo/getting-started/installation.html


    git clone https://github.com/arthur-bit-monnot/rsim.git
    cd rsim
    cargo build --release
    ln -s target/release/rsim . # link executable into the current directory
    

# Running your first simulation

First you need to build your controller.

    cd controller/
    make
    
This should have created a shared library file `controller/libdummy_controller.so`. This implements a (not so smart) controller and provides it as a shared library.


Then you need your environment. An environment is a 2D image, where each pixel represents 10x10 centimeters.
A black pixel is interpreted as an obstacle. Look at the prexisting maps in `maps/` and choose one to use, for instance `maps/base.jpg`.


To run a simulation, you should provide rsim with a controller and and a map. For instance :

    ./rsim --controller controller/libdummy_controller.so --map maps/base.jpg
    
The expected outcome is that the robot moves towards the target point (red cross). However, not being smart enough to avoid an obstacle it stops to avoid a collision.
After one minute, the simulation will timeout.

The purple points on the visualisation depicts obstacles that are seen by the robot's laser scan.
    
     