#!/bin/bash

#######****************** Options *******************#########


USAGE() { 
    echo "Usage: $0 -P -D -H" 1>&2
    echo "-P : Run all tests with the PFM Controller"
    echo "-D : Run all tests with the DWA Controller"
    echo "-H : Run all tests with the Highlevel Controller"
    echo "The default order of execution for multiple controllers is PFM -> DWA -> Highlevel"
    exit 1
}

# We gave a initial value to TCWI and TCC so that if we forget to put -I and -C, it will warn us
PFM="something"
DWA="something"
HLC="something"

options="PDW"
while getopts $options option
do
    case $option in
    P) PFM=$option;;
    D) DWA=$option;;
    H) HLC=$option;;
    \?) USAGE;;
    esac
done

if ((OPTIND == 1)); then
    USAGE
fi
shift $((OPTIND - 1))

# Check if all options are there
if [ -z "$PFM" ] || [ -z "$DWA" ] || [ -z "$HLC" ]; then
    USAGE
fi

#######******************* Compile ********************#########

cd controller
make
sleep 1
cd ..

#######****************** Main Test *******************#########

# Choose a method:
if [ "$PFM" == "P" ] ; then
    CONTROLLER="libpfm_controller.so"
    # Empty(PFM - SUCCESS: 28.53s)
    ./rsim --controller controller/$CONTROLLER --map maps/empty.jpg --target-x 50 --target-y 50
    # Base(PFM - SUCCESS: 35.88s)
    ./rsim --controller controller/$CONTROLLER --map maps/base.jpg --target-x 50 --target-y 50
    # Cone(PFM - SUCCESS: 36.83s)
    ./rsim --controller controller/$CONTROLLER --map maps/cone.jpg --target-x 79 --target-y 35
    # High_Density(PFM - TIMEOUT Error: it tries going around but it never enters the dense obstacle field)
    ./rsim --controller controller/$CONTROLLER --map maps/high_density.png --target-x 50 --target-y 50
    # Line(PFM - TIMEOUT Error: it spins above the center of the line non-stop)
    ./rsim --controller controller/$CONTROLLER --map maps/line.png --target-x 30 --target-y 55
    # Race_Track(PFM - CRASH Error: repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/race_track.jpg --target-x 79 --target-y 55
    # Test1(PFM - SUCCESS: 37.32s)
    ./rsim --controller controller/$CONTROLLER --map maps/test1.jpg --target-x 50 --target-y 50
    # Test2(PFM - SUCCESS: 38.08s)
    ./rsim --controller controller/$CONTROLLER --map maps/test2.jpg --target-x 74 --target-y 22
    # Test3(PFM - SUCCESS: 28.83s)
    ./rsim --controller controller/$CONTROLLER --map maps/test3.jpg --target-x 40 --target-y 30
    # Test4(PFM - SUCCESS: 61.78s : it goes around the obstacle field)
    ./rsim --controller controller/$CONTROLLER --map maps/test4.jpg --target-x 70 --target-y 55 --timeout 70
    # Test5(PFM - CRASH Error: repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/test5.jpg --target-x 79 --target-y 45
    # Test6(PFM - SUCCESS: 41.15s)
    ./rsim --controller controller/$CONTROLLER --map maps/test6.jpg --target-x 45 --target-y 35
    # Test7(PFM - CRASH Error: repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/test7.jpg --target-x 55 --target-y 10
    # Walls(PFM - CRASH Error: the attractive force is not strong enough at the end + repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/walls.jpg --target-x 60 --target-y 54
fi
# Comments DWA TODO
if [ "$DWA" == "D" ] ; then
    CONTROLLER="libdwa_controller.so"
    # Empty(PFM - SUCCESS: 28.53s)
    ./rsim --controller controller/$CONTROLLER --map maps/empty.jpg --target-x 50 --target-y 50
    # Base(PFM - SUCCESS: 35.88s)
    ./rsim --controller controller/$CONTROLLER --map maps/base.jpg --target-x 50 --target-y 50
    # Cone(PFM - SUCCESS: 36.83s)
    ./rsim --controller controller/$CONTROLLER --map maps/cone.jpg --target-x 79 --target-y 35
    # High_Density(PFM - TIMEOUT Error: it tries going around but it never enters the dense obstacle field)
    ./rsim --controller controller/$CONTROLLER --map maps/high_density.png --target-x 50 --target-y 50
    # Line(PFM - TIMEOUT Error: it spins above the center of the line non-stop)
    ./rsim --controller controller/$CONTROLLER --map maps/line.png --target-x 30 --target-y 55
    # Race_Track(PFM - CRASH Error: repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/race_track.jpg --target-x 79 --target-y 55
    # Test1(PFM - SUCCESS: 37.32s)
    ./rsim --controller controller/$CONTROLLER --map maps/test1.jpg --target-x 50 --target-y 50
    # Test2(PFM - SUCCESS: 38.08s)
    ./rsim --controller controller/$CONTROLLER --map maps/test2.jpg --target-x 74 --target-y 22
    # Test3(PFM - SUCCESS: 28.83s)
    ./rsim --controller controller/$CONTROLLER --map maps/test3.jpg --target-x 40 --target-y 30
    # Test4(PFM - SUCCESS: 61.78s : it goes around the obstacle field)
    ./rsim --controller controller/$CONTROLLER --map maps/test4.jpg --target-x 70 --target-y 55 --timeout 70
    # Test5(PFM - CRASH Error: repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/test5.jpg --target-x 79 --target-y 45
    # Test6(PFM - SUCCESS: 41.15s)
    ./rsim --controller controller/$CONTROLLER --map maps/test6.jpg --target-x 45 --target-y 35
    # Test7(PFM - CRASH Error: repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/test7.jpg --target-x 55 --target-y 10
    # Walls(PFM - CRASH Error: the attractive force is not strong enough at the end + repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/walls.jpg --target-x 60 --target-y 54
fi
# Comments Highlevel TODO 
if [ "$HLC" == "H" ] ; then
    CONTROLLER="libdwa_controller.so"
    # Empty(PFM - SUCCESS: 28.53s)
    ./rsim --controller controller/$CONTROLLER --map maps/empty.jpg --target-x 50 --target-y 50
    # Base(PFM - SUCCESS: 35.88s)
    ./rsim --controller controller/$CONTROLLER --map maps/base.jpg --target-x 50 --target-y 50
    # Cone(PFM - SUCCESS: 36.83s)
    ./rsim --controller controller/$CONTROLLER --map maps/cone.jpg --target-x 79 --target-y 35
    # High_Density(PFM - TIMEOUT Error: it tries going around but it never enters the dense obstacle field)
    ./rsim --controller controller/$CONTROLLER --map maps/high_density.png --target-x 50 --target-y 50
    # Line(PFM - TIMEOUT Error: it spins above the center of the line non-stop)
    ./rsim --controller controller/$CONTROLLER --map maps/line.png --target-x 30 --target-y 55
    # Race_Track(PFM - CRASH Error: repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/race_track.jpg --target-x 79 --target-y 55
    # Test1(PFM - SUCCESS: 37.32s)
    ./rsim --controller controller/$CONTROLLER --map maps/test1.jpg --target-x 50 --target-y 50
    # Test2(PFM - SUCCESS: 38.08s)
    ./rsim --controller controller/$CONTROLLER --map maps/test2.jpg --target-x 74 --target-y 22
    # Test3(PFM - SUCCESS: 28.83s)
    ./rsim --controller controller/$CONTROLLER --map maps/test3.jpg --target-x 40 --target-y 30
    # Test4(PFM - SUCCESS: 61.78s : it goes around the obstacle field)
    ./rsim --controller controller/$CONTROLLER --map maps/test4.jpg --target-x 70 --target-y 55 --timeout 70
    # Test5(PFM - CRASH Error: repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/test5.jpg --target-x 79 --target-y 45
    # Test6(PFM - SUCCESS: 41.15s)
    ./rsim --controller controller/$CONTROLLER --map maps/test6.jpg --target-x 45 --target-y 35
    # Test7(PFM - CRASH Error: repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/test7.jpg --target-x 55 --target-y 10
    # Walls(PFM - CRASH Error: the attractive force is not strong enough at the end + repulsive forces missguide the robot to a wall)
    ./rsim --controller controller/$CONTROLLER --map maps/walls.jpg --target-x 60 --target-y 54
fi
