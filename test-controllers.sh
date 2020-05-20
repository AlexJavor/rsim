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

# We give a initial value to PFM, DWA and HLC so that if we forget to put -P, -D and -H, it will warn us
PFM="something"
DWA="something"
HLC="something"
# ...dark side

options="PDH"
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

# ***************************** PFM success rate: 8/14 ********************************* #
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
    # Line(PFM - TIMEOUT Error: spins above the center of the line non-stop)
    ./rsim --controller controller/$CONTROLLER --map maps/line.png --target-x 30 --target-y 55 --timeout 70
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

#**************************** DWA success rate: 11/14 ***************************** #
if [ "$DWA" == "D" ] ; then
    CONTROLLER="libdwa_controller.so"
    # Empty(DWA - SUCCESS: 37.65s)
    ./rsim --controller controller/$CONTROLLER --map maps/empty.jpg --target-x 50 --target-y 50
    # Base(DWA - SUCCESS: 38.72s)
    ./rsim --controller controller/$CONTROLLER --map maps/base.jpg --target-x 50 --target-y 50
    # Cone(DWA - TIMEOUT Error: Ignores the target)
    ./rsim --controller controller/$CONTROLLER --map maps/cone.jpg --target-x 79 --target-y 35
    # High_Density(DWA - SUCCESS: 39.53)
    ./rsim --controller controller/$CONTROLLER --map maps/high_density.png --target-x 50 --target-y 50
    # Line(DWA - SUCCESS: 66.53s)
    ./rsim --controller controller/$CONTROLLER --map maps/line.png --target-x 30 --target-y 55 --timeout 70
    # Race_Track(DWA - SUCCESS: 50.90s)
    ./rsim --controller controller/$CONTROLLER --map maps/race_track.jpg --target-x 79 --target-y 55
    # Test1(DWA - CRASH Error: Spins once, and then it slowly approches to target)
    ./rsim --controller controller/$CONTROLLER --map maps/test1.jpg --target-x 50 --target-y 50
    # Test2(DWA - SUCCESS: 43.25s)
    ./rsim --controller controller/$CONTROLLER --map maps/test2.jpg --target-x 74 --target-y 22
    # Test3(DWA - CRASH Error: Slowly approches straight to the obstacle until it crashes)
    ./rsim --controller controller/$CONTROLLER --map maps/test3.jpg --target-x 40 --target-y 30
    # Test4(DWA - SUCCESS: 47.90s)
    ./rsim --controller controller/$CONTROLLER --map maps/test4.jpg --target-x 70 --target-y 55 --timeout 70
    # Test5(DWA - SUCCESS: 51.62s)
    ./rsim --controller controller/$CONTROLLER --map maps/test5.jpg --target-x 79 --target-y 45
    # Test6(DWA - SUCCESS: 43.55s : Spins once before it reaches the end)
    ./rsim --controller controller/$CONTROLLER --map maps/test6.jpg --target-x 45 --target-y 35
    # Test7(DWA - SUCCESS: 39.63s)
    ./rsim --controller controller/$CONTROLLER --map maps/test7.jpg --target-x 55 --target-y 10
    # Walls(DWA - SUCCESS: 43.47s)
    ./rsim --controller controller/$CONTROLLER --map maps/walls.jpg --target-x 60 --target-y 54
fi

# ******************************* Highlevel success rate: 5/14 ********************************** #
if [ "$HLC" == "H" ] ; then
    CONTROLLER="libhighlevel_controller.so"
    # Empty(HLC - SUCCESS: 28.38s)
    ./rsim --controller controller/$CONTROLLER --map maps/empty.jpg --target-x 50 --target-y 50
    # Base(HLC - SUCCESS: 43.00s)
    ./rsim --controller controller/$CONTROLLER --map maps/base.jpg --target-x 50 --target-y 50
    # Cone(HLC - CRASH Error: Goes backwards until it crashes with the first obstacle it encounters)
    ./rsim --controller controller/$CONTROLLER --map maps/cone.jpg --target-x 79 --target-y 35
    # High_Density(HLC - TIMEOUT Error: Stuck near an obstacle)
    ./rsim --controller controller/$CONTROLLER --map maps/high_density.png --target-x 50 --target-y 50
    # Line(HLC - TIMEOUT Error: spins above the center of the line non-stop)
    ./rsim --controller controller/$CONTROLLER --map maps/line.png --target-x 30 --target-y 55 --timeout 70
    # Race_Track(HLC - SUCCESS: 52.83s)
    ./rsim --controller controller/$CONTROLLER --map maps/race_track.jpg --target-x 79 --target-y 55
    # Test1(HLC - SUCCESS: 50.08s)
    ./rsim --controller controller/$CONTROLLER --map maps/test1.jpg --target-x 50 --target-y 50
    # Test2(HLC - SUCCESS: 44.30s)
    ./rsim --controller controller/$CONTROLLER --map maps/test2.jpg --target-x 74 --target-y 22
    # Test3(HLC - TIMEOUT Error: Stuck near an obstacle)
    ./rsim --controller controller/$CONTROLLER --map maps/test3.jpg --target-x 40 --target-y 30
    # Test4(HLC - TIMEOUT Error: Stuck near an obstacle)
    ./rsim --controller controller/$CONTROLLER --map maps/test4.jpg --target-x 70 --target-y 55 --timeout 70
    # Test5(HLC - CRASH Error: Goes backwards until it crashes with the first obstacle it encounters)
    ./rsim --controller controller/$CONTROLLER --map maps/test5.jpg --target-x 79 --target-y 45
    # Test6(HLC - TIMEOUT Error: Scared to enter the field)
    ./rsim --controller controller/$CONTROLLER --map maps/test6.jpg --target-x 45 --target-y 35
    # Test7(HLC - CRASH Error: Goes backwards until it crashes with the first obstacle it encounters)
    ./rsim --controller controller/$CONTROLLER --map maps/test7.jpg --target-x 55 --target-y 10
    # Walls(HLC - CRASH Error: Goes backwards until it crashes with the first obstacle it encounters)
    ./rsim --controller controller/$CONTROLLER --map maps/walls.jpg --target-x 60 --target-y 54
fi
