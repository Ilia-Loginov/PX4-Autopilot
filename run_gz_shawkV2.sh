#!/bin/bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris_shawkV2.world

#run test case

#DONT_RUN=1 make px4_sitl gazebo mavsdk_tests
#test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/hitl_shawkV2.json --speed-factor 1 --gui --case 'Takeoff and Land' --verbose --hitl --connection ethernet
