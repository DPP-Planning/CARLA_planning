#!/bin/sh

runs="1"
noise="0"
timeout="3600"
path_planner="dlite"
motion_planner="basic"

while getopts r:n:t:p:m: flag
do
    case "${flag}" in
        r) runs=${OPTARG};;
        n) noise=${OPTARG};;
        t) timeout=${OPTARG};;
        p) path_planner=${OPTARG};;
        m) motion_planner=${OPTARG};;
    esac
done


scenic agents/navigation/scenic_test_0.scenic -b --count ${runs} --param ps ${path_planner} --param mp ${motion_planner} --param timeout ${timeout} --param noise ${noise} --simulate --2d --model scenic.simulators.carla.model
#scenic agents/navigation/scenic_test_A.scenic -b --count ${runs} --param timeout ${timeout} --param noise ${noise} --simulate --2d --model scenic.simulators.carla.model
#scenic agents/navigation/scenic_test_B.scenic -b --count ${runs} --param timeout ${timeout} --param noise ${noise} --simulate --2d --model scenic.simulators.carla.model
#scenic agents/navigation/scenic_test_C.scenic -b --count ${runs} --param timeout ${timeout} --param noise ${noise} --simulate --2d --model scenic.simulators.carla.model
