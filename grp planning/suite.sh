#!/bin/sh

runs="1"
noise="0"
timeout="3600"

while getopts r:n:t: flag
do
    case "${flag}" in
        r) runs=${OPTARG};;
        n) noise=${OPTARG};;
        t) timeout=${OPTARG};;
    esac
done


#scenic agents/navigation/scenic_test_0.scenic -b --count ${runs} --param timeout ${timeout} --param noise ${noise} --simulate --2d --model scenic.simulators.carla.model
#scenic agents/navigation/scenic_test_A.scenic -b --count ${runs} --param timeout ${timeout} --param noise ${noise} --simulate --2d --model scenic.simulators.carla.model
scenic agents/navigation/scenic_test_B.scenic -b --count ${runs} --param timeout ${timeout} --param noise ${noise} --simulate --2d --model scenic.simulators.carla.model
#scenic agents/navigation/scenic_test_C.scenic -b --count ${runs} --param timeout ${timeout} --param noise ${noise} --simulate --2d --model scenic.simulators.carla.model
