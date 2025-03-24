#!/bin/bash

# Load parameters
rosparam load $(rospack find ctmp_single)/config/regions.yaml
rosparam set query_mode true


for i in {1..9}
do
    rosparam set curr_object $i
    rosparam set region_type "pick"

    # Run first node
    rosrun ctmp_single ctmp_single_preprocess

    # Change parameters
    rosparam set region_type "place"

    # Run second node
    rosrun ctmp_single ctmp_single_preprocess
done
