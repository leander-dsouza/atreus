#!/bin/bash

repos_links=(
    git@github.com:mojin-robotics/ar_track_alvar.git
    git@github.com:SyrianSpock/realsense_gazebo_plugin.git
    git@github.com:plusk01/gzsatellite.git
    git@github.com:aws-robotics/aws-robomaker-small-warehouse-world.git
)

# Travel to the earlier directory.
cd ../

# Clone repos
for repo in ${repos_links[@]}; do
    git clone $repo
done
