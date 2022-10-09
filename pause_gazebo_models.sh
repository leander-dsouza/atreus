#!/bin/bash

sed -i "s/.*export GAZEBO_MASTER_URI.*/export GAZEBO_MASTER_URI=/g" /usr/share/gazebo/setup.sh
sed -i "s/.*export GAZEBO_MODEL_DATABASE_URI.*/export GAZEBO_MODEL_DATABASE_URI=/g" /usr/share/gazebo/setup.sh

sed -i "s/.*export GAZEBO_MASTER_URI.*/export GAZEBO_MASTER_URI=/g" /usr/share/gazebo-11/setup.sh
sed -i "s/.*export GAZEBO_MODEL_DATABASE_URI.*/export GAZEBO_MODEL_DATABASE_URI=/g" /usr/share/gazebo-11/setup.sh
