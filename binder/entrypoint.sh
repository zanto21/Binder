#!/bin/bash

# Launch the ROS core and web tools when containter starts
source ${HOME}/workspace/ros/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch &

# Add other startup programs here

# Start MongoDB and save data on working directory
MONGODB_URL=mongodb://127.0.0.1:27017
mongod --fork --logpath ${HOME}/mongod.log

# Create a symbolic link to the folder neem_data
ln -s /neem_data ${PWD}/neem_data

# Launch Knowrob
export KNOWROB_MONGODB_URI=${MONGODB_URL}/?appname=knowrob
roslaunch --wait knowrob knowrob.launch &

# The following line will allow the binderhub start Jupyterlab, should be at the end of the entrypoint.
exec "$@"