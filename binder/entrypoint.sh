#!/bin/bash

# Launch the ROS core and web tools when containter starts
source ${HOME}/workspace/ros/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch &

# Add other startup programs here

# Start MongoDB and save data on working directory
MONGODB_URL=mongodb://127.0.0.1:27017
mongod --fork --logpath ${HOME}/mongod.log

export KNOWROB_MONGO_USER=""#neemReader
export KNOWROB_MONGO_PW=""#qEWRqc9UdN5TD7No7cjymUA8QEweNz
export KNOWROB_MONGO_DB=""#neems
export KNOWROB_MONGO_HOST='127.0.0.1'
export KNOWROB_MONGO_PORT=27017

# Die URI Variable wird von vielen KnowRob-Versionen bevorzugt
export KNOWROB_MONGODB_URI='mongodb://127.0.0.1:27017'
# -----------------------

# Create a symbolic link to the folder neem_data
ln -s /neem_data ${PWD}/neem_data

# Launch Knowrob
roslaunch --wait knowrob openease-test.launch &

# The following line will allow the binderhub start Jupyterlab, should be at the end of the entrypoint.
exec "$@"