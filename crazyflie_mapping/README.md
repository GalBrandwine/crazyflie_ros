# Crazyflie mapping
A swarm management and mappping tools for crazyflie.

## Hardware dependencies:
* Crazyflie
* Flow_deck
* Ranger_deck

## Software dependencies:
* Python modules (pip install):
    * Numpy
    * Bresenham
    * Matplotlib == 2.2.4
    * Shapely
    * Descartes
    * Copy
    
* Ros packages (apt-get install ros-kinetic-PACKAGE_NAME):
    * tf2_sensor_msgs
    * nav_msgs
    * sensor_msgs

## Usage:
* Inside 'indoors_mapping.launch', set the correct CF's radio & the correct params for Mapping node.
* from terminal: roslaunch crazyflie_mapping indoors_mapping.launch