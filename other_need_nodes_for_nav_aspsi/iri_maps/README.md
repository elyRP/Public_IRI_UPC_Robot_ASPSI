# Description

Navigation maps package, with 2D map images normally generated with Gmapping and used by map_server package.

# Dependencies


# Install

This package, as well as all IRI dependencies, can be installed by cloning the
repository inside the active workspace:

```
roscd
cd ../src
git clone https://gitlab.iri.upc.edu/labrobotica/ros/navigation/iri_maps
```

However, this package is normally used as part of a wider installation (i.e. a
robot, an experiment or a demosntration) which will normally include a complete
rosinstall file to be used with the [wstool](http://wiki.ros.org/wstool) tool.

# How to use it

 When launching a map_server node, pass the path to a map yaml from this repository as argument.
 
 For example:
 
```
rosrun map_server map_server `rospack find iri_maps`/maps/willow.yaml
```