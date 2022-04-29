# MoveIt based collision checker + object placement

ROS based collision checker service based on an edited MoveIt core package. 

The idea is based on the work made by @pradeepr-roboticist in this [issue](https://github.com/ros-planning/moveit/issues/2097).

The `moveit_core` is updated for ROS `noetic` as of **29 Apr 2022**.

_______
### Installation

My suggestion is to install `moveit_core` first from `apt`, and then override the package by installing this in your `catkin_ws`.

_______
### How it works

The main edited file in `moveit_core` is `moveit_core\collision_detection_fcl\collision_env_fcl.cpp` line [333](https://github.com/eliabntt/moveit_based_collision_checker_and_placement/blob/main/moveit_core/collision_detection_fcl/src/collision_env_fcl.cpp#L333).
This function accept in input two lists of objects, checks if the objects are in the moveit planning scene (i.e. they exists), and perform a low level collision checking request (standard moveit library).
It requires as output the distance, the contacts and the cost returning this information (lines 361-367 for those settings).

The function can be called with `planning_scene_->getCollisionEnv()->checkCollisionBetweenObjectGroups(ob_l1, ob_l2);`.
The object lists are simply the ids of the objects (strings) in the scene.

The collision checking node I developed than expose a ROS service that make use of this function.

The main purpose of the service is to PLACE objects in a given scene. It does that by random location guessing within a given perimeter and a given world.

You need to provide the stl/dae for all the objects you're going to use.

It also implements a checking procedure such that the object is placed within an ARBITRARY polygon.

There are two ways of using this service:
- single (always the same) object placement in the world: simply check collisions between the two meshes (one is in ob_l1, one is in ob_l2). The mesh of this object gets pre-loaded when the node is launched.
- multiple object placement in the world: recursevly check collisions between one mesh and the already placed ones. First iteration is `mesh1-world`, second is `mesh2-[world,mesh1]`, etc. NOTE: note that this can be used also as a single-object placement with different meshes per each iteration.

_________
### Usage

Simply build the node.
Run `roslaunch collision_check.launch`.

This will launch:
- the collision check node
- a moveit demo scene with a `firefly` robot (generated with `moveit_setup_assistant`, based on [firefly_rotors_simulator](https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_description/urdf/firefly.xacro))

The `firefly` will be used for "camera" placement, i.e. the single object placement. In the launch file you can edit the object
The service is listening at `/fake/collision_checker/check` and is defined as follows:

```
string env_stl_path # ALWAYS REQUIRED
string[] ob_names # OPTIONAL
string[] ob_stl_paths # REQUIRED IF OB_NAMES SET (can be also dae)
bool is_cam
float64 forced_z # REQUIRED -1 if NOT used, otherwise set
float64[] min_limits # REQUIRED min and max limits for the rng
float64[] max_limits
geometry_msgs/Point[] env_polygon # REQUIRED the polygon to check the in/out
int32 limit_collision # REQUIRED maximum amount of collision point detected between the meshes
bool reset # reset the environment
---
float64[] x
float64[] y
float64[] z
float64[] yaw
```

The rule is that either `is_cam` needs to be set to `TRUE` OR `ob_names` needs to be non-empty.
`is_cam` takes preference to `ob_names` (i.e. the program check `is_cam` value before proceding).
`is_cam` leads to option 1 above (one mesh checked against the environment, mesh pre-loaded).
`ob_names` (1 or more) leads to option 2 above (different meshes checked against the environment).

As I said above `is_cam` use the pre-loaded mesh (setup in the launch file as param).

When option 2 is sought `ob_stl_paths` needs to have one entry for EACH object *even when repeated* with the full absolute path of the mesh.

You need to set `min` and `max` limits for `x,y,z`, the `polygon` vertices (`x,y`). Those will be used by the uniform random number generator.

The service return the in-order `x,y,z,yaw` location found. (yaw spanning the whole circle).
IF IT WAS NOT POSSIBLE TO PLACE THE OBJECT THE SYSTEM USE A FIXED VALUE
```
x = max_limits.at(0) + 10;
y = max_limits.at(1) + 10;
z = -10;
yaw = 0;
```
(lines 155-158 of the code).



The workflow is as follow:
- reset the scene if required or if the env changes
- create the env if necessary (red color, ID "world")
- create the object mesh in the env
- until a valid location is found generate random x,y,z,yaw values
- check for collision
- repeat a maximum of 90 times or until the maximum amount of possible collision is satisfied

___________
## License

All Code in this repository - unless otherwise stated in local license or code headers is

Copyright 2021 Max Planck Institute for Intelligent Systems, Tübingen.

Licensed under the terms of the GNU General Public Licence (GPL) v3 or higher. See: https://www.gnu.org/licenses/gpl-3.0.en.html
