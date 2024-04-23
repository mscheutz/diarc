# MoveItComponent Configuration Files
### Overview
One of the major advantages of MoveIt! is its ability to work on a wide array of machines. To fully take 
advantage of this functionality, we need our MoveItComponent to be as generalizable. The JSON configurations 
found in this directory allow for the same codebase to be reconfigured for the specific needs of your 
setup, and take advantage of the full ADE system while minimizing the time sink into that process.

To facilitate this goal, this document outlines the different parameters that can be set and their functionality.

### Using a config file, and how it works
Several robot components extend the base MoveItComponentImpl and need to be launched using the sub-class (no explicit
-config flag needed in this case as it's hardcoded in the sub-class).

For those components that don't have a required sub-class, you can use a config file to run the MoveItComponent 
with the -config argument, i.e.,

```$xslt
ant launch -Dmain=edu.tufts.hrilab.moveit.MoveItComponent -Dargs="-config UR5"
```

This tells the MoveItComponent to seek out 'UR5.JSON' in `com/moveit/config/`. It's also possible to specify 
absolute/relative paths, but they must be JSON files ending in `.json`.

When a JSON file is properly specified, GSON is used to load the contents into dummy classes (found in 
`com/moveit/config/GsonObjects`). These objects are later used in MoveItComponentImpl to pull out the data 
specified by the config file and set the appropriate class variables.

### Special MoveIt Config Cases
Some robots have a component that extends the base MoveItComponentImpl and don't usually need the `-config` argument.
There are some special cases though of robots with their own component but need to specify a `-config` argument.
These are the known cases:
- `Fetch_TOWER.json`: When using the TOWER configuration of the Fetch. We need a new move group for the presser. If that
  is the case you need to load the move group and use `Fetch_TOWER.json` as an argument to the Fetch component. More 
  information is on the [TOWER README](https://hrilab.tufts.edu:22280/robots/TOWER/documentation/-/tree/main). 

### Parameters
For clarity, the parameters that can be set will be broken into a handful of different types.

##### Simple base parameters
Here are the simple base parameters found in UR5.json:
```$xslt
  "configName": "UR5",
  "baseLinkString": "base_link",
  "pointingFrame": "camera_frame",
  "pressingFrame": "presser_link",
  "maxMovetoAttempts": 5,
  "maxNodeCheckCount": 10,
  "armReach": 0.5,
  "graspApproachOffset": 0.265,
  "graspContactOffset": 0.165,
  "allowDisableCollisionAvoidance": false,
```

The **configName** is a unique name to identify this configuration. It is used only for debugging/logging.
 
 Note that the appropriate MoveGroup is built for your version of ROS 
 automatically.
 
**baseLinkString** is where things get less trivial. The base link is the link from which transforms are 
computed relative to. By ROS URDF convention, this is `base_link`, but it doesn't need to be.

**pointingFrame** when pointing to an object, this frame is used as the origin of the direction
vector used as the pointing direction.

**pressingFrame** when pressing an object, we might have an additional attachment to improve pressing on the robot.
This frame is used to specify which frame is used to press and object.

**maxMoveToAttempts** is used when computing movement requests: it is used as the number of planning attempts 
in the request to be generated. It is also used in situations where it may be desireable to try a movement again 
if it failed the first time: the movements will not exceed this amount.

**maxNodeCheckCount** is used when setting up ROS nodes. If, after this many attempts to reach ROS (spaced out
 by 1 second), we still haven't been able to connect to ROS, we should have failed to connect.

**armReach** is the maximum length of the arm, and is used when pointing to objects or locations.

**graspApproachOffset** and **graspContactOffset** are used when grabbing objects. Object grasping is a two-stage
process: in stage one, we get pretty close to the object and stop for just a moment. This allows for us to
potentially take momentum into account, or to disable collision avoidance (see allowDisableCollisionAvoidance).
We then move into the second stage, in which the object is actually grabbed. 
In stage one, the end-effector joint is moved to graspApproachOffset distance away from the goal position.
In stage two, the end-effector joint is moved to graspContactOffset distance away from the goal position
(note that these positions, being in ROS, are in meters). These values are non-zero to take into account
the offset between the end-effector link and gripper pads. They are typically best found experimentally.

**allowDisableCollisionAvoidance** is a boolean that allows MoveIt to ignore potential collisions with objects
viewed in the OctoMap (depth camera data). In machines like the PR2, a gentle collision in the second stage is
OK, so this is true. In the case of the UR5, the safety constraints are not controlled by us so this isn't an option.
Note that whatever the state of this variable, the explicitly set collision objects will always be present and cannot
be planned into.

**rosNamespace** is an optional arg that allows you set push-down the MoveGroup and JointStateSub DIARCROS nodes into a
ROS namespace. This will change all the topics of those two nodes to be in this namespace. The default namespace is simply "/".

#### eeTransform
The transform that aligns the end-effector's coordinate frame with a standard gripper pose (x: dir of 
gripper pointing. y: dir of gripper open/close. z: remaining direction orthogonal to x and y). This is needed
so that pointTo, graspObject, and other methods work consistently across robot platforms.

#### Optional static transforms
Your use case may present a situation where you need to add a tf transform (e.g., adding a camera frame that wasn't there before).
Additionally, modifying a URDF might not be undesirable because it requires us to maintain 
our own fork of that robot. 

To get around this, config files allow you to add static transforms. localStaticTransforms is a JSON array 
that takes the `parent`, `child`, `x`, `y`, `z`, `roll`, `pitch`, and `yaw` parameters. Being an array, 
multiple static transforms can be added, i.e.,

```$xslt
  "localStaticTransforms": [
    {
      "parent": "wrist_3_link",
      "child": "camera_frame",
      "x": -0.0325,
      "y": 0.1268,
      "z": 0.06,
      "roll": -1.5707,
      "pitch": 0,
      "yaw": 0
    }
  ]
```

This example transform (found in UR5.json) adds a link our TF node was not previously aware of. 
`wrist_3_link` is a known link already being published. `camera_frame` is a link added for camera purposes 
that is not in the URDF.

Note that in this case, we are only updating our local TF data. Adding transforms in this manner 
does not make them visible to the rest of the ROS system. If that is your need, see ROS's 
`static_transform_publisher`.

Also be aware that this applies the transform translation first, meaning that the x, y and z are relative 
to the parent link. The roll, pitch, and yaw then change the orientation of the origin, allowing better 
control of camera frames or the position of child links.

#### Groups
Groups are a collection of joints that MoveIt! can specify goals to. In order for us to specify what those 
groups should do, we need to know what they are and what's in them.

The default group name must be set. This is the group name used when dealing with trajectories, as a 
fallback when things go wrong, and as a default if no group is specified. As with before, the groups 
are a JSON array, so one to many groups can be specified:

```$xslt
  "defaultGroupName": "manipulator",
  "groups": [
    {
      "groupName": "manipulator",
      "jointNames": [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
      ],
      "effectorLinkName": "ee_link",
      "gripperLinkName": "ee_link",
      "gripperClassName": "edu.tufts.hrilab.manipulator.robotiq.2f85"
    }
  ]
```

A handful of important things are happening here. First, the group name corresponds to the jointNames. 
You can often get away with making up new groupings, but MoveIt needs to be able to find your default group.

Additionally, we're setting the effector link name, the gripper link name, and the gripper class name. 
**effectorLinkName** refers to the name of the link at the tip of the end effector. This is often the 
same as the gripperLinkName as far as we're concerned, but it may not be: there is sometimes a rotation 
between the two that can lead to issues.  **gripperLinkName** is the name of the link onto which the 
gripper is mounted.
Finally, **gripperClassName** is the fully qualified class of the gripper to use. Grippers are instantiated in
edu.tufts.hrilab.manipulator.GenericManipulator, which uses 
this string to find the appropriate class. See that class for more documentation on this area.

#### Poses
Poses can be added as a collection of joint states with a name and a group (although, at the time of 
writing, the group is ignored in favor of using the default group). As with the previous collections, 
this is a JSON array to allow one or more to be set. At minimum, you should set your start pose, i.e.,

```$xslt
  "poses": [
    {
      "poseName": "start",
      "poseGroup": "arms",
      "jointStates": [
        -1.570,
        -1.221,
        -2.094,
        -0.872,
        1.5708,
        0.0
      ]
    }
  ]
```

This example is the start pose for the UR5. The 0th index of the jointStates corresponds to the goal 
joint value for the 0th joint name of the manipulator group, and so on. The start pose should be specified 
to allow `goToStartPose()` to work, but other poses are optional. They will be available to be called 
from `goToPose(pose name)`

#### Collisions
In many use cases, being able to use the depth camera data to be aware of collisions through the octomap 
is suitable. However, when working with the UR5, we found that due to the depth camera being mounted to 
the end effector, having the ability to create a static world of collision objects was helpful. The 
Collisions parameter is an array, allowing many collision objects to be specified, i.e.,

```$xslt
  "collisions": [
    {
      "parent": "base_link",
      "name": "table",
      "x": "0",
      "y": "0",
      "z": "0",
      "roll": "0",
      "pitch": "0",
      "yaw": "0",
      "width": "2",
      "depth": ".74",
      "height": "0"
    },
    {
      "parent": "ee_link",
      "name": "camera",
      "x": "0",
      "y": "0",
      "z": "0.07",
      "roll": "0",
      "pitch": "0",
      "yaw": "0",
      "width": "0.05",
      "depth": "0.18",
      "height": ".05"
    }
  ]
```

This example is from UR5.json, with 4 of the collision objects removed for brevity. The parameters 
being set are similar to the static joint transformations in that we're creating a child of an existing 
parent. The x, y, and z offset from the parent is specified, as is the roll, pitch, and yaw.

Where this differs is that instead of adding simply a transform, we are adding in a box. The box has 
the parameters **width** (the size in meters of the box in the x direction relative to the parent link), 
**depth** (size in the y), and **height** (size in the z). 

There are a handful of nuances to be aware of here. First is that unlike the static transforms, these 
are being published. It will therefore be possible to view them from Rviz.

Second, note that the parent of the table object is the base link, but the parent of the camera object 
is the ee_link. By setting the parent of the table to be the base link, the table is always at that 
location relative to the stationary base link. However, by specifying ee_link as the parent to the camera, 
the camera collision object will move with the ee_link.

It's also worth noting that the height of the table is 0. This produces a barrier of 0 thickness that will 
not be planned into, and is still functional. 

When creating these objects, or using a scene that has them, you may encounter an error from the move_group 
about being 'unable to sample any valid goal states for the goal tree' (or something like that). If you 
see this error, you've likely given collision objects that are always in collision with the robot, or 
you've attempted to plan into a collision object. Use RViz to make sure your collision objects are 
correct and that your goal is valid.
