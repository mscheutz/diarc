# What is this?
This directory contains manipulators for use in the MoveItComponentImpl. To create a new gripper,
write a new class in a new directory that extends the GenericManipulator. It's an abstract class,
so you'll need to implement the unimplemented methods (the rest are handy utils).

You'll then need to add it to the getClassName() function in GenericManipulator, providing a string
that will allow users to reference it from a configuration JSON. You'll then need to update
src-build-targets.xml. The convention I'm proposing is that new manipulators have the 
target name 'manipulator-uniquePackageName' (i.e., 'manipulator-robotiq'). You'll then need to add
it as a dependency for whatever build target you're working on. 

It should now be possible to use the string you put in getClassName() earlier to call forth this
particular manipulator.
