beenperceived(assistaselfphysobj_0).
at(assistaselfphysobj_0,pose_0).

name(assista,assista).
diarcAgent(assista).
memberOf(assista, self).
object(assista,agent).
free(assista).
currenteetype(assista,gripper).

%Where is this supposed to come from?
at(assista,pose_0).

diarcAgent(human).
name(human,human).
memberOf(human, self).
object(human, mobileManipulator).
free(human).
subtype(mobileManipulator, agent).