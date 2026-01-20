cameraHeight(338).

object(gripper,eetype).
object(screwdriver,eetype).
object(robotone, agent).
object(robottwo, agent).

free(robotone).
free(robottwo).
currenteetype(robotone,gripper).
currenteetype(robottwo,gripper).

%%%%%%% Planning %%%%%%%%%%%%%%
subtype(modelID, cognexobj).
subtype(eetype,concept).

subtype(var, object).
subtype(concept, var).
subtype(physical, var).
subtype(location, concept).
subtype(method, concept).
subtype(cognexobj, physical).
subtype(agent, physical).
subtype(container, cognexobj).
subtype(arm, agent).
subtype(place, location).
subtype(pose, location).

actor(james).
supervisor(james).
admin(james).
actor(pete).
supervisor(pete).
admin(pete).
actor(eric).
supervisor(eric).
admin(eric).

%todo: we shouldn't have to assert this.
predicate(property_of,var,property).
