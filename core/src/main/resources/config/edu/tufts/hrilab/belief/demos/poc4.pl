object(gripper,eetype).
object(screwdriver,eetype).

free(self).
currenteetype(self,gripper).

%%%%%%% Planning %%%%%%%%%%%%%%
subtype(modelID, physobj).
subtype(eetype,concept).

subtype(var, object).
subtype(concept, var).
subtype(physical, var).
subtype(location, concept).
subtype(method, concept).
subtype(physobj, physical).
subtype(agent, physical).
subtype(container, physobj).
subtype(arm, agent).
subtype(place, location).
subtype(pose, location).
