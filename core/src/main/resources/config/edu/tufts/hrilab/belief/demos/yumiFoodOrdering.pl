subtype(var, object).
subtype(concept, var).
subtype(physical, var).
subtype(location, concept).
subtype(pose, concept).
subtype(physobj, physical).
subtype(agent, physical).
subtype(property, concept).
subtype(area, concept).
subtype(room, concept).

subtype(robot, agent).
subtype(yumi, robot).
subtype(mobileyumi, robot).
subtype(yumipose, pose).
subtype(humanpose, pose).

%TODO based on agents in the GM?
object(left, yumi).
object(right, yumi).
object(human, mobileyumi).

%For instruction from the webapp
actor(manager).
actor(front).
role(front,supervisor(X)):-diarcAgent(X).
role(manager,supervisor(X)):-diarcAgent(X).
role(manager,admin(X)):-diarcAgent(X).

%Prevent the customer from messing with the execution state
isAdminGoal(supersedeSystemGoal(D,defineItemByAnalogy(D,X))).
isAdminGoal(defineItemByAnalogy(D,X)).