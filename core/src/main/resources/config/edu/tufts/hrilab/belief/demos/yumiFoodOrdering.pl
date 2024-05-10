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
subtype(mobileManipulator, robot).
subtype(leftArmpose, pose).
subtype(rightArmpose, pose).
subtype(humanpose, pose).

%Prevent the customer from messing with the execution state
isAdminGoal(supersedeSystemGoal(D,defineItemByAnalogy(D,X))).
isAdminGoal(defineItemByAnalogy(D,X)).
isAdminGoal(defineItem(D,X)).
admin_of(Y,X):-bel(X,admin(Y)).