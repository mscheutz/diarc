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
subtype(gofa, robot).
%subtype(human, agent).
subtype(yumipose, pose).
subtype(mobileyumipose, pose).
subtype(gofapose, pose).

%TODO based on agents in the GM?
object(yumi, yumi).
object(mobileyumi, mobileyumi).
object(gofa, gofa).

%For instruction from the webapp
actor(employee).
actor(customer).
role(employee,supervisor(X)):-diarcAgent(X).
role(employee,admin(X)):-diarcAgent(X).

%Prevent the customer from messing with the execution state
isAdminGoal(freeze(D,X,S)).
isAdminGoal(endFreeze(D,X,S,A)).
isAdminGoal(cancelCurrentGoal(D,X,S)).
isAdminGoal(cancelGoalInQueueIndex(D,X,S)).
isAdminGoal(Y) :- (want(B,D,Y),Y=freeze(D,X,S),diarcAgent(D)).
isAdminGoal(Y) :- (want(B,D,Y),Y=endFreeze(D,X,S,A),diarcAgent(D)).
isAdminGoal(Y) :- (want(B,D,Y),Y=cancelCurrentGoal(D,X,S),diarcAgent(D)).
isAdminGoal(Y) :- (want(B,D,Y),Y=cancelGoalInQueueIndex(D,X,S),diarcAgent(D)).
isOpenGoal(orderMeal(D,X)).
isOpenGoal(orderMeal(D,X,S)).