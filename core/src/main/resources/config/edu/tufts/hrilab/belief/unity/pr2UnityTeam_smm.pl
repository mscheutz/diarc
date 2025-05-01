/*ds realted stuff, not exactly sure what it's for */
%dspred(true(truth),1.0,1.0).

/* agents the system should know about */
name(robot1,robot1). /* we might want this to be dynamic too */
name(robot2,robot2).

actor(server).
actor(player1).
actor(player2).

diarcAgent(self).
diarcAgent(robot1).
diarcAgent(robot2).

memberOf(X,X).
memberOf(robot1, self).
memberOf(robot2, self).
object(robot1, agent).
object(robot2, agent).

object(self, agent).
team(self).

amIn(robot1,alpha).
amIn(robot2,beta).
workload(1).

/* agent name infrence */
have(A,exists(X,name(X))):-name(A,Y).
itk(A,name(B,X)):-(name(B,X),itkR(A,nameOf(B))).
itk(A,at(B,X)):-(at(B,X),itkR(A,locationOf(B))).

/* moral/trust related reasoning */
/* agent on believes things that people it trusts tell it*/
bel(X,trusted(Y)):-admin_of(Y,X).
trust(X,Y):-diarcAgent(X),bel(X,trusted(Y)).

%% if one diarcAgent believes something, then all diarc agents believe it
%bel(diarcAgents,X):-wantBel(B,A,X),trust(A,B),diarcAgent(A).
%bel(A,X) :- diarcAgent(A), bel(diarcAgents, X).
bel(A,X):-believes(A,X).
bel(D,X):-believes(A,X),diarcAgent(A),diarcAgent(D).

/*rules about who the agent is obliged to listen to */
supervisor(server).
supervisor(player1).
supervisor(player2).

supervisor_of(S,D):-supervisor(S),diarcAgent(D).
admin_of(S,D):-admin(S),diarcAgent(D).
explanationType(A, incomplete) :- not(role(A,novice)).

/*
rules about admin goals, which only adminstrators can give
these actions no longer exist, saving as arefrence for how to do this in the future if we ever want to */
isAdminGoal(did(D,modifyActionLearning(X,S))).
isAdminGoal(did(D,modifyAction(X,S,A))).
isAdminGoal(self,disableObstacleChecks(self)).
isAdminGoal(self,disableSupportChecks(self)).
isAdminGoal(D,Y) :- (want(B,Y),Y=did(D,modifyActionLearning(X,S))),diarcAgent(D).
isAdminGoal(D,Y) :- (want(B,Y),Y=did(D,modifyAction(X,S,A))),diarcAgent(D).

/* question answering related -- not too sure how this works*/
have(A,exists(X,name(X))):-name(A,Y).
itk(A,name(B,X)):-(name(B,X),itkR(A,nameOf(B))).
itk(A,at(B,X)):-(at(B,X),itkR(A,locationOf(B))).
