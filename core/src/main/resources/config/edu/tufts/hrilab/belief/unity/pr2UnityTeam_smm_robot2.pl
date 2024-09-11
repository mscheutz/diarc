/*ds realted stuff, not exactly sure what it's for */
%dspred(true(truth),1.0,1.0).

/* agents the system should know about */
name(robot2,robot2).

actor(brad).
actor(tyler).
actor(ravenna).
actor(commX).
actor(server).
actor(player1).
actor(player2).

diarcAgent(self).
diarcAgent(robot2).
diarcAgent(dempster).
diarcAgent(shafer).
diarcAgent(andy).

memberOf(X,X).
memberOf(robot2, self).
object(robot2, agent).

object(self, agent).
team(self).

amIn(robot2,beta).
workload(1).
distraction(1).
propertyOf(betarighttwo,damaged(100)).
propertyOf(alphaleftone,damaged(80)).
propertyOf(alphalefttwo,damaged(30)).
propertyOf(alphaleftthree,damaged(50)).

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
supervisor(brad).
supervisor(ravenna).
supervisor(commX).
supervisor(server).
supervisor(player1).
supervisor(player2).
admin(brad).

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
/*
admin goal stuf not currently being used, keeping it like this in case we want to do something with an obligation hirearchy in the future.
*/
/* obligation rules */
oblSub1(A,B,X):-(want(B,X),supervisor_of(B,A),not(isAdminGoal(A,X))).
%oblSub1(A,B,X):-(want(B,X),X=did(A,Y),is_superior(B,A),not(isAdminGoal(A,X))).
%oblSub1(A,B,X):-(want(B,X),X=obs(A,Y),is_superior(B,A),not(isAdminGoal(A,X))).
oblSub2(A,B,X):-(want(B,X),X=did(A,Y),admin_of(B,A),isAdminGoal(A,X)).
%oblSub2(A,B,X):-(want(B,X),X=obs(A,Y),admin_of(B,A),isAdminGoal(A,X)).
obl(A,X):-(oblSub1(A,B,X)).
obl(A,X):-(oblSub2(A,B,X)).
/* failure reason explination */
/*
brad: at some point we might want a rule this so that we can do stuff based on the actor in the goal:
failureReason(did(A,X),isNotAuthorized(B)):-(want(B,X),not(obl(A,did(A,X)))).
*/
failureReason(X,isNotAuthorized(B)):-(want(B,X),X=did(A,Y),not(obl(A,X))).
%failureReason(X,isNotAuthorized(B)):-(want(B,X),X=obs(A,Y),not(obl(A,X))).
failureReason(bel(A,X),not(trust(A,B))):-(wantBel(B,A,X),not(trust(A,B))).

/* saftey realted goal submission rules
not currently used by anything
*/
unsafe(X):-(hasEffect(X,possibly(harmed(A)))).
isAllowed(X):-not(forbidden(X)).
per(A,not(do(A,X))):-(unsafe(do(A,X))),diarcAgent(A).
per(A,not(X)):-(unsafe(X)),diarcAgent(A).
goal(A,X,normal):-(obl(A,X),not(per(A,not(X))),not(goalExceptionPred(X))).
%if an agent will do something then it has a goal to do it
%goal(A,X,normal):-will(A,X).

/*
creates infinite recursive search
bel(A,X):-(bel(B,X),trust(A,B))
Work around rules because "X:-bel(self,X)" is not an acceptable prolog rule
*/
/*brad: not entirely sure what these are for */
%want(A,X):-(bel(D,Y),Y=want(A,X)),diarcAgent(D).
will(A,X):-(bel(D,Y),Y=will(A,X)),diarcAgent(D).

/* preditcates for beleif notifications */
/* vision notifications */
definitionOf(A,X):-bel(self,definitionOf(A,X)).
instanceOf(A,X):-bel(self,instanceOf(A,X)).
/* affordance */
implies(A,X):-(bel(self,implies(A,X))).

/* question answering related -- not too sure how this works*/
have(A,exists(X,name(X))):-name(A,Y).
itk(A,name(B,X)):-(name(B,X),itkR(A,nameOf(B))).
itk(A,at(B,X)):-(at(B,X),itkR(A,locationOf(B))).
