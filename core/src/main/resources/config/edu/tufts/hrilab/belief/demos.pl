%% agents the system should know about
%% LOAD SPECIFIC AGENTS INFO FILE (e.g., agents/agents.pl)

%% agent name inference
have(A,exists(X,name(X))):-name(A,Y).
itk(A,name(B,X)):-(name(B,X),itkR(A,nameOf(B))).
itk(A,at(B,X)):-(at(B,X),itkR(A,locationOf(B))).

%% moral/trust related reasoning
%% agent on believes things that people it trusts tell it
bel(X,trusted(Y)):-admin_of(Y,X).
trust(X,Y):-diarcAgent(X),bel(X,trusted(Y)).

%% if one diarcAgent believes something, then all diarc agents believe it
%bel(diarcAgents,X):-wantBel(B,A,X),trust(A,B),diarcAgent(A).
%bel(A,X) :- diarcAgent(A), bel(diarcAgents, X).
bel(A,X):-believes(A,X).
bel(D,X):-believes(A,X),diarcAgent(A),diarcAgent(D), not(A=D).
propertyOf(X,free) :- bel(A, propertyOf(X,free)), diarcAgent(A).
propertyOf(X,free) :- not(propertyOf(X,blocked)), propertyOf(X,location).

is_supervisor(A,B):-role(A,supervisor(B)).
admin_of(A,B):-role(A,admin(B)).
is_supervisor(A,B):-admin_of(A,B).

role(Y,novice) :- diarcAgent(X),bel(X,novice(Y)).

%% explanations
explanationType(A, incomplete) :- not(role(A,novice)).

%% rules about admin goals, which only adminstrators can give
%% do we need rule with D,D,X as well?
isAdminGoal(learnAction(D,A,X)).
isAdminGoal(endActionLearning(D,A,X)).
isAdminGoal(pauseActionLearning(D,A,X)).
isAdminGoal(resumeActionLearning(D,A,X)).
isAdminGoal(cancelActionLearning(D,A,X)).
isAdminGoal(modifyAction(D,C,X,S,A)).
isAdminGoal(Y) :- (want(B,D,Y),Y=learnAction(D,A,X),diarcAgent(D)).
isAdminGoal(Y) :- (want(B,D,Y),Y=endActionLearning(D,A,X),diarcAgent(D)).
isAdminGoal(Y) :- (want(B,D,Y),Y=pauseActionLearning(D,A,X),diarcAgent(D)).
isAdminGoal(Y) :- (want(B,D,Y),Y=resumeActionLearning(D,A,X),diarcAgent(D)).
isAdminGoal(Y) :- (want(B,D,Y),Y=cancelActionLearning(D,A,X),diarcAgent(D)).
isAdminGoal(Y) :- (want(B,D,Y),Y=modifyAction(D,C,X,S,A),diarcAgent(D)).

%% failure reason explination
%brad: at some point we might want a rule this so that we can do stuff based on the actor in the goal:
%failureReason(did(A,X),isNotAuthorized(B)):-(want(B,X),not(obl(A,did(A,X)))).
failureReason(X,isNotAuthorized(B)):-(want(B,X),X=did(A,Y),not(obl(A,X))).
failureReason(bel(A,X),not(trust(A,B))):-(wantBel(B,A,X),not(trust(A,B))).


%% creates infinite recursive search
%% bel(A,X):-(bel(B,X),trust(A,B))
%% Work around rules because "X:-bel(self,X)" is not an acceptable prolog rule
%want(A,X):-(bel(D,Y),Y=want(A,X),diarcAgent(D)).
will(X):-(bel(D,Y),Y=will(X),diarcAgent(D)).

%% preditcates for beleif notifications
definitionOf(A,X):-bel(self,definitionOf(A,X)). % vision notifications
instanceOf(A,X):-bel(self,instanceOf(A,X)). % vision notifications
implies(A,X):-(bel(self,implies(A,X))). % affordance

%for explanation demo
isIn(A,X):-(bel(D,Y),Y=isIn(A,X),diarcAgent(D)).
occupied(ROOM):- bel(A,isIn(PERSON,ROOM)),diarcAgent(A),actor(PERSON).

% spatial reasoning
isLocated(Nao1,behind,Nao2) :- see(Nao1,Nao2).
openspace(inFrontOf(X)) :- not(see(X,obstacle)).
openspace(behind(Nao2)) :- openspace(inFrontOf(Nao1)),isLocated(Nao1,behind,Nao2).
believes(Nao1,propertyOf(behind(area,Nao2),safe)) :- openspace(inFrontOf(Nao1)),isLocated(Nao2,inFrontOf,Nao1).
behind(Nao2,sensor) :- isLocated(Nao1,behind,Nao2),diarcAgent(Nao1).
isLocated(Nao2,inFrontOf,Nao1) :- isLocated(Nao1,behind,Nao2).

%%%%%%%%%%%%%%%%% TO BE REMOVED %%%%%%%%%%%%%%%%%%%%%%%%%%%

%/* saftey realted goal submission rules
%not currently used by anything
%*/
%per(A,not(do(A,X))):-(unsafe(do(A,X)),diarcAgent(A)).
%%per(A,not(X)):-(unsafe(X),diarcAgent(A)).
%goal(A,X,normal):-(obl(A,X),not(per(A,not(X))),not(goalExceptionPred(X))).
%%if an agent will do something then it has a goal to do it
%%goal(A,X,normal):-will(A,X).
%
%/*safety*/
%%bel(ravenna,believes(andy,instanceOf(X,knife))).
%%bel(ravenna,raised(andy,arms)).
%%bel(ravenna,isStaff(ravenna)).
%
%hasEffect(did(A,releaseObject(X)),possibly(harmed(Y))):-raised(andy,arms),believes(A,instanceOf(X,knife)).
%isUnsafe(did(A,X)):-(hasEffect(did(A,X),possibly(harmed(A)))).
%
%isForbidden(did(A,X)):-bel(A,notPermitted(give(X,staff))),isStaff(Y).
%isStaff(ravenna).
%
%isImproper(did(A,handle(X))) :- belongsTo(X,Y),not(past(permit(Y,A,did(A,handle(X))))).
on(magnet,fridge,back,front).
