%% agents the system should know about
name(self,shafer). %% we might want this to be dynamic too
name(shafer,shafer).
name(dempster,dempster).
name(robotone,robotone).
name(robottwo,robottwo).
name(armone,armone).
name(armtwo,armtwo).
name(zno,zno).
name(zno_two,zno_two).
name(andy,andy).
name(fetch,fetch).
name(temi,temi).
name(leftArm,leftArm).
name(rightArm,rightArm).
name(human,human).

actor(brad).
actor(eric).
actor(medic).
actor(searcher).
actor(user).
actor(chris).
actor(evan).
actor(tyler).
actor(ravenna).
actor(marlow).
actor(manager).
actor(front).

diarcAgent(self).
diarcAgent(dempster).
diarcAgent(shafer).
diarcAgent(robotone).
diarcAgent(robottwo).
diarcAgent(armone).
diarcAgent(armtwo).
diarcAgent(zno).
diarcAgent(zno_two).
diarcAgent(andy).
diarcAgent(fetch).
diarcAgent(temi).
diarcAgent(gofa).
diarcAgent(yumi).
diarcAgent(mobileyumi).
diarcAgent(leftArm).
diarcAgent(rightArm).
diarcAgent(human).

memberOf(X,X).
memberOf(temi, self).
memberOf(fetch, self).
memberOf(andy, self).
memberOf(leftArm, self).
memberOf(rightArm, self).
memberOf(human, self).
memberOf(human, self).
object(temi, agent).
object(andy, agent).
object(fetch, agent).
object(leftArm, yumi).
object(rightArm, yumi).
object(human, mobileyumi).

object(self, agent).
team(self).

/*rules about who the agent is obliged to listen to */
%% supervisors
role(brad,supervisor(X)):-diarcAgent(X).
role(eric,supervisor(X)):-diarcAgent(X).
role(medic,supervisor(X)):-diarcAgent(X).
role(searcher,supervisor(X)):-diarcAgent(X).
role(user,supervisor(X)):-diarcAgent(X).
role(chris,supervisor(X)):-diarcAgent(X).
role(evan,supervisor(X)):-diarcAgent(X).
role(ravenna,supervisor(X)):-diarcAgent(X).
role(tyler,supervisor(X)):-diarcAgent(X).
role(marlow,supervisor(X)):-diarcAgent(X).
role(manager,supervisor(X)):-diarcAgent(X).

%% admin
role(brad,admin(X)):-diarcAgent(X).
role(manager,admin(X)):-diarcAgent(X).
%role(eric,admin(X)):-diarcAgent(X).
%role(medic,admin(X)):-diarcAgent(X).
%role(searcher,admin(X)):-diarcAgent(X).
%role(user,admin(X)):-diarcAgent(X).
%role(chris,admin(X)):-diarcAgent(X).
role(evan,admin(X)):-diarcAgent(X).
%role(ravenna,admin(X)):-diarcAgent(X).
%role(tyler,admin(X)):-diarcAgent(X).
%role(marlow,admin(X)):-diarcAgent(X).
