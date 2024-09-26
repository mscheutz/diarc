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
supervisor(brad).
supervisor(eric).
supervisor(medic).
supervisor(searcher).
supervisor(user).
supervisor(chris).
supervisor(evan).
supervisor(ravenna).
supervisor(tyler).
supervisor(marlow).
supervisor(manager).

%% admin
admin(brad).
admin(manager).
admin(evan).
