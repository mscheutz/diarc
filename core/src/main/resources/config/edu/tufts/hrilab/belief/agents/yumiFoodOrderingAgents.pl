%% agents the system should know about
name(self,self). %% we might want this to be dynamic too
name(leftArm,leftArm).
name(rightArm,rightArm).
name(human,human).

actor(brad).
actor(eric).
actor(pete).
actor(matthias).

diarcAgent(self).
diarcAgent(leftArm).
diarcAgent(rightArm).
diarcAgent(human).

memberOf(X,X).
memberOf(leftArm, self).
memberOf(rightArm, self).
memberOf(human, self).
object(leftArm, yumi).
object(rightArm, yumi).
object(human, mobileManipulator).

object(self, agent).
team(self).

/*rules about who the agent is obliged to listen to */
%% supervisors
supervisor(brad).
supervisor(eric).
supervisor(pete).
supervisor(matthias).

%% admin
admin(brad).
admin(eric).
admin(pete).
admin(matthias).
