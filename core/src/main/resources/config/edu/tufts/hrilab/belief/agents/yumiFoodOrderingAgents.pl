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
role(brad,supervisor(X)):-diarcAgent(X).
role(eric,supervisor(X)):-diarcAgent(X).
role(pete,supervisor(X)):-diarcAgent(X).
role(matthias,supervisor(X)):-diarcAgent(X).

%% admin
role(brad,admin(X)):-diarcAgent(X).
role(eric,admin(X)):-diarcAgent(X).
role(pete,admin(X)):-diarcAgent(X).
role(matthias,admin(X)):-diarcAgent(X).
