%% agents the system should know about.
name(robotone,robotone).

actor(eric).
actor(pete).

diarcAgent(self).
diarcAgent(robotone).

object(self,agent).

memberOf(X,X).
memberOf(robotone, self).
memberOf(robottwo, self).
team(self).

/*rules about who the agent is obliged to listen to */
%% supervisors
supervisor(pete).
supervisor(user).
supervisor(eric).

%% admin
admin(eric).
admin(user).
