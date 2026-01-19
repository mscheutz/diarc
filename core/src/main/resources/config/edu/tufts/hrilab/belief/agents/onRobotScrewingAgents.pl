%% agents the system should know about.
name(robotone,robotone).

actor(brad).
actor(james).

diarcAgent(self).
diarcAgent(robotone).

object(self,agent).

memberOf(X,X).
memberOf(robotone, self).
team(self).

/*rules about who the agent is obliged to listen to */
%% supervisors
supervisor(brad).
supervisor(pete).
supervisor(user).
supervisor(eric).

%% admin
admin(brad).
admin(eric).
admin(user).
