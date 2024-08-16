%% agents the system should know about.
name(robotone,robotone).
name(robottwo,robottwo).

actor(brad).
actor(james).

diarcAgent(self).
diarcAgent(robotone).
diarcAgent(robottwo).

object(self,agent).

memberOf(X,X).
memberOf(robotone, self).
memberOf(robottwo, self).
team(self).

/*rules about who the agent is obliged to listen to */
%% supervisors
supervisor(brad).
supervisor(james).
supervisor(eric).

%% admin
admin(brad).
admin(james).
admin(eric).
