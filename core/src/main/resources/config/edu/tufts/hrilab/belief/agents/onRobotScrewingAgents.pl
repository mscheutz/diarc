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
role(brad,supervisor(X)):-diarcAgent(X).
role(james,supervisor(X)):-diarcAgent(X).
role(eric,supervisor(X)):-diarcAgent(X).

%% admin
role(brad,admin(X)):-diarcAgent(X).
role(james,admin(X)):-diarcAgent(X).
role(eric,admin(X)):-diarcAgent(X).
