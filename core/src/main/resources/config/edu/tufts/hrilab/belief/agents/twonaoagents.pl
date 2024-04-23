%% agents the system should know about
actor(chris).
actor(evan).
actor(tyler).
actor(ravenna).
actor(marlow).

diarcAgent(self).
diarcAgent(dempster).
diarcAgent(shafer).
team(self).

memberOf(X,X).
memberOf(dempster, self).
memberOf(shafer, self).

object(dempster, agent).
object(shafer, agent).
object(self, agent).

/*rules about who the agent is obliged to listen to */
%% supervisors
role(chris,supervisor(X)):-diarcAgent(X).
role(evan,supervisor(X)):-diarcAgent(X).
role(ravenna,supervisor(X)):-diarcAgent(X).
role(tyler,supervisor(X)):-diarcAgent(X).
role(marlow,supervisor(X)):-diarcAgent(X).

%% admin
role(evan,admin(X)):-diarcAgent(X).