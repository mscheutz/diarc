%% agents the system should know about
actor(matt).
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
supervisor(matt).
supervisor(evan).
supervisor(ravenna).
supervisor(tyler).
supervisor(marlow).

%% admin
admin(evan).