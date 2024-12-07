%% agents the system should know about
name(self,robot). %% we might want this to be dynamic too
name(robot,robot).

actor(matthias).

diarcAgent(self).
diarcAgent(robot).

memberOf(X,X).
memberOf(robot, self).
object(robot, agent).
object(self, agent).
team(self).

/*rules about who the agent is obliged to listen to */
%% supervisors
supervisor(hengxu).

%% admin
admin(hengxu).
