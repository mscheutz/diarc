%% agents the system should know about
name(self,self).

actor(evan).

diarcAgent(self).

memberOf(X,X).
memberOf(self, self).
team(self).

/*rules about who the agent is obliged to listen to */
%% supervisors
supervisor(evan).

%% admin
admin(evan).
