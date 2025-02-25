%% agents the system should know about
name(self,ava). %% we might want this to be dynamic too
name(ava,ava).

actor(user).
actor(evan).
actor(patient).

diarcAgent(self).
diarcAgent(ava).

team(self).
memberOf(X,X).
memberOf(ava, self).
object(ava, agent).
object(self, agent).

/*rules about who the agent is obliged to listen to */
%% supervisors
supervisor(evan).
supervisor(patient).
supervisor(user).

%% admin
admin(evan).

%% Specific beliefs to ava
alwaysAt(patient,robot).
at(ava,unknown).