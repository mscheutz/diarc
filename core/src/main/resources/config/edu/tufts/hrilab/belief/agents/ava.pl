%% agents the system should know about
name(self,ava). %% we might want this to be dynamic too
name(ava,ava).

actor(user).
actor(user1).
actor(user2).
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
supervisor(user1).
supervisor(user2).

%% admin
admin(evan).
admin(user).
admin(user1).
admin(user2).

%% Specific beliefs to ava
alwaysAt(patient,robot).
at(ava,unknown).