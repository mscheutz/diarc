%% agents the system should know about
name(self,ava). %% we might want this to be dynamic too
name(ava,ava).

actor(webapp).
actor(asrapp).
actor(local_user).
actor(local_visitor).

diarcAgent(self).
diarcAgent(ava).

team(self).
memberOf(X,X).
memberOf(ava, self).
object(ava, agent).
object(self, agent).

/*rules about who the agent is obliged to listen to */
%% supervisors
supervisor(webapp).
supervisor(asrapp).
supervisor(local_user).
supervisor(local_visitor).

%% admin
admin(webapp).
admin(asrapp).
admin(local_user).

%% Specific beliefs to ava
alwaysAt(patient,robot).
at(ava,unknown).