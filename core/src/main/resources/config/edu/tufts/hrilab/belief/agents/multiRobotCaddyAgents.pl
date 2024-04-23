%% agents the system should know about
name(self,self). %% we might want this to be dynamic too
name(fetch,fetch).
name(spot,spot).
name(temi,temi).

actor(evan).
actor(eric).

diarcAgent(self).
diarcAgent(fetch).
diarcAgent(spot).
diarcAgent(temi).
object(temi, temi).
object(fetch, fetch).
object(spot, spot).
object(self, agent).
team(self).

/*rules about who the agent is obliged to listen to */
%% supervisors
role(evan,supervisor(X)):-diarcAgent(X).
role(eric,supervisor(X)):-diarcAgent(X).

%% admin
role(evan,admin(X)):-diarcAgent(X).
role(eric,admin(X)):-diarcAgent(X).

%memberOf(X,Z):-memberOf(X,Y),memberOf(Y,Z). %todo: this transitivity rule breaks inference.
memberOf(X,X). %todo: is this safe for tuprolog inference?
memberOf(fetch, self).
memberOf(spot, self).
memberOf(temi, self).
%if a member is a diarc agent should you be a diarc agent?
%diarcAgent(X) :- memberOf(Y,X),diarcAgent(Y).
