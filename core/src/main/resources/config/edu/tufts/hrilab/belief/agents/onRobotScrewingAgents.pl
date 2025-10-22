%% agents the system should know about.
name(robotone,robotone).

actor(brad).
actor(james).

diarcAgent(self).
diarcAgent(robotone).

object(self,agent).

memberOf(X,X).
memberOf(robotone, self).
team(self).

/*rules about who the agent is obliged to listen to */
%% supervisors
supervisor(brad).
supervisor(pete).
supervisor(james).
supervisor(user).
supervisor(eric).
supervisor(たたら).
supervisor(佐藤).
supervisor(鈴木).
supervisor(高橋).

%% admin
admin(brad).
admin(james).
admin(eric).
admin(user).
admin(たたら).
admin(佐藤).
admin(鈴木).
admin(高橋).