%% agents the system should know about
name(self,andy). %% we might want this to be dynamic too
name(andy,andy).

actor(tyler).

diarcAgent(self).
diarcAgent(andy).

/*rules about who the agent is obliged to listen to */
%% supervisors
supervisor(tyler).

%% admin
admin(tyler).