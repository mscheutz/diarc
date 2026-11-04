% Hard-coded facts for testing proof-of-concept kit-assembly-only planning domain
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

%TODO: make these assertions in the setup script?
object(tableA, area).
object(tableB, area).
object(tableC, area).
object(tableD, area).
object(tableE, area).
object(tableF, area).
object(tableG, area).

%TODO: where is room information populated?
object(room1, room).
object(room2, room).
object(room3, room).

connected(room1, room2).
connected(room2, room1).
connected(room3, room1).
connected(room1, room3).
connected(room1,room1).
connected(room2,room2).
sealed(room1, room2).
sealed(room2, room1).
%sealed(room1, room3).
%sealed(room3, room1).

