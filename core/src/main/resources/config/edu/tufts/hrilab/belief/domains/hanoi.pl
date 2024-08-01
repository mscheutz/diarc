%% Types
subtype(var, object).
subtype(concept, var).
subtype(physical, var).

subtype(stackable, physical).
subtype(disc, stackable).
subtype(peg, stackable).

%% Objects

object(peg1, stackable).
object(peg2, stackable).
object(peg3, stackable).
object(cube1, disc).
object(cube2, disc).
object(cube3, disc).

%% Preds
smaller(peg1, cube1).
smaller(peg1, cube2).
smaller(peg1, cube3).
smaller(peg2, cube1).
smaller(peg2, cube2).
smaller(peg2, cube3).
smaller(peg3, cube1).
smaller(peg3, cube2).
smaller(peg3, cube3).
smaller(cube2, cube1).
smaller(cube3, cube1).
smaller(cube3, cube2).
clear(peg2).
clear(peg3).
clear(cube1).
on(cube3, peg1).
on(cube2, cube3).
on(cube1, cube2).

%% Faces
free(self).
over(self, peg2).