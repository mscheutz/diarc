import edu.tufts.hrilab.fol.Symbol;


() = forward(Symbol ?from:location, Symbol ?to:location, Symbol ?fromDir:direction, Symbol ?toDir:direction) {
    conditions : {
        pre : at(?actor, ?from);
        pre : dir(?fromDir);
        pre : connected(?from, ?to, ?fromDir);
        pre : connecting(?from, ?to, ?toDir);
    }
    effects : {
        success : at(?actor, ?to);
        success : not(at(?actor, ?from));

        success : dir(?toDir);
        success : not(dir(?fromDir));

        success : canTurn();
    }
    op:log("info", "forward");
}

() = turnLeft(Symbol ?location:location, Symbol ?junction:junction, Symbol ?from:direction, Symbol ?to:direction) {
    conditions : {
        pre : canTurn();
        pre : ccw(?to, ?from);
        pre : dir(?from);
        pre : junction(?location, ?junction);
        pre : validTurn(?junction, ?from);
    }
    effects : {
        success : dir(?to);
        success : not(dir(?from));
        success : not(canTurn());
    }
    op:log("info", "turnLeft");
}

() = turnRight(Symbol ?location:location, Symbol ?junction:junction, Symbol ?from:direction, Symbol ?to:direction) {
    conditions : {
        pre : canTurn();
        pre : cw(?to, ?from);
        pre : dir(?from);
        pre : junction(?location, ?junction);
        pre : validTurn(?junction, ?from);
    }
    effects : {
        success : dir(?to);
        success : not(dir(?from));
        success : not(canTurn());
    }
    op:log("info", "turnRight");
}