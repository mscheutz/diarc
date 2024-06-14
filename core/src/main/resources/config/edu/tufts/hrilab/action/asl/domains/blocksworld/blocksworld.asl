import edu.tufts.hrilab.fol.Symbol;

() = pickup(Symbol ?obj:physobj) {

    conditions : {
        pre : clear(?obj);
        pre : ontable(?obj);
        pre : handempty(?actor);
    }
    effects : {
        success : not(ontable(?obj));
        success : not(clear(?obj));
        success : not(handempty(?actor));
        success : holding(?actor, ?obj);
    }

    op: log ("info",">> pickup ?obj");

    act:pickUp(?obj);
}

() = putdown(Symbol ?obj:physobj) {

    conditions : {
        pre : holding(?actor, ?obj);
    }
    effects : {
        success : not(holding(?actor, ?obj));
        success : clear(?obj);
        success : handempty(?actor);
        success : ontable(?obj);
    }

    op: log ("info",">> putdown ?obj");
}

() = stack(Symbol ?obj1:physobj, Symbol ?obj2:physobj) {

    conditions : {
        pre : holding(?actor, ?obj1);
        pre : clear(?obj2);
    }
    effects : {
        success : not(holding(?actor, ?obj1));
        success : not(clear(?obj2));
        success : clear(?obj1);
        success : handempty(?actor);
        success : on(?obj1, ?obj2);
    }

    op: log ("info",">> stack ?obj1 on ?obj2");

    act:moveObjectRelativeTo(?obj1, above, ?obj2);
    // TODO: move object down
    // act:moveObjectInDirection(?obj1, down, arm, 0.05);
    act:releaseObject(?obj1);
}

() = unstack(Symbol ?obj1:physobj, Symbol ?obj2:physobj) {

    conditions : {
        pre : on(?obj1, ?obj2);
        pre : clear(?obj1);
        pre : handempty(?actor);
    }
    effects : {
        success : holding(?actor, ?obj1);
        success : clear(?obj2);
        success : not(clear(?obj1));
        success : not(handempty(?actor));
        success : not(on(?obj1, ?obj2));
    }

    op: log ("info",">> unstack ?obj1 from ?obj2");

    act:pickUp(?obj1);
}
