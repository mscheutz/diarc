() = pickup() {
    conditions : {
        pre : gripperOpen(?actor);
    }
    effects : {
        success : holding(?actor);
    }

    op:log("info","[pickup]");
}

() = putdown() {
    conditions : {
        pre : holding(?actor);
    }
    effects : {
        success : not(holding(?actor));
    }
    op:log("info","[putdown]");
}

() = opengripper() {
    conditions : {
        pre : not(holding(?actor));
    }
    effects : {
        success : gripperOpen(?actor);
    }
    op:log("info","[opengripper]");
    act:fakeAction();
    op:log("info","[opengripper] success");
}

() = closegripper() {
    conditions : {
        pre : not(holding(?actor));
    }
    effects : {
        success : not(gripperOpen(?actor));
    }
    op:log("info","[closegripper]");
}

//Only present to include dummy predicate in the domain to make it so init isn't empty
() = fillerAction() {
    effects : {
        success : fillerPredicate(?actor);
    }
    op:log("info","[fillerAction]");
}