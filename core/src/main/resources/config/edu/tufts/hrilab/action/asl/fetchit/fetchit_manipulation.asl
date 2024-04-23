import edu.tufts.hrilab.fol.Symbol;

() = placeIn(Symbol ?objectRef_0, Symbol ?objectRef_1, Symbol ?arm="arm") {
    conditions : {
      pre : holding(?actor,?objectRef_0, ?arm);
      //pre : grasping(?actor,?objectRef_0, ?arm);
    //  pre : see(?actor, ?objectRef_1);
    }
    effects : {
      success infer : in(?objectRef_0, ?objectRef_1);
      success : not(holding(?actor,?objectRef_0, ?arm));
    }
    act:findObject(?objectRef_1);
    act:moveObjectAbove(?objectRef_0, ?objectRef_1, ?arm);
    act:releaseObject(?objectRef_0);
    op:log("debug", "[placeIn] successfully placed ?objectRef_0 in ?objectRef_1");
}

() = placeOn(Symbol ?objectRef_0, Symbol ?objectRef_1, Symbol ?arm="arm") {
    conditions : {
      pre : holding(?actor,?objectRef_0, ?arm);
    }
    effects : {
      success infer : on(?objectRef_0, ?objectRef_1);
      success : not(holding(?actor,?objectRef_0,?arm));
      //success infer : not(grasping(?actor,?objectRef_0,?arm));
      //success infer : not(touching(?actor,?objectRef_0));
    }

    act:goToPose(dropOff);
    act:moveObject(?objectRef_0, ?arm, down);
    act:releaseObject(?objectRef_0);
    op:log("debug", "[placeOn] successfully placed ?objectRef_0 on ?objectRef_1");
}

