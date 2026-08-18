import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.lang.String;
import java.lang.Integer;
import java.lang.Boolean;
import java.util.List;

() = init(Symbol ?location = location_0:location) {
    effects : {
        success : free();
    }
    op:log(debug, "[init] setting up spot");
    act:undock();
    //act:localizeWithFiducial();
    act:localizeWithReference(?location);
    act:goToLocation(?location);
    act:gotocarrypose();
    act:dock();

}
() = shutDown() {
    act:openGripper(arm);
    act:goToPose(stow);
    act:closeGripper(arm);
    act:goToLocation(location_0:location);
    act:dock();
}

() = initFacts() {
    effects : {
        success : free();
        success : observableat(cup:property, spotlocation_1:location);
        success : observable(cup:property);
        success : observableat(human:property, spotlocation_2:location);
        success : observable(human:property); 
        success : observableat(unknown:property, location_3:location);
        success : observable(unknown:property); //todo: Remove if we want to demonstrate instruction
    }

    Term !props;
    Symbol !refId;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "human(X:physobj)");
    act:positReference(!props);
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cup(X:physobj)");
    act:positReference(!props);
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(unknown(X:physobj))");
    act:positReference(!props);
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(object(X:physobj))");
    act:positReference(!props);
  
}

() = localize(Symbol ?location = location_0:location, boolean ?useFiducial = false) {
    effects : {
      success : not(at(?actor, X));
      success : at(?actor, ?location);
    }

    op:log(debug, "[localize] localizing to: ?location");
    if (op:equals(?useFiducial, true)) {
      act:localizeWithFiducial();
    } else {
      act:localizeWithReference(?location);
    }
}

() = moveto(Symbol ?from:location, Symbol ?to:location) {
    conditions : {
        pre : at(?actor, ?from);
        pre : not(perceptionpose());
    }
    effects : {
        success : not(at(?actor, ?from));
        success : at(?actor, ?to);
    }
    op:log(info, "Moving from ?from to ?to.");
    act:goToLocation(?to);
}

//correct syntax for picking up a cup is using cup(cup1) as the argument
() = pickup(Symbol ?object:physobj, Symbol ?from:location) {
    conditions : {
        pre : at(?actor, ?from);
        pre : at(?object, ?from);
        pre : free();
        pre : perceptionpose();
    }
    effects : {
        success : not(at(?object, ?from));
        success : not(free());
        success : holding(?actor, ?object);
        success : not(perceptionpose());
    }
    act:gotoperceptionpose();
    op:log(info, "Picking up ?object from ?from.");
    act:pickUp(?object);
    op:log(info, "Picked up ?object from ?from.");

}

() = grab(Symbol ?object:physobj) {
    //act:goToPose(detection);
    act:openGripper(arm);
    act:startVisualSearch(?object);
    try {
      act:findObject(?object);
      //act:pauseCapture();
      op:log("info", "[grab] calling grabObject...");
      act:grabObject(?object);
      op:log("info", "[grab] grabObject returned");
    } catch() {
      op:log("info", "[grab] IN CATCH BLOCK");
      exit(FAIL);
    } finally {
      op:log("info", "[grab] IN FINALLY BLOCK");
//      act:resumeCapture();
      act:stopVisualSearch(?object);
    }
}

() = pickUp(Symbol ?object:physobj) {
    act:grab(?object);
//    act:gotocarrypose();
}

() = gotoperceptionpose() {
    effects : {
        success : perceptionpose();
    }

    act:goToPose(detection);
}

() = gotocarrypose() {
    effects : {
        success : not(perceptionpose());
    }

    act:goToPose(carry);
}
