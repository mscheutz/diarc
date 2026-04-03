import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import java.lang.String;
import java.lang.Integer;
import java.lang.Boolean;
import java.util.List;


//////// Core behaviors //////////

() = init(Symbol ?location = location_0:location) {

    act:initFacts();

    act:claim();
    act:powerOn();

    op:log(debug, "[init] set up spot");
}

() = initFacts() {
    effects : {
        success : free(?actor);
        success : observableat(bottle:property, location_3:location); //todo: Remove if we want to demonstrate instruction
        success : observable(bottle:property); //todo: Remove if we want to demonstrate instruction
        success : observableat(cup:property, location_3:location);
        success : observable(cup:property); //todo: Remove if we want to demonstrate instruction
        success : observableat(unknown:property, location_3:location);
        success : observable(unknown:property); //todo: Remove if we want to demonstrate instruction
    }

    Term !props;
    Symbol !refId;

    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(bottle(X:physobj))");
    act:positReference(!props);
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(cup(X:physobj))");
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

() = shutDown() {
    java.lang.Integer !dockId = 521;

    act:openGripper(arm);
    act:goToPose(stow);
    act:closeGripper(arm);
    act:dock(!dockId);
}

() = stow() {
    effects : {
        success : not(perceptionpose(?actor));
    }

    tsc:goToPose(stow);
}

() = perceiveEntity(Symbol ?object:physobj) {
    op:log(debug, "[verifyYouCanSee] running visual search and looking for ?object");
    try {
      act:startVisualSearch(?object);
      act:findObject(?object);
    } finally {
      act:stopVisualSearch(?object);
    }
}

() = pickUp(Symbol ?object:physobj) {
    act:grab(?object);
//    act:gotocarrypose();
}

() = grab(Symbol ?object:physobj) {
    //act:goToPose(detection);
    act:openGripper(arm);
    act:startVisualSearch(?object);
    try {
      act:findObject(?object);
      //act:pauseCapture();
      act:grabObject(?object);
    } catch() {
      op:log("info", "[grab] IN CATCH BLOCK");
      exit(FAIL);
    } finally {
      op:log("info", "[grab] IN FINALLY BLOCK");
//      act:resumeCapture();
      act:stopVisualSearch(?object);
    }
}

() = putDown() {
    act:goToPose(putDown);
    act:openGripper(arm);
    act:gotocarrypose();
    act:closeGripper(arm);
    act:stow();
}

///////// Operators

() = gotoperceptionpose(Symbol ?actor:agent) {
    conditions : {
        pre : not(perceptionpose(?actor));
    }
    effects : {
        success : perceptionpose(?actor);
    }

    act:goToPose(detection);
}

() = gotocarrypose() {
    effects : {
        success : not(perceptionpose(?actor));
    }

    act:goToPose(carry);
}

() = deliveritem(Symbol ?object:physobj, Symbol ?at:location, Symbol ?type:property) {
    String !typeString;
    conditions : {
        pre : at(?actor, ?at);
        pre : holding(?actor, ?object);
        //pre : property_of(?object, ?type);
    }
    effects : {
        success : not(at(?object, ?at));
        success : free(?actor);
        success : not(holding(?actor, ?object));
        success : delivered(?object, ?at);
        success : not(available(?object));
        //success : observable(?type);
    }


    op:log(info, "Delivering ?type ?object to ?at.");
    act:gotocarrypose();
//    act:goToPose(putDown);
    act:askQuestionFromString(pete, "Let me know when you're holding the object.", holding(X));
    op:log(info, "Finishing handoff");
    act:openGripper(arm);
    op:sleep(500);
    tsc:closeGripper();
    tsc:goToPose(stow);
}

() = moveto(Symbol ?from:location, Symbol ?to:location) {
    conditions : {
        pre : at(?actor, ?from);
        pre : not(perceptionpose(?actor));
    }
    effects : {
        success : not(at(?actor, ?from));
        success : at(?actor, ?to);
    }
    op:log(info, "Moving from ?from to ?to.");
    act:goToLocation(?to);
}

() = pickup(Symbol ?object:physobj, Symbol ?from:location) {
    conditions : {
        pre : at(?actor, ?from);
        pre : at(?object, ?from);
        pre : free(?actor);
        pre : perceptionpose(?actor);
    }
    effects : {
        success : not(at(?object, ?from));
        success : not(free(?actor));
        success : holding(?actor, ?object);
        success : not(perceptionpose(?actor));
    }
    op:log(info, "Picking up ?object from ?from.");
    act:pickUp(?object);
    op:log(info, "Picked up ?object from ?from.");
    // act:goto(?from); // EAK: why was this here?
}

() = putdown(Symbol ?object:physobj, Symbol ?to:location) {
    conditions : {
        pre : at(?actor, ?to);
        pre : holding(?actor, ?object);
    }
    effects : {
        success : free(?actor);
        success : not(holding(?actor, ?object));
        success : at(?object, ?to);
        success : not(holding(?actor, ?object));
    }
    op:log(info, "Putting down ?object at ?to.");
    act:putDown();
    // act:goto(?to); // EAK: why was this here?
}


() = perceiveentity(Symbol ?object:physobj, Symbol ?type:property, Symbol ?location:location) {
    conditions : {
        pre : at(?actor, ?location);
        pre : observableat(?type, ?location);
        pre : observable(?type);
        pre : property_of(?object, ?type);
        pre : perceptionpose(?actor);
    }
    effects : {
        success : at(?object, ?location);
        success : available(?object);
        success : not(observable(?type));
    }
    op:log(info, "Perceiving ?type ?object at ?location");
    act:perceiveEntity(?object);
}

() = runDemo() {
    act:deliver(physobj_1:physobj,location_1:location);
}
