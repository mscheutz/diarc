import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import java.lang.String;
import java.lang.Integer;
import java.lang.Boolean;
import java.util.List;

() = relocalize() {
  Symbol !defaultLocation = location_0:location;

  act:setLocationWithFiducial(!defaultLocation);
}


//todo: this doesn't actually work, since it will happily localize to the wrong place? if we sit down etc it assumes we're closest to the dock location where the tour started i think.
() = failedLocalization(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {
  Symbol !defaultLocation = location_0:location;

  recovery: {
    failedActions: {init(self:agent)}
    actionStatuses: {FAIL_RETURNVALUE}
  }

    act:setInitialLocation(!defaultLocation);
//  act:setLocationWithFiducial(!defaultLocation);
  op:log(warn, "[failedLocalization] location initialized to !defaultLocation after failing to localize");
}

() = test_deliver() {
    act:deliver_to(physobj_0:physobj, location_0:location);
    act:deliver_to(physobj_1:physobj, location_0:location);
    act:deliver_to(physobj_2:physobj, location_0:location);
    act:deliver_to(physobj_3:physobj, location_0:location);
}

() = init[""]() {
    Symbol !defaultLocation = location_0:location;
    Boolean !result;

    act:claim();
    act:powerOn();
    act:openGripper();
    act:closeGripper();
    act:stow();
//    act:stand();
//    act:undock();

    !result = act:hasBeenLocalized();

    if (op:equals(!result, true)) {
        op:log(debug, "[init] no need to localize again");
    } else {
//        act:localize();
      act:setLocationWithFiducial(!defaultLocation);
    }

    Term !props;
    Symbol !refId;
    for (!i=0; !i lt 5; !i ++) {
        !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(apple(X:physobj))");
        act:create_deliverable(!props);
    }
    for (!i=0; !i lt 5; !i ++) {
        !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(bottle(X:physobj))");
        act:create_deliverable(!props);
    }
    op:log(debug, "[init] set up spot");
}

() = create_deliverable(Term ?props) {
    effects : {
        success : available(!refId);
    }
    Symbol !refId = act:positReference(?props);
}

() = map() {
    Symbol !location;
    Integer !numLocations;
    !numLocations = tsc:getNumberOfKnownLocations();
    op:log(debug, "[map] number of known locations is !numLocations");
    for (!i=0; !i lt !numLocations; !i ++) {
        !location = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "location_!i:location");
        act:goToLocationUnsafe(!location);
        act:bindLocationToCurrentPose(!location);
        op:log(debug, "[map] mapped !location");
    }
    tsc:writeReferencesToFile("/home/pete/dev/diarc/core/src/main/resources/config/edu/tufts/hrilab/spot/phiLocationRefs.json");
    act:localize();
}

() = wander() {
    Symbol !location;
    Integer !numLocations;
    !numLocations = tsc:getNumberOfKnownLocations();
    for (!i=0; !i lt !numLocations; !i ++) {
        !location = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "location_!i:location");
        act:goToLocation(!location);
        op:log(debug, "[wander] at location !location");
    }
    op:log(info, "[wander] done with tour. at location !location");
}

() = shutDown() {
    java.lang.Integer !dockId = 521;

    act:openGripper();
    act:stow();
    act:closeGripper();
    act:dock(!dockId);
}

() = test() {
    act:map();
    act:wander();
}

() = perceiveEntity(Symbol ?object:physobj) {
    op:log(debug, "[verifyYouCanSee] running visual search and looking for ?object");
    act:startVisualSearch(?object);
    act:findObject(?object);
    act:stopVisualSearch(?object);
}

() = pickUp(Symbol ?object:physobj) {
//    act:command(arm_carry);
    act:openGripper(none);
    act:startVisualSearch(?object);
    act:findObject(?object);
    act:pickUpObject(?object);
    act:stopVisualSearch(?object);
//    act:command(arm_carry);
}

///////// Operators

() = deliver(Symbol ?object:physobj, Symbol ?at:location, Symbol ?type:property) {
    conditions : {
        pre : at(?actor, ?at);
        pre : holding(?actor, ?object);
        pre infer : property_of(?object, ?type); //todo: remove infer
    }
    effects : {
        success : not(at(?object, ?at));
        success : free(?actor);
        success : not(holding(?actor, ?object));
        success : delivered(?object, ?at);
        success : not(available(?object));
    }


    op:log(info, "Delivering ?type ?object to ?at.");

    //act:command(arm_carry);
    //act:askQuestionFromString(brad, "Let me know when you're holding the ?type.", response(X)); //todo: fill out the response
    op:log(info, "Finishing handoff");
//    act:openGripper(none);
//    op:sleep(3000);
//    act:closeGripper(none);
//    act:command(arm_stow);
}

() = moveto(Symbol ?from:location, Symbol ?to:location) {
    conditions : {
        pre : at(?actor, ?from);
    }
    effects : {
        success : at(?actor, ?to);
        success : not(at(?actor, ?from));
    }
    op:log(info, "Moving from ?from to ?to.");
    //act:goToLocation(?to);
}

() = pickup(Symbol ?object:physobj, Symbol ?from:location) {
    conditions : {
        pre : at(?actor, ?from);
        pre : at(?object, ?from);
        pre : free(?actor);
    }
    effects : {
        success : not(at(?object, ?from));
        success : not(free(?actor));
        success : holding(?actor, ?object);
    }
    op:log(info, "Picking up ?object from ?from.");
    //act:pickUp(?object);
}

() = putdown(Symbol ?object:physobj, Symbol ?to:location) {
    conditions : {
        pre : at(?actor, ?to);
        pre : holding(?actor, ?object);
    }
    effects : {
        success : free(?actor);
        success : at(?object, ?to);
        success : not(holding(?actor, ?object));
    }
    op:log(info, "Putting down ?object at ?to.");
    //act:pickUp(?object);
}


() = perceiveentity(Symbol ?object:physobj, Symbol ?type:property, Symbol ?location:location) {
    conditions : {
        pre : at(?actor, ?location);
        pre : observableat(?type, ?location);
        pre infer : property_of(?object, ?type); //todo: remove infer
    }
    effects : {
        success : at(?object, ?location);
    }
    op:log(info, "Perceiving ?type ?object at ?location");
    act:perceiveEntity(?object);
}

