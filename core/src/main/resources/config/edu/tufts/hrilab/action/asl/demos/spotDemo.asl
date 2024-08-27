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

() = init[""](String ?ignore = "none()") {
    effects : {
        success : free(?actor);
        success : observableat(bottle:property, location_3:location); //todo: Remove if we want to demonstrate instruction
        success : observable(bottle:property); //todo: Remove if we want to demonstrate instruction
        success : observableat(apple:property, location_3:location);
        success : observable(apple:property); //todo: Remove if we want to demonstrate instruction
        success : at(?actor, !defaultLocation);
    }

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

    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(apple(X:physobj))");
    act:positReference(!props);
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(bottle(X:physobj))");
    act:positReference(!props);
    op:log(debug, "[init] set up spot");
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
    tsc:writeLocationReferencesToFile("/home/pete/dev/diarc/core/src/main/resources/config/edu/tufts/hrilab/spot/vaLocationRefs.json");
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
    act:command(arm_carry);
}

() = putDown() {
    act:goToPutDownPose();
    act:openGripper();
    act:command(arm_carry);
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

    act:detectionPose();
}

() = resetarm(Symbol ?actor:agent) {
    conditions : {
        pre : free(?actor);
    }
    effects : {
        success : not(perceptionpose(?actor));
    }

    act:stow();
}


() = deliver(Symbol ?object:physobj, Symbol ?at:location, Symbol ?type:property) {
    conditions : {
        pre : at(?actor, ?at);
        pre : holding(?actor, ?object);
        pre : property_of(?object, ?type);
    }
    effects : {
        success : not(at(?object, ?at));
        success : free(?actor);
        success : not(holding(?actor, ?object));
        success : delivered(?object, ?at);
        success : not(available(?object));
        success : observable(?type);
    }


    op:log(info, "Delivering ?type ?object to ?at.");

    act:command(arm_carry);
    act:askQuestionFromString(brad, "Let me know when you're holding the ?type.", response(X));
    op:log(info, "Finishing handoff");
    act:openGripper(none);
    op:sleep(3000);
    act:closeGripper(none);
    act:command(arm_stow);
}

() = moveto(Symbol ?from:location, Symbol ?to:location) {
    conditions : {
        pre : at(?actor, ?from);
    }
    effects : {
        success : at(?actor, ?to);
        success infer: not(at(?actor, ?from));
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
    act:deliver_to(physobj_1:physobj,location_1:location);
}

() = goToPutDownPose() {
//    String !pose = "1.1 0 0.35 -0.7071 0 0 0.7071";
    String !pose = "1.1 0 0.4 -0.7071 0 0 0.7071";
    act:goToArmPose(!pose, "body");
}
