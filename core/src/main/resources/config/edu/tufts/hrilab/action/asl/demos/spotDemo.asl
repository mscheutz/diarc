import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import java.lang.String;
import java.lang.Integer;
import java.lang.Boolean;
import java.util.List;


/////////// Recovery Policies /////////
////todo: this doesn't actually work, since it will happily localize to the wrong place? if we sit down etc it assumes we're closest to the dock location where the tour started i think.
() = failedLocalization(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {
  Symbol !defaultLocation = location_0:location;

  recovery: {
    failedActions: {init(self:agent)}
    actionStatuses: {FAIL_RETURNVALUE}
  }

    act:setInitialLocation(!defaultLocation);
//  act:setLocationWithFiducial(!defaultLocation;
  op:log(warn, "[failedLocalization] location initialized to !defaultLocation after failing to localize");
}

() = failedGrasp(Predicate ?failedActionPredicate, List ?failureReasons, Predicate ?goal) {
  Symbol !defaultLocation = location_0:location;

  recovery: {
//    failedActions: {pickUpObject(self:agent, X:physobj)}
    failureReasons: {failedGrasp(X)}
//    actionStatuses: {FAIL}
  }

//    act:setInitialLocation(!defaultLocation);
//  act:setLocationWithFiducial(!defaultLocation);
  op:log(warn, "[failedGrasp] trying grasp again");
}


//////// Core behaviors //////////

() = init[""](String ?ignore = "none()") {
    Symbol !defaultLocation = location_0:location;

    effects : {
        success : free(?actor);
        success : observableat(bottle:property, location_23:location); //todo: Remove if we want to demonstrate instruction
        success : observable(bottle:property); //todo: Remove if we want to demonstrate instruction
        success : observableat(apple:property, location_23:location);
        success : observable(apple:property); //todo: Remove if we want to demonstrate instruction
        success : observableat(unknown:property, location_23:location);
        success : observable(unknown:property); //todo: Remove if we want to demonstrate instruction
        success : at(?actor, !defaultLocation);
    }

    Boolean !result;

    act:claim();
    act:powerOn();
    act:openGripper();
    act:closeGripper();
    tsc:stow();
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

    //!props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(apple(X:physobj))");
    //act:positReference(!props);
    //!props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(bottle(X:physobj))");
    //act:positReference(!props);

    op:log(debug, "[init] set up spot");
}

() = relocalize() {
  Symbol !defaultLocation = location_0:location;

  act:setLocationWithFiducial(!defaultLocation);
}


() = map() {
    Symbol !location;
    Integer !numLocations;
//    java.util.Map !bindings;

    act:getNavGraphPointsAndPopulateConsultant();
    !numLocations = tsc:getNumberOfKnownLocations();
    op:log(debug, "[map] number of known locations is !numLocations");


    for (!i=0; !i lt !numLocations; !i ++) {
        !location = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "location_!i:location");
        act:goToLocationUnsafe(!location);
        act:bindLocationToCurrentPose(!location);
        op:log(debug, "[map] mapped !location");
    }
    tsc:writeLocationReferencesToFile("/home/pete/dev/diarc/core/src/main/resources/config/edu/tufts/hrilab/spot/room.json");
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

() = stow() {
    effects : {
        success : not(perceptionpose(?actor));
    }

    tsc:stow();
}

() = openGripper() {
//    Predicate !queryPred;
//    Symbol !object;
//    Predicate !toRetract;
//    Predicate !free = free(?actor);
//
//    //todo: this shouldn't need to be done this way. should be handled in effects, etc
//    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor, X)");
//    !object = act:getBinding(!queryPred);
//    if (~op:equalsValue(!object, none)) {
//        !toRetract = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor, !object");
//        op:log(debug, "[openGripper] found that holding(?actor, !object), retracting since we're opening gripper");
//        act:retractBelief(!toRetract);
//        act:assertBelief(!free);
//    }
    tsc:openGripper();
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
    tsc:openGripper();
    act:startVisualSearch(?object);
    act:findObject(?object);
    act:pickUpObject(?object);
    act:stopVisualSearch(?object);
    act:gotocarrypose();
}

() = putDown() {
    act:goToPutDownPose();
    tsc:openGripper();
    act:gotocarrypose();
    act:closeGripper();
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

() = gotocarrypose() {
    effects : {
        success : not(perceptionpose(?actor));
    }
    act:command(arm_carry);
}

() = deliveritem(Symbol ?object:physobj, Symbol ?at:location, Symbol ?type:property) {
    String !typeString;
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
    !typeString = op:getName(?type);
    act:gotocarrypose();
//    act:goToPutDownPose();
    act:askQuestionFromString(pete, "Let me know when you're holding the !typeString object.", holding(X));
    op:log(info, "Finishing handoff");
    act:openGripper();
    op:sleep(500);
    act:closeGripper();
    act:command(arm_stow);
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
    act:goto(?from);
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
    act:goto(?to);
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

() = goToPutDownPose() {
//    String !pose = "1.1 0 0.35 -0.7071 0 0 0.7071";
    String !pose = "1.1 0 0.4 -0.7071 0 0 0.7071";
    act:goToArmPose(!pose, "body");
}
