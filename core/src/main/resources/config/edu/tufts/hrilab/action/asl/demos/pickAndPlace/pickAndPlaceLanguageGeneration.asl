import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import java.lang.String;
import java.util.Map;

//TODO: Can refactor and reuse many of these actions, didn't have time and wrote a bunch of overlapping cases
(Symbol ?objectPose) = askForObjectPose(Symbol ?descriptor) {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"I dont know where the ?descriptor is, at what pose can I expect to find it?", pose(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?objectPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?objectPose = op:get(!bindings,!x);
        act:assertIsPose(?objectPose);
    }
}

(Symbol ?object) = askForObjectToPickUp() {
    Variable !x = "X";
    Map !bindings;

    //TODO: get descriptors for all detected objects and add into question specifically
    !bindings = act:askQuestionFromString(?actor, "I see multiple objects, which one do you want me to pick up?", objectType(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?object = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?object = op:get(!bindings,!x);
        act:assertIsPhysobj(?object);
    }
}

() = reportNotPickingUpObject() {
    act:generateResponseFromString("okay, I am not picking up any object");
}

(Symbol ?response) = reportFullLocationAndAskToSwap() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"I cannot put down the object because the location is full. Would you like me to swap out one of the objects?", val(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        ?response = op:get(!bindings,!x);
    }
}

(Symbol ?object) = askForObjectToSwap() {
    Map !bindings;
    Variable !x = "X";

    !bindings = act:askQuestionFromString(?actor,"Which object should be swapped?", objectType(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?object = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?object = op:get(!bindings, !x);
        act:assertIsPhysobj(?object);
    }
}

() = reportNotSwappingObjects() {
    act:generateResponseFromString("okay, I am not swapping any objects");
}

() = reportHoldingObject(Symbol ?descriptor) {
    act:generateResponseFromString("I am holding the ?descriptor");
}

() = reportObjectLocationUnknown(Symbol ?descriptor) {
    act:generateResponseFromString("I do not know where the ?descriptor is");
}

() = reportObjectIsAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    act:generateResponseFromString("the ?objectDescriptor is at ?poseDescriptor");
}

() = reportObjectLocationsAllUnknown() {
    act:generateResponseFromString("I don't know the location of any objects");
}

() = reportObjectExists(Symbol ?descriptor) {
    act:generateResponseFromString("There is a ?descriptor");
}

() = reportNoObjectsAtPose(Symbol ?descriptor) {
    act:generateResponseFromString("There are no objects at ?descriptor");
}

() = reportNumberOfObjectsAt(java.lang.Integer ?numObjects, Symbol ?descriptor) {
    if (op:equals(?numObjects,1)) {
        act:generateResponseFromString("There is ?numObjects object at ?descriptor");
    } else {
        act:generateResponseFromString("There are ?numObjects objects at ?descriptor");
    }
}

() = reportGoalStatus(java.lang.String ?status) {
    act:generateResponseFromString("?status goal: ");
}

() = reportNoCurrentGoal() {
    act:generateResponseFromString("no current goal");
}

() = reportNoPendingGoal() {
   act:generateResponseFromString("no pending goals");
}

() = reportCurrentPose(Symbol ?armPose) {
   act:generateResponseFromString("I am currently at ?armPose");
}

() = reportObjectToSwapNotAtCurrentPose() {
   act:generateResponseFromString("The object you indicated is not at my current pose");
}

() = reportObjectsToSwapAreTheSame() {
   act:generateResponseFromString("You asked me to swap an object with itself. Doing nothing");
}

() = reportDetectedObject(Symbol ?objectDescriptor) {
    act:generateResponseFromString("yes, I see ?objectDescriptor");
}

() = reportCantVerifyBecauseActiveGoal() {
    act:generateResponseFromString("I cannot verify that because I am currently doing something");
}

() = reportObjectNotBelievedToBeAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    act:generateResponseFromString("I do not believe ?objectDescriptor is at ?poseDescriptor");
}

() = reportObjectBelievedToBeAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    act:generateResponseFromString("I believe ?objectDescriptor is at ?poseDescriptor");
}

() = reportObjectNotAtPose(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    act:generateResponseFromString("I do not see ?objectDescriptor at ?poseDescriptor");
}

() = reportDetectedObject(Symbol ?objectDescriptor, Symbol ?poseDescriptor) {
    act:generateResponseFromString("I see ?objectDescriptor at ?poseDescriptor");
}

(Symbol ?response) = askToVerifyBelief() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"Would you like me to verify that?", val(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        ?response = op:get(!bindings,!x);
    }
}

() = reportDontBelieveObjectsAtPose(Symbol ?poseDescriptor) {
    act:generateResponseFromString("I do not believe any objects are at ?poseDescriptor");
}

() = reportBelieveObjectsAtPose(Symbol ?poseDescriptor) {
    act:generateResponseFromString("I believe objects are at ?poseDescriptor");
}

() = reportObjectsNotAtPose(Symbol ?poseDescriptor) {
    act:generateResponseFromString("I do not see any objects at ?poseDescriptor");
}

() = reportDetectedObjects(Symbol ?poseDescriptor) {
    act:generateResponseFromString("I see objects at ?poseDescriptor");
}

(Symbol ?response) = reportHoldingObjectAndAskToPutDown() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"I cannot pick up the object because I am holding something. Would you like me to put it down somewhere?", val(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "no");
    } else {
        ?response = op:get(!bindings,!x);
    }
}

(Symbol ?objectPose) = askForPose() {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"Where should I put it down?", pose(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?objectPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?objectPose = op:get(!bindings,!x);
        act:assertIsPose(?objectPose);
    }
}

() = reportKnownObjects(java.util.List ?objects) {
    String !descriptor;
    foreach (!descriptor : ?objects) {
        act:generateResponseFromString(!descriptor);
    }
}

() = reportKnownPoses(java.util.List ?poses) {
    String !descriptor;
    foreach (!descriptor : ?poses) {
        act:generateResponseFromString(!descriptor);
    }
}

() = reportSeenObject(Symbol ?descriptor) {
    act:generateResponseFromString("I see ?descriptor");
}

() = reportObjectsAtSameLocation() {
    act:generateResponseFromString("The objects are already at the same pose");
}

(Symbol ?objectPose) = reportFailedDetectionAndAskForObjectPose(Symbol ?descriptor) {
    Map !bindings;
    Variable !x = "X";

    (!bindings) = act:askQuestionFromString(?actor,"I cannot see ?descriptor, what pose is it located at?", pose(X));
    if (op:invokeMethod(!bindings, "isEmpty")) {
        ?objectPose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "none");
    } else {
        ?objectPose = op:get(!bindings,!x);
        act:assertIsPose(?objectPose);
    }
}