import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import java.util.List;

(Symbol ?location) = getCurrentLocation() {
    Predicate !queryPred;
    List !bindings;

    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        (?location) =op:get(!elem,!bindingVar);
    }
}

(Symbol ?physobj) = getHeldObject() {
    Predicate !queryPred;
    List !bindings;

    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        //TODO: bug where empty array (only sometimes) still returns true?
        !elem = op:get(!bindings, 0);
        (?physobj) =op:get(!elem,!bindingVar);
    } else {
        ?physobj = op:setNull();
    }
}

(Symbol ?pose) = getObjectPose(Symbol ?physobj) {
    Predicate !queryPred;
    List !bindings;

    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?physobj,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        (?pose) =op:get(!elem,!bindingVar);
    }
}

(java.lang.Boolean ?holding) = checkHolding(Symbol ?physobj) {
    Predicate !queryPred;
    List !bindings;

    ?holding = op:newObject("java.lang.Boolean", false);

    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor,?physobj)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        ?holding = op:newObject("java.lang.Boolean", true);
    }
}

//EW: "poseToAvoid" is a weird/incomplete way of implementing this - rewrite
(Symbol ?pose) = getPoseWithSpace(Symbol ?poseToAvoid) {
    java.lang.Integer !numObjects;
    java.lang.Integer !poseIndex = 0;
    java.lang.Boolean !doneChecking = false;
    List !poseDescriptors;
    java.lang.Integer !numberOfPoses;
    java.lang.Integer !candidateIndex = -1;

    op:log(debug,"getPoseWithSpace");

    !poseDescriptors = act:getPoseDescriptors();
    !numberOfPoses = op:invokeMethod(!poseDescriptors, "size");

    while (op:equals(!doneChecking, false)) {
        if (op:ge(!poseIndex,!numberOfPoses)) {
            if (op:equals(!candidateIndex,-1)) {
                ?pose = op:setNull();
            } else {
                ?pose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "pose_!candidateIndex:pose");
            }
            !doneChecking = op:newObject("java.lang.Boolean", true);
        } else {
            ?pose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "pose_!poseIndex:pose");
            !numObjects = act:getNumberOfObjectsAt(?pose);
            if (op:lt(!numObjects,1)) {
                //choose location with 0 current blocks ideally
                !doneChecking = op:newObject("java.lang.Boolean", true);
            } elseif (op:lt(!numObjects,2) && ~op:equals(?pose,?poseToAvoid)) {
                if (op:equals(!candidateIndex,-1)) {
                    !candidateIndex = op:newObject("java.lang.Integer", !poseIndex);
                }
            }
            !poseIndex = op:++(!poseIndex);
        }
    }
}

(java.lang.Integer ?numObjects) = getNumberOfObjectsAt(Symbol ?pose) {
    Predicate !queryPred;
    List !bindings;
    java.util.Map !binding;
    Symbol !object;
    Symbol !objectDescriptor;
    edu.tufts.hrilab.fol.Variable !objectVar;

    ?numObjects = op:newObject("java.lang.Integer", 0);

    !objectVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(X,?pose)");
    !bindings = act:queryBelief(!queryPred);
    foreach (!binding : !bindings) {
        !object = op:get(!binding,!objectVar);
        !objectDescriptor = tsc:getDescriptorForID(!object);
        if (~op:equals(!objectDescriptor,none)) {
            ?numObjects = op:++(?numObjects);
        }
    }
}

() = believeObjectAt(Symbol ?object, Symbol ?pose) {
    Predicate !fact;
    Symbol !holding;

    act:assertIsPose(?pose);
    act:assertIsPhysobj(?object);

    !holding = act:checkHolding(?object);
    if (op:equalsValue(!holding,true)) {
        !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "free(?actor)");
        act:assertBelief(!fact);
    }
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "holding(?actor,?object)");
    act:retractBelief(!fact);
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object,X)");
    act:retractBelief(!fact);
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object,?pose)");
    act:assertBelief(!fact);
}

() = assertIsPhysobj(Symbol ?ref) {
    java.lang.String !type;
    if (op:isNull(?ref)) {
        exit(FAIL,notAnObject(null));
    }
    !type = op:invokeMethod(?ref,"getType");
    if (~op:equals("physobj",!type)) {
        exit(FAIL,notAnObject(?ref));
    }
}

() = assertIsPose(Symbol ?ref) {
    java.lang.String !type;
    if (op:isNull(?ref)) {
        exit(FAIL,notAPose(null));
    }
    !type = op:invokeMethod(?ref,"getType");
    if (~op:equals("pose",!type)) {
        exit(FAIL,notAPose(?ref));
    }
}