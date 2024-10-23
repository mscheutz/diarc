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

(Symbol ?pose) = getPoseWithSpace() {
    java.lang.Integer !numObjects;
    java.lang.Integer !poseIndex = 0;
    java.lang.Boolean !doneChecking = false;

    op:log(debug,"getPoseWithSpace");

    while (op:equals(!doneChecking, false)) {
        //TODO: Don't have number of locations hardcoded
        if (op:gt(!poseIndex,2)) {
            ?pose = op:setNull();
            !doneChecking = op:newObject("java.lang.Boolean", true);
        } else {
            ?pose = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "pose_!poseIndex:pose");
            !numObjects = act:getNumberOfObjectsAt(?pose);
            if (op:lt(!numObjects,2)) {
                !doneChecking = op:newObject("java.lang.Boolean", true);
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