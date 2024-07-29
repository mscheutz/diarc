import java.lang.String;
import java.lang.Integer;
import java.lang.Double;
import java.util.List;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;
import java.util.Map;
import java.util.HashMap;
import edu.tufts.hrilab.cognex.consultant.CognexJob;
import edu.tufts.hrilab.cognex.consultant.CognexResult;
import edu.tufts.hrilab.cognex.consultant.CognexReference;
//======================bind descriptor to a cognex job ============
() = defineItem["defines new item, and asks for relevant parameters"](edu.tufts.hrilab.fol.Symbol ?item){
  Symbol !jobID;
  Variable !x = "X";
  Map !bindings;
  //TODO:brad: how do we determine the addressee here?
  Symbol !addressee="james";

  (!bindings) = act:askQuestionFromString(!addressee,"which Cognex job is used to detect it", job(X));
  (!jobID) = op:get(!bindings, !x);
  tsc:addDetectionType(?item,!jobID);

  act:generateResponseFromString("okay");
}

() = verifyYouCanSee["Looks for an entity at the current location"](edu.tufts.hrilab.fol.Symbol ?refID) {
    Symbol !currPose = "current";

    effects : {
        success : at(?refID, !currPose);
    }

    Predicate !queryPred;
    List !bindings;
    HashMap !binding;
    Variable !bindingVar;

    Symbol !descriptor;
    Symbol !distSymbol;
    Double !dist;
    Symbol !xDistSymbol;
    Double !xDist;

    Double !downDist;
    Double !backDist;

    op:log(debug, "[perceiveEntity] refID: ?refID");

    !descriptor = tsc:getDescriptorForID(?refID);
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate","objectDefinition(!descriptor,A,B)");
    !bindings = act:queryBelief(!queryPred);
    op:log(debug, "[perceiveEntity] descriptor: !descriptor ::: queryPred: !queryPred");

    !bindingVar =op:newObject("edu.tufts.hrilab.fol.Variable","A");
    if (~op:isEmpty(!bindings)) {
        (!binding) = op:get(!bindings, 0);
        (!distSymbol) = op:get(!binding,!bindingVar);
    }
    op:log(debug, "[perceiveEntity] bindingVar: !bindingVar :::::: binding: !binding");
    !dist = op:/(!distSymbol,1);
    !downDist = op:*(!dist,-1);

    !bindingVar =op:newObject("edu.tufts.hrilab.fol.Variable","B");
    if (~op:isEmpty(!bindings)) {
        !binding = op:get(!bindings, 0);
        !xDistSymbol = op:get(!binding,!bindingVar);
    }
    !xDist = op:/(!xDistSymbol,1);
    op:log(debug, "[perceiveEntity] xDist: !xDist:::::: xDistSymbol: !xDistSymbol");
    !backDist = op:*(!xDist,-1);

    tsc:moveZRelative(!dist);
    tsc:moveXRelative(!xDist);

    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !binding = op:get(!bindings, 0);
        !currPose =op:get(!binding,!bindingVar);
    }

    act:perceiveEntityFromSymbol(?refID);
    tsc:moveZRelative(!downDist);
    tsc:moveXRelative(!backDist);
}

() = observeDescriptor["runs a job for the given ?descriptor and saves the results in the cognex consultant"](edu.tufts.hrilab.fol.Symbol ?descriptor, java.lang.Integer ?numResults) {
    //Runtime variables
    Integer !i = 0;
    List !cameraResults;
    String !jobName;
    edu.tufts.hrilab.cognex.consultant.CognexJob !job;

    edu.tufts.hrilab.fol.Predicate !queryPred;
    java.util.List !bindings;
    java.util.HashMap !binding;
    edu.tufts.hrilab.fol.Variable !bindingVar;

    edu.tufts.hrilab.fol.Symbol !distSymbol;
    java.lang.Double !dist;
    edu.tufts.hrilab.fol.Symbol !xDistSymbol;
    java.lang.Double !xDist;

    java.lang.Double !downDist;
    java.lang.Double !backDist;

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","objectDefinition(?descriptor,A,B)");
    !bindings = act:queryBelief(!queryPred);

    !bindingVar =op:newObject("edu.tufts.hrilab.fol.Variable","A");
    if (~op:isEmpty(!bindings)) {
        (!binding) = op:get(!bindings, 0);
        (!distSymbol) = op:get(!binding,!bindingVar);
    }
    op:log(debug, "[observeDescriptor] bindingVar: !bindingVar :::::: binding: !binding");
    !dist = op:/(!distSymbol,1);
    !downDist = op:*(!dist,-1);

    !bindingVar =op:newObject("edu.tufts.hrilab.fol.Variable","B");
    if (~op:isEmpty(!bindings)) {
        !binding = op:get(!bindings, 0);
        !xDistSymbol = op:get(!binding,!bindingVar);
    }
    !xDist = op:/(!xDistSymbol,1);
    op:log(debug, "[observeDescriptor] xDist: !xDist:::::: xDistSymbol: !xDistSymbol");
    !backDist = op:*(!xDist,-1);

    //prepare by moving relevant part of circuit breaker in view
    tsc:moveZRelative(!dist);
    tsc:moveXRelative(!xDist);
    !job = tsc:getCognexJobForDescriptor(?descriptor);
    !jobName = op:invokeMethod(!job, "getName");
    !cameraResults = tsc:getCameraData(!jobName);
    act:bindResultsRecursive(!job, !cameraResults, !i);
    tsc:moveZRelative(!downDist);
    tsc:moveXRelative(!backDist);
}

() = bindResultsRecursive["Recursively iterates through the given ?cameraResults and binds them to references"](CognexJob ?job, java.util.List ?cameraResults, java.lang.Integer ?i) {
    List !additionalProps;
    Predicate !leftPred;
    Predicate !rightPred;
    Integer !size;
    Integer !first = 0;
    Integer !iCopy;

    CognexResult !result;
    CognexReference !ref;

    if (op:isEmpty(?cameraResults)) {
        return;
    } else {
        (!additionalProps) = tsc:getEmptyProps();
        (!result) = op:invokeMethod(?cameraResults, "remove", !first);
        (!size) = op:size(?cameraResults);
        op:log("info","[bindResultsRecursive] !size results left!");
        //create reference and add any additional props
        (!ref) = tsc:createCogRefWithProps(?job, !additionalProps);
        //bind reference to the result that it matches
        (!iCopy) = op:newObject("java.lang.Integer", ?i);
        tsc:bindCognexResult(!ref, !result, !iCopy);
        (?i) = op:++(?i);
        act:bindResultsRecursive(?job, ?cameraResults, ?i);
    }
}

(edu.tufts.hrilab.fol.Symbol ?return) = getRefForJob["runs a job for the given ?descriptor and saves and returns the first result"](edu.tufts.hrilab.fol.Symbol ?descriptor) {
    List !cameraResults;
    String !jobName;
    CognexJob !job;
    CognexResult !result;
    CognexReference !ref;
    List !additionalProps;

    (!job) = tsc:getCognexJobForDescriptor(?descriptor);
    (!jobName) = op:invokeMethod(!job, "getName");
    op:log(debug, "Job for ?descriptor: !jobName");
    (!cameraResults) = tsc:getCameraData(!jobName);
    if (op:isEmpty(!cameraResults)) {
        (?return) = op:newObject("edu.tufts.hrilab.fol.Symbol", "error");
    } else {
        (!result) = op:get(!cameraResults, 0);
        //create reference and add any additional props
        (!additionalProps) = tsc:getEmptyProps();
        (!ref) = tsc:createCogRefWithProps(!job, !additionalProps);
        //bind reference to the result that it matches
        tsc:bindCognexResult(!ref, !result, 0);
        (?return) = op:invokeMethod(!ref, "getRefId");
    }
}
