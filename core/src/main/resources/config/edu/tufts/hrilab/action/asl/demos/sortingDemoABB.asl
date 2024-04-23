//The circuitBreakerScrewingDemo includes the scripts used to configure
//and run various screwing primitives (and their compiled versions)
//to fill in different holes on the large (NF32SV and small (NV30FAU) circuit breakers
//====================== Setup action scripts ======================
() = init["workaround for not being able to retract facts from belief init files"](){
    edu.tufts.hrilab.fol.Predicate !fact;
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,pose_0:pose)");
    act:assertBelief(!fact);
}

() = setupPoses() {
    act:init();
}

//====================== Demo specific? ==============================
//TODO: brad should this be moved to a more genral place?
() = recordCameraPoseAsk["records current pose and asks for an off set"](edu.tufts.hrilab.fol.Symbol ?poseName){
  //answer bindings
  java.util.Map !bindings;
  edu.tufts.hrilab.fol.Symbol !z;
  edu.tufts.hrilab.fol.Variable !x = "X";
  //TODO:brad: this is like this for explicit actor based reasons. passing in ?actor for a role other than actor causes issues in ActionContext.java line 86 which I think is itself a work around...
  edu.tufts.hrilab.fol.Symbol !speaker;

 (!speaker) = op:newObject("edu.tufts.hrilab.fol.Symbol",?actor);

//  act:submitTTSRequest(?actor,listener,"okay");

  (!bindings) = act:askQuestionFromString(!speaker,"what is its offset", val(X,Y));
  (!z) = op:get(!bindings, !x);
  act:recordPose(?poseName,!z);

   act:submitTTSRequest(!speaker,!speaker,"okay");
}

() = defineItem["defines new item, and asks for relevant parameters"](edu.tufts.hrilab.fol.Symbol ?item){
  edu.tufts.hrilab.fol.Symbol !jobID;
  edu.tufts.hrilab.fol.Variable !x = "X";
  java.util.Map !bindings;
    //TODO:brad: this is like this for explicit actor based reasons. passing in ?actor for a role other than actor causes issues in ActionContext.java line 86 which I think is itself a work around...
  edu.tufts.hrilab.fol.Symbol !speaker;
  (!speaker) = op:newObject("edu.tufts.hrilab.fol.Symbol",?actor);

//  act:submitTTSRequest(?actor,listener,"okay");

  (!bindings) = act:askQuestionFromString(!speaker,"which Cognex job is used to detect it", job(X));
  (!jobID) = op:get(!bindings, !x);
  tsc:addDetectionType(?item,!jobID);

  act:submitTTSRequest(!speaker,!speaker,"okay");
}

() = runBehavior["todo"]() {
    edu.tufts.hrilab.fol.Symbol !ref;

    act:gotoCamerapose(pose_0:pose);
    //updated pillBottle to have semantic type property
    !ref = act:findObjectOfType(pillBottle);
    act:getOn(!ref, pose_1:pose);
    if (act:itemWeightGreaterThan(100)) {
        act:getOn(!ref, pose_2:pose);
    } else {
        act:getOn(!ref, pose_3:pose);
    }
    act:moveConveyorBackward();
}

(edu.tufts.hrilab.fol.Symbol ?return) = getRefForJob["runs a job for the given ?descriptor and saves and returns the first result"](edu.tufts.hrilab.fol.Symbol ?descriptor) {
    java.util.List !cameraResults;
    java.lang.String !jobName;
    edu.tufts.hrilab.abb.cognex.CognexJob !job;
    edu.tufts.hrilab.abb.cognex.CognexResult !result;
    edu.tufts.hrilab.abb.cognex.CognexReference !ref;
    java.util.List !additionalProps;

    (!job) = act:getCognexJobForDescriptor(?descriptor);
    (!jobName) = op:invokeMethod(!job, "getName");
    op:log(debug, "Job for ?descriptor: !jobName");
    (!cameraResults) = act:getCameraData(!jobName);
    if (op:isEmpty(!cameraResults)) {
        (?return) = op:newObject("edu.tufts.hrilab.fol.Symbol", "error");
    } else {
        (!result) = op:get(!cameraResults, 0);
        //create reference and add any additional props
        (!additionalProps) = act:getEmptyProps();
        (!ref) = act:createCogRefWithProps(!job, !additionalProps);
        //bind reference to the result that it matches
        act:bindCognexResult(!ref, !result, 0);
        (?return) = op:invokeMethod(!ref, "getRefId");
    }
}

(edu.tufts.hrilab.fol.Symbol ?objectRef:ref) = findObjectOfType(edu.tufts.hrilab.fol.Symbol ?descriptor) {
    edu.tufts.hrilab.fol.Predicate !queryPred;
    java.util.List !bindings;
    java.util.HashMap !binding;
    edu.tufts.hrilab.fol.Variable !bindingVar;

    java.lang.Double !heightMeters;
    edu.tufts.hrilab.fol.Symbol !zHeight;
    edu.tufts.hrilab.fol.Symbol !area;
    edu.tufts.hrilab.fol.Symbol !currPose;

    edu.tufts.hrilab.fol.Symbol !distSymbol;
    java.lang.Double !dist;
    edu.tufts.hrilab.fol.Symbol !xDistSymbol;
    java.lang.Double !xDist;

    java.lang.Double !downDist;
    java.lang.Double !backDist;

    effects : {
            success : at(?objectRef, !currPose);
    }

    op:log(info, "[findObjectOfType] calling getRefForJob");
    ?objectRef = act:getRefForJob(?descriptor);
    op:log(info, "[findObjectOfType] finished getRefForJob with result + ?objectRef");

    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    (!queryPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,X)");
    !bindings = act:queryBelief(!queryPred);
    op:log(warn,"[findObjectOfType] bindings !bindings queryPred !queryPred");
    if (~op:isEmpty(!bindings)) {
        !binding = op:get(!bindings, 0);
        (!currPose) =op:get(!binding,!bindingVar);
    } else {
        op:log(error,"[findObjectOfType] actor not at a valid location. cannot bind ?objectRef to that location in effects");
        exit(FAIL,"not(at(?actor,valid(location)))" );
    }
}

() = controlLoop(edu.tufts.hrilab.fol.Symbol ?actor) {
    edu.tufts.hrilab.fol.Predicate !actionPredicate;
    java.lang.Long !gid;
    !actionPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "sort", ?actor);
    while(true) {
        !gid = act:submitGoal(!actionPredicate);
        act:joinOnGoal(!gid);
    }
}


() = getOn(edu.tufts.hrilab.fol.Symbol ?object, edu.tufts.hrilab.fol.Symbol ?destination) {
   edu.tufts.hrilab.fol.Predicate !goalPred;

   (!goalPred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object,?destination)");
   goal:!goalPred;
}

() = grab["?actor grabs ?physobj"](edu.tufts.hrilab.fol.Symbol ?physobj:physobj, edu.tufts.hrilab.fol.Symbol ?pose:pose) {

    conditions : {
        pre : free(?actor);
        pre : at(?physobj,?pose);
        pre : at(?actor, ?pose);
    }
    effects : {
        success : holding(?actor, ?physobj);
        success : not(free(?actor));
        success : not(at(?physobj,?pose));
        nonperf : not(at(?actor,?pose));
        nonperf : unknownlocation(?actor);
    }
    op:log(info, "[grab]");
    //act:gotoCamerapose(?pose);
    act:senseAndGoToDetectedObject();
    act:pickUp();
}

() = putdown["?actor releases ?physobj"](edu.tufts.hrilab.fol.Symbol ?physobj:physobj, edu.tufts.hrilab.fol.Symbol ?pose:pose) {
    conditions : {
        pre : holding(?actor, ?physobj);
        pre : at(?actor,?pose);
    }
    effects : {
        success : not(holding(?actor, ?physobj));
        success : free(?actor);
        success : at(?physobj,?pose);
    }

    op:log(info, "[putdown] ?physobj + ?pose");
    act:putDown();
}


() = gotopose["goes to pose at camera height"](edu.tufts.hrilab.fol.Symbol ?pose:pose) {
    edu.tufts.hrilab.fol.Predicate !queryPred;
    java.util.List !bindings;
    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Symbol !currPose = "current";
    edu.tufts.hrilab.fol.Variable !bindingVar;
    !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,X)");
    !bindings = act:queryBelief(!queryPred);
    if (~op:isEmpty(!bindings)) {
        !elem = op:get(!bindings, 0);
        (!currPose) =op:get(!elem,!bindingVar);
    }

    act:gotopose(?pose, !currPose);
}

() = gotopose["moves to ?pose1, from ?pose2"](edu.tufts.hrilab.fol.Symbol ?pose1:pose, edu.tufts.hrilab.fol.Symbol ?pose2:pose) {
    conditions : {
        pre : at(?actor, ?pose2);
    }
    effects : {
        success : not(at(?actor, ?pose2));
        success : at(?actor,?pose1);
        nonperf : not(at(?actor,?pose1));
        nonperf : not(at(?actor,?pose2));
        nonperf : unknownlocation(?actor);
    }
    act:goToLocation(?pose1);
    op:log(info, "Finished going to pose ?pose1");
}
