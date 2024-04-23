() = defineScrewType["defines new type of screw for screw, and asks for relevant parameters"](edu.tufts.hrilab.fol.Symbol ?screwType){

  java.util.Map !bindings;
  java.util.List !bindingArgs;
  edu.tufts.hrilab.fol.Symbol !jobID;
  edu.tufts.hrilab.fol.Variable !x = "X";
  edu.tufts.hrilab.fol.Symbol !screwLength;
  edu.tufts.hrilab.fol.Symbol !targetTorque;
  edu.tufts.hrilab.fol.Symbol !poseRef;
  edu.tufts.hrilab.fol.Predicate !posePred;
  //TODO:brad:how do we determine addressee?
  edu.tufts.hrilab.fol.Symbol !addressee ="james";

//TODO:brad: nee to assert screwtype if we remove this, otherwise I'm not sure how we want to handle the binding between the hole detection job and the screwtype defined here
//  (!bindings) = act:askQuestionFromString(!addressee,"which Cognex job is used to detect its hole", job(X));
//  (!jobID) = op:get(!bindings, !x);
//  tsc:addDetectionType(?screwType,!jobID);

  (!bindings) = act:askQuestionFromString(!addressee,"what is its screwing length", val(X,Y));
  (!screwLength) = op:get(!bindings, !x);

  (!bindings) = act:askQuestionFromString(!addressee,"what is its target torque in millinewton meters", val(X,Y));
  (!targetTorque) = op:get(!bindings, !x);

  tsc:configureScrewdriverProgram(?screwType, !screwLength, !targetTorque);

  (!bindings) = act:askQuestionFromString(!addressee,"at which pose can I find ?screwType screws", pose(X));
  (!poseRef) = op:get(!bindings, !x);
  (!posePred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "foundAt(?screwType,!poseRef)");
  act:assertBelief(!posePred);

  act:generateResponseFromString("okay");
}


() = mountSingleScrew["finds and mounts a single screw to the kolver screwdriver"]() {
    java.lang.Double !mountDesc = -0.174;
    java.lang.Double !mountAsc = 0.174;
    edu.tufts.hrilab.fol.Symbol !cameraResult;
    edu.tufts.hrilab.fol.Symbol !screwdriver = "screwdriver";
    edu.tufts.hrilab.fol.Symbol !feederDet = "screwHead";
    java.lang.Integer !mountID = 2;

    op:log("info","[mountSingleScrew] mounting screw");
    (!cameraResult) =act:getRefForJob(!feederDet);
    op:log(info, "[mountSingleScrew] rotating to EE: !screwdriver");
    act:rotateToEE(!screwdriver);
    //TODO:implement this new version
    tsc:moveToCognexTarget(!cameraResult);
    tsc:moveZRelative(!mountDesc);
    //seat screw
    //tsc:pickupScrew();
    //lift mounted screw
    tsc:moveZRelative(!mountAsc);
    op:log("info","[mountSingleScrew] screw mounted");
}


//TODO: this name collides with the low level mount name in mtracs
() = mountScrew["goes to the source of the given screw type and mounts one to the screwdriver"](edu.tufts.hrilab.fol.Symbol ?screwType){

  op:log(info,"[mountScrew] ?screwType");
  //get pose binding for screw type from belief
  edu.tufts.hrilab.fol.Variable !bindingVar;
  edu.tufts.hrilab.fol.Predicate !queryPred;
  java.util.List !bindings;
  java.util.HashMap !elem;
  edu.tufts.hrilab.fol.Symbol !poseRef;

  !bindingVar = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createVariable", "X");
  !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "foundAt(?screwType,X)");
  !bindings = act:queryBelief(!queryPred);
  if (~op:isEmpty(!bindings)) {
     !elem = op:get(!bindings, 0);
     (!poseRef) =op:get(!elem,!bindingVar);
  } else {
      op:log(error, "[mountScrew] missing pose for location of screw type ?screwType");
  }
  //go to that pose
  act:goToCameraPose(!poseRef);
  //Screw screw
  act:mountSingleScrew();
}

() = alignWith["aligns above a screw hole by reference id"](edu.tufts.hrilab.fol.Symbol ?holeRef){

    op:log(info,"[alignWith] ?holeRef");
   //getCognexResult from ?holeRef
   //move to cognexResult
    tsc:moveToCognexTarget(?holeRef);
}


() = runScrewdriverJob["screws in a screw once aligned"](edu.tufts.hrilab.fol.Symbol ?screwType){

    edu.tufts.hrilab.fol.Predicate !queryPred;
    java.util.List !bindings;
    java.util.HashMap !binding;
    edu.tufts.hrilab.fol.Variable !bindingVar;

    edu.tufts.hrilab.fol.Symbol !screwDescSymbol;
    java.lang.Double !screwDesc;


    edu.tufts.hrilab.fol.Symbol !default = "default";


    (!queryPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "screwDescent(?screwType,X)");
    (!bindingVar) =op:newObject("edu.tufts.hrilab.fol.Variable","X");

    (!bindings) = act:queryBelief(!queryPred);

    if (~op:isEmpty(!bindings)) {
        (!binding) = op:get(!bindings, 0);
        (!screwDescSymbol) = op:get(!binding,!bindingVar);
    }
    //Todo: Will: will this be automatically type converted?
    (!screwDesc) = op:/(!screwDescSymbol,1);
    op:log("info","[runScrewDriverJob] descending !screwDesc");
    tsc:moveZRelative(!screwDesc);


    op:log("info", "[runScrewDriverJob] screw type: ?screwType");
    //tsc:tightenScrew(?screwType); //needs to be "m3" or "deepM3"
    op:log("info","[runScrewDriverJob] ascending !screwDesc");
    (!screwDesc) = op:/(!screwDescSymbol,-1);
    tsc:moveZRelative(!screwDesc);
    op:log("info","[runScrewDriverJob] rotating ee to !default");
    act:rotateToEE(!default);
    op:log("info","[runScrewDriverJob] finished screwing");
}

() = moveToCameraHeight["moves up by a camera z height as defined by an inline constant"]() {
    //Todo: Will/Mar: change these from hardcoded values and/or see if these need to be inverses or not
    java.lang.Double !dist = 0.15;
    tsc:moveZRelative(!dist);
}

() = moveToObjectHeight["moves down by a camera z height as defined by an inline constant"]() {
    //Todo: Will/Mar: change these from hardcoded values and/or see if these need to be inverses or not
    java.lang.Double !dist = -0.15;
    tsc:moveZRelative(!dist);
}