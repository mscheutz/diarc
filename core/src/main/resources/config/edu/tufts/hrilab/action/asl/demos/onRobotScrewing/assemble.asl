() = assemble["assembles model for ?modelID from belief"](edu.tufts.hrilab.fol.Symbol ?modelID){
    edu.tufts.hrilab.fol.Predicate !goalState;
    //edu.tufts.hrilab.action.goal.PriorityTier !priorityTier = SKIPPENDING;
    //edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;

    edu.tufts.hrilab.fol.Symbol !descriptor;

    op:log(debug,"[assemble] model ID: ?modelID");

    (!descriptor) = tsc:getDescriptorForID(?modelID);

    op:log(debug,"[assemble] descriptor: !descriptor");

    !goalState = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "assemble!descriptor(?actor,?modelID)");
    op:log(debug,"[assemble ]goal state: !goalState");
    //TODO:brad: it's not ideal that this calls submit goal, it will break translation
    act:submitGoal(!goalState);
    //act:submitGoal(!goalState, !execType, !priorityTier);
}

() = startLearningAssembleScript["assembles model for ?modelID from belief"](edu.tufts.hrilab.fol.Symbol ?modelName){

    effects:{
        success: object(?modelName,physobj);
    }

    //assemble(modelA) -> assembleModelA
    edu.tufts.hrilab.fol.Predicate !scriptID;
    edu.tufts.hrilab.fol.Symbol !startSymbol ="start";
    edu.tufts.hrilab.fol.Predicate !objectPred;
    edu.tufts.hrilab.fol.Symbol !descriptor;

    (!descriptor) = tsc:getDescriptorForID(?modelName);

    (!scriptID) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "assemble!descriptor(?actor,?modelName)");

    //assert object definition for modelName to belief for planner can use it
    //need to do this here because modifyActionLearning blocks
    (!objectPred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "object(?modelName,physobj)");
    op:log(debug,"objectPred !objectPred");
    act:assertBelief(!objectPred);

    async {
        act:learnAction(!scriptID);
    }
    act:waitForActionLearningStart(!scriptID);
}

() = endLearningAssembleScript["ends learning of assemble?modelName()"](edu.tufts.hrilab.fol.Symbol ?modelName){
    edu.tufts.hrilab.fol.Predicate !scriptID;
    edu.tufts.hrilab.fol.Symbol !endSymbol= "end";
    edu.tufts.hrilab.fol.Predicate !modPred;
    edu.tufts.hrilab.fol.Predicate !modLocation;

    edu.tufts.hrilab.fol.Symbol !descriptor;

    (!descriptor) = tsc:getDescriptorForID(?modelName);
    op:log(debug, "[endLearningAssembleScript] descriptor: !descriptor");

    //generate new signature that is unique to model name
    (!scriptID) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "assemble!descriptor(?actor,?modelName)");
    op:log(debug, "[endLearningAssembleScript] scriptID: !scriptID");
    act:endActionLearning(!scriptID);
}

() = getOn["gets ?object on to the surface beneath ?destination"](edu.tufts.hrilab.fol.Symbol ?object, edu.tufts.hrilab.fol.Symbol ?destination) {
   edu.tufts.hrilab.fol.Predicate !goalPred;

   (!goalPred) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?object,?destination)");
   goal:!goalPred;
}

//TODO: move get a version of this working in mainline DIARC
//interactive version where it asks questions about the differences
() = modifyAssemble["creates a new assembly script based on an existing one"](edu.tufts.hrilab.fol.Predicate ?newScriptID, edu.tufts.hrilab.fol.Predicate ?oldScriptID){

  java.util.List !predArgs;
  edu.tufts.hrilab.fol.Symbol !newModelID;
  edu.tufts.hrilab.fol.Symbol !oldModelID;
  edu.tufts.hrilab.fol.Predicate !newScriptGoal;
  edu.tufts.hrilab.fol.Predicate !oldScriptGoal;
  edu.tufts.hrilab.fol.Predicate !likeGoal;
  edu.tufts.hrilab.fol.Predicate !uniqueEffect;
  edu.tufts.hrilab.fol.Predicate !modLocation;
  edu.tufts.hrilab.fol.Predicate !objectPred;

  java.util.Map !bindings;
  java.util.List !bindingArgs;
  edu.tufts.hrilab.fol.Variable !x = "X";
  edu.tufts.hrilab.fol.Variable !y = "Y";

  edu.tufts.hrilab.fol.Predicate !modification;
  java.lang.String !modName;
  edu.tufts.hrilab.fol.Predicate !location;

  edu.tufts.hrilab.fol.Symbol !speaker;

  edu.tufts.hrilab.fol.Symbol !descriptor;

  !speaker = op:newObject("edu.tufts.hrilab.fol.Symbol",?actor);

  !predArgs = op:invokeMethod(?newScriptID, getArgs);
  !newModelID = op:get(!predArgs, 1);

  !predArgs = op:invokeMethod(?oldScriptID, getArgs);
  !oldModelID = op:get(!predArgs, 1);

//did(modifyAssemble(self:agent,assemble(self:agent,nv30-fau),assemble(self:agent,nf32-sv circuit breaker),replace(did(screwIn(self:agent,m3,physobj_7:physobj)),did(screwIn(self:agent,m3,physobj_5:physobj))),none))
  //get action based on model id
  op:log(debug, "in modify assemble");

  !descriptor = tsc:getDescriptorForID(!newModelID);
  !newScriptGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "assemble!descriptor(?actor,!newModelID)");
  op:log(debug, "nsg !newScriptGoal");
  !descriptor = tsc:getDescriptorForID(!oldModelID);
  !oldScriptGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "assemble!descriptor(?actor,!oldModelID)");
  op:log(debug, "osg !oldScriptGoal");
  !likeGoal =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate", "like", !newScriptGoal,!oldScriptGoal);
  op:log(debug, "lg !likeGoal");
  //submit modify goal

  !bindings = act:askQuestionFromString(!speaker,"what are the differences", mod(X,Y));
  !modification = op:get(!bindings, !x);
  !modName = op:getName(!modification);
  !location = op:get(!bindings, !y);
  op:log(debug, "modification !modification location !location");
  act:modifyAction(!likeGoal,!modification,!location);

//
  !bindings = act:askQuestionFromString(!speaker,"okay are there any more differences", mod(X,Y));
  !modification = op:get(!bindings, !x);
  !modName = op:getName(!modification);
  !location = op:get(!bindings, !y);
  while(op:!=(!modName,"none")){
    op:log(debug, "modification !modification");
    act:modifyAction(!newScriptGoal,!modification,!location);
    !bindings = act:askQuestionFromString(!speaker,"okay are there any more differences", mod(X,Y));
    !modification = op:get(!bindings, !x);
    !location = op:get(!bindings, !y);
    !modName = op:getName(!modification);
 }

  act:generateResponseFromString("okay");
}

//TODO:brad: refactor "assemble?modelID(?modelID)" predicate creation?

//copy case, here an new action is generated based on an existing action and a given modification
() = modifyAssemble["creates a new assembly script based on an existing one"](edu.tufts.hrilab.fol.Predicate ?newScriptID, edu.tufts.hrilab.fol.Predicate ?oldScriptID, edu.tufts.hrilab.fol.Predicate ?modification, edu.tufts.hrilab.fol.Predicate ?location){

//  effects:{
//   success: object(!newModelID,physobj);
//  }

  java.util.List !predArgs;
  edu.tufts.hrilab.fol.Symbol !newModelID;
  edu.tufts.hrilab.fol.Symbol !oldModelID;
  edu.tufts.hrilab.fol.Predicate !newScriptGoal;
  edu.tufts.hrilab.fol.Predicate !oldScriptGoal;
  edu.tufts.hrilab.fol.Predicate !likeGoal;
  edu.tufts.hrilab.fol.Predicate !uniqueEffect;
  edu.tufts.hrilab.fol.Predicate !modLocation;
  edu.tufts.hrilab.fol.Predicate !objectPred;

  edu.tufts.hrilab.fol.Symbol !descriptor;

 (!predArgs) = op:invokeMethod(?newScriptID, getArgs);
 (!newModelID) = op:get(!predArgs, 1);

  (!predArgs) = op:invokeMethod(?oldScriptID, getArgs);
  (!oldModelID) = op:get(!predArgs, 1);

//did(modifyAssemble(self:agent,assemble(self:agent,nv30-fau),assemble(self:agent,nf32-sv circuit breaker),replace(did(screwIn(self:agent,m3,physobj_7:physobj)),did(screwIn(self:agent,m3,physobj_5:physobj))),none))
  //get action based on model id
  op:log(debug, "in modify assemble");
  (!descriptor) = tsc:getDescriptorForID(!newModelID);
  (!newScriptGoal) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "assemble!descriptor(?actor,!newModelID)");
  op:log(debug, "nsg !newScriptGoal");
  (!descriptor) = tsc:getDescriptorForID(!oldModelID);
  (!oldScriptGoal) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "assemble!descriptor(?actor,!oldModelID)");
  op:log(debug, "osg !oldScriptGoal");
  (!likeGoal) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate", "like", !newScriptGoal,!oldScriptGoal);
  op:log(debug, "lg !likeGoal");
  //submit modify goal
  act:modifyAction(!likeGoal,?modification,?location);

  //add effect so it can be accessed later
//  (!uniqueEffect) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "insert(assembled(?actor,?newScriptID))");
//  (!modLocation) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "effect(success)");
//  act:modifyAction(!newScriptGoal,!uniqueEffect,!modLocation);
  (!objectPred) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "object(!newModelID,physobj)");
  act:assertBelief(!objectPred);
}

//within action case, here the action is modified in place, given a modification
() = modifyAssemble["creates a new assembly script based on an existing one"](edu.tufts.hrilab.fol.Predicate ?scriptID,  edu.tufts.hrilab.fol.Predicate ?modification, edu.tufts.hrilab.fol.Predicate ?location){

  java.util.List !predArgs;
  edu.tufts.hrilab.fol.Symbol !modelID;
  edu.tufts.hrilab.fol.Predicate !scriptGoal;


 (!predArgs) = op:invokeMethod(?scriptID, getArgs);
 (!modelID) = op:get(!predArgs, 0);

  //get action based on model id
  op:log(debug, "in modify assemble in place");
  (!descriptor) = tsc:getDescriptorForID(!modelID);
  (!scriptGoal) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "assemble!descriptor(?actor,!modelID)");
  op:log(debug, "nsg !scriptGoal");

  //submit modify goal
  act:modifyAction(!scriptGoal,?modification,?location);
}

//TODO:brad update this to new assemble goal submission format?
() = assembleVision["classifies model in front of itself and calls relevant assemble script"](){

     edu.tufts.hrilab.fol.Symbol !modelType;
     edu.tufts.hrilab.fol.Predicate !scriptID;
     edu.tufts.hrilab.fol.Symbol !unk;
     (!unk) =op:newObject(edu.tufts.hrilab.fol.Symbol,"unknown");
     edu.tufts.hrilab.fol.Symbol !fail;
     (!fail) =op:newObject(edu.tufts.hrilab.fol.Symbol,"failure");
     edu.tufts.hrilab.fol.Predicate !failCond;

    edu.tufts.hrilab.fol.Symbol !descriptor;

     //TODO:brad: move this out?
     act:goToCameraPose("conveyor");
     act:centerObject();
     (!modelType)=act:classifyModel();
     if(op:equals(!modelType,!unk)){
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "find(?actor,knownModelType)");
        exit(FAIL, !failCond);
     }

     if(op:equals(!modelType,!fail)){
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "error(classifyingObject)");
     }

//     goal:assembled(?actor, !modelType);

     (!descriptor) = tsc:getDescriptorForID(?modelType);

     (!scriptID) =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "assemble!descriptor(?actor,!modelType)");
     goal:!scriptID;

}
