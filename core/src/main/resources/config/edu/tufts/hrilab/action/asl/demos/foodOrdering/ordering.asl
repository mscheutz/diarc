import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;
import java.util.Map;
import java.util.List;
import java.lang.String;
import java.lang.Integer;


//Locations, Areas, and Poses
(Symbol ?locationRef) = saveLocation(Symbol ?descriptor) {
    ?locationRef = tsc:saveLocation(?descriptor);
}

(Symbol ?areaRef) = addArea(Symbol ?descriptor, Symbol ?locationRef) {
    Predicate !tmp;

    op:log(debug, "[addArea] saving area ?descriptor reachable from ?locationRef");
    ?areaRef = tsc:addArea(?descriptor);
}

(Symbol ?poseRef) = saveCameraPose["Save a pose which puts the camera above a given area"](Symbol ?descriptor, String ?poseString, Symbol ?areaRef) {
    op:log(debug, "[saveCameraPose] saving camera pose ?descriptor for area ?areaRef");

    Predicate !tmp;
    ?poseRef=act:addCameraPose(?descriptor, ?poseString);

    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "poseAbove(?actor,?poseRef,?areaRef)");
    act:assertBelief(!tmp);
}

(Symbol ?poseRef) = saveAreaPose["Save a pose which puts the gripper vertically at a reasonable dropoff point in a given area"](Symbol ?descriptor, String ?poseString, Symbol ?areaRef) {
    op:log(debug, "[saveAreaPose] saving area pose ?descriptor for area ?areaRef");

    Predicate !tmp;
    ?poseRef=act:addCameraPose(?descriptor, ?poseString);

    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "areaPose(?actor, ?poseRef,?areaRef)");
    act:assertBelief(!tmp);
}


//Ingredient definition (interactive and grasp point)
() = defineIngredient["defines a new ingredient and asks where it is located"](edu.tufts.hrilab.fol.Symbol ?descriptor){

    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Variable !x = "X";
    edu.tufts.hrilab.fol.Symbol !area;
    edu.tufts.hrilab.fol.Symbol !parentType;
    edu.tufts.hrilab.fol.Symbol !job;

    edu.tufts.hrilab.fol.Symbol !right="rightArm:yumi";
    edu.tufts.hrilab.fol.Symbol !detectionArea;


    !bindings = act:askQuestionFromString(?actor,"Where is it located?", pattern(area(X)));
    !area= op:get(!bindings, !x);

    !bindings = act:askQuestionFromString(?actor,"Okay, what vision job is used to detect it?", job(X));
    !job = op:get(!bindings, !x);

    act:defineIngredientHelper(?descriptor,!area,!job);

    !bindings = act:askQuestionFromString(?actor,"At which area should I look for a ?descriptor?", pattern(area(X)));
    !detectionArea = op:get(!bindings, !x);

    !right.act:goTo(!detectionArea);
    !right.act:teachGraspPoint(?descriptor);

    act:generateResponseFromString("okay");

    op:sleep(2000);
    !right.act:goTo(!detectionArea);

    act:generateResponseFromString("Okay, I know what ?descriptor is");
}

() = defineIngredientHelper["helper method to standardize interaction based item definition with asl based definition"](Symbol ?descriptor, Symbol ?area, Symbol ?job){

    Predicate !tmp;

    ?descriptor=act:addDetectionType(?descriptor,?job);

    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "observableAt(?descriptor,?area)");
    act:assertBelief(!tmp);
}


() = teachGraspPoint(Symbol ?type) {
    Symbol !refId;
    !refId = act:positItem(?type);
    op:log(info, "posited item !refId");
    op:log(info, "perceiving entity for refId !refId of type ?type");
    act:teachGraspPointHelper(!refId, ?type);
    act:invalidateReference(!refId);
}

(Symbol ?refId) = positItem(Symbol ?itemType) {
    Term !props;
    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(?itemType(X:physobj))");
    op:log(info, "positing reference for type ?itemType with props !props");
    ?refId = act:positReference(!props);
}

() = teachGraspPointHelper(Symbol ?refId, Symbol ?itemType) {
    Map !bindings;
    Variable !x = "X";
    Symbol !response;
    Symbol !equivalentPose;

    act:perceiveEntityFromSymbol(?refId);

    !bindings = act:askQuestionFromString(?actor,"I see a ?itemType, can you show me how to grasp it?", hold(X));
    !response = op:get(!bindings, !x);
    if (~op:equalsValue(!response, none())) {
        act:defineGraspPointForDescriptor(?refId, ?itemType);
    } else {
        //todo: this is a bad failure justification
        exit(FAIL, not(found(graspPoint)));
    }

}

//Item definition
() = defineItem[""](edu.tufts.hrilab.fol.Symbol ?itemName){

    java.lang.Integer !i =0;
    java.lang.Integer !optionIndex =0;

    java.util.Map !bindings;
    java.util.List !bindingArgs;
    edu.tufts.hrilab.fol.Variable !x = "X";

    edu.tufts.hrilab.fol.Term !optionsCountQuery= "val(X)";
    edu.tufts.hrilab.fol.Term !options;
    java.lang.Integer !optionsCount= 0;
    edu.tufts.hrilab.fol.Term !optionsQuery= "option(X)";
    edu.tufts.hrilab.fol.Symbol !optionSymbol;
    java.util.List !optionsArgs;
    edu.tufts.hrilab.fol.Term !endQuery= "want(A,endItemDefinition(Y:agent,X:property))";
    edu.tufts.hrilab.fol.Term !props;
    edu.tufts.hrilab.fol.Symbol !trayRefId;
    edu.tufts.hrilab.fol.Symbol !itemRefId;
    edu.tufts.hrilab.fol.Predicate !tmpPredicate;
    edu.tufts.hrilab.fol.Predicate !location= "none()";
    edu.tufts.hrilab.fol.Predicate !responsePredicate;

    edu.tufts.hrilab.fol.Predicate !scriptID;
    edu.tufts.hrilab.fol.Symbol !startSymbol ="start";
    edu.tufts.hrilab.fol.Symbol !endSymbol= "end";

    java.util.List !signatureArgs;
    !signatureArgs = op:newArrayList("edu.tufts.hrilab.fol.Symbol");

    op:add(!signatureArgs,?actor);

    ?itemName=act:addItem(?itemName);
    java.lang.String !signatureName= "prepare?itemName";

    !props = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "and(?itemName(X:item))");
    !itemRefId = act:positReference(!props);

    !scriptID =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", !signatureName,!signatureArgs);

    act:updateActionLearning(!scriptID,!startSymbol);
    !bindings = act:askQuestionFromString(?actor,"Okay. How do I prepare a ?itemName ?",!endQuery);

    //generate new signature that is unique to model name
    op:log(warn, "[defineItem] ending learning for: !scriptID");
    act:updateActionLearning(!scriptID,!endSymbol);

    !responsePredicate=op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "isComplete",?itemName);
    !tmpPredicate=op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "generateResponse",?actor,!responsePredicate);
    !tmpPredicate = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "insert",!tmpPredicate);
    act:modifyAction(!scriptID, !tmpPredicate,!location );

    act:invalidateReference(!itemRefId);

    act:generateResponseFromString("okay");
}

//interactive version where it asks questions about the differences
() = defineItemByAnalogy["creates a new prepare script based on an existing one"](edu.tufts.hrilab.fol.Symbol ?newItem){

  Symbol !oldItem;
  edu.tufts.hrilab.fol.Predicate !oldScriptGoal;
  edu.tufts.hrilab.fol.Predicate !newScriptGoal;
  edu.tufts.hrilab.fol.Predicate !likeGoal;
  edu.tufts.hrilab.fol.Predicate !modLocation;
  edu.tufts.hrilab.fol.Predicate !objectPred;
  edu.tufts.hrilab.fol.Predicate !location;

  java.util.Map !bindings;
  java.util.List !bindingArgs;
  edu.tufts.hrilab.fol.Variable !x = "X";
  edu.tufts.hrilab.fol.Variable !y = "Y";
  edu.tufts.hrilab.fol.Predicate !modification;

  java.lang.String !modName;

  edu.tufts.hrilab.fol.Predicate !oldResponse;
  edu.tufts.hrilab.fol.Predicate !newResponse;

  edu.tufts.hrilab.fol.Symbol !speaker;

   //TODO:brad: do we still need to do this?
  (!speaker) = op:newObject("edu.tufts.hrilab.fol.Symbol",?actor);

  ?newItem=act:addItem(?newItem);

  !bindings = act:askQuestionFromString(?actor,"What item is it like?", item(X));
  !oldItem = op:get(!bindings, !x);
  !oldScriptGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate", "prepare!oldItem(?actor)");
  op:log(debug, "osg !oldScriptGoal");

  !newScriptGoal = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate", "prepare?newItem(?actor)");
  op:log(debug, "nsg !newScriptGoal");

  !likeGoal =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate", "like", !newScriptGoal,!oldScriptGoal);
  op:log(debug, "lg !likeGoal");

  !bindings = act:askQuestionFromString(!speaker,"okay. what are the differences", mod(X,Y));
  !modification = op:get(!bindings, !x);
  !location = op:get(!bindings, !y);
  op:log(debug, "modification !modification location !location");
  act:modifyAction(!likeGoal,!modification,!location);

  !bindings = act:askQuestionFromString(!speaker,"okay. are there any more differences?", mod(X,Y));
  !modification = op:get(!bindings, !x);
    !location = op:get(!bindings, !y);
  !modName = op:getName(!modification);
  while(op:!=(!modName,"none")){
    op:log(debug, "modification !modification location !location");
    act:modifyAction(!newScriptGoal,!modification,!location);
    !bindings = act:askQuestionFromString(!speaker,"okay. are there any more differences?", mod(X,Y));
    !modification = op:get(!bindings, !x);
    !location = op:get(!bindings, !y);
    !modName = op:getName(!modification);
  }

  !newResponse=op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "isComplete",?newItem);
  !newResponse=op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "generateResponse",?actor,!newResponse);

  !oldResponse=op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "isComplete",!oldItem);
  !oldResponse=op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "generateResponse",?actor,!oldResponse);

  //replace(newStep,oldStep)
  !modification =op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","replace", !newResponse,!oldResponse);
  !location= op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory","createPredicate", "none()");
  act:modifyAction(!newScriptGoal,!modification,!location);

  act:generateResponseFromString("okay");
}

() = describeCurrentGoal["describes ?actor's current active task(s)"](){

    java.util.List !goalPreds;
    java.lang.Integer !resultsSize=0;
    java.lang.String !status;
    Predicate !goalPred;

    !goalPreds = act:getSystemGoalPredicates();
    !resultsSize = op:size(!goalPreds);
    if(op:gt(!resultsSize,0)){
        foreach(!goalPred: !goalPreds){
            !status= op:getName(!goalPred);
            !goalPred= op:getArg(!goalPred,0);
            act:generateResponseFromString("!status task: ");
            act:generateResponse(!goalPred);
        }
    } else {
        act:generateResponseFromString("no current task");
    }
}

() = describePendingGoals["describes ?actor's pending goals task"](){
    java.util.List !goalPreds;
    java.lang.Integer !resultsSize=0;

    !goalPreds = act:getPendingGoalsPredicates;
    !resultsSize = op:size(!goalPreds);

    if(op:gt(!resultsSize,0)){
       foreach(!goalPred: !goalPreds){
           act:generateResponse(!goalPred);
       }
    } else{
       act:generateResponseFromString("no pending tasks");
    }

}

() = resetDomain(){
    Predicate !queryPred;// = at(?actor,X);

    !queryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate","itemAt(X,Y)");

     Variable !key = X;
    java.util.List !bindings;
    java.util.Map !binding;
    Symbol !refId;


    !bindings = act:queryBelief(!queryPred);
    if (op:isEmpty(!bindings)) {
        op:log("warn", "[resetDomain] itemAt returned no results");
    } else {
         foreach(!binding: !bindings){
             !refId =  op:get(!binding, !key);
             act:invalidateReference(!refId);
         }
    }
}