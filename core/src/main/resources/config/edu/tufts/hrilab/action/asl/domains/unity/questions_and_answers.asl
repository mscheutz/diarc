import java.lang.String;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;

//["Only exists to make my function call faster."]
() = askQuestionCrouch(){
  act:askQuestion(brad, crouch);
}

//["Makes the robot ask about repairing a specific tube. For testing purposes"]
() = aapromptRepairTubeQuery(){
  java.util.List !damagedList;

  !damagedList = op:newArrayList("java.lang.String");
  op:add(!damagedList, "leftthree");

  !damagedList = act:initiateRepairTubeQuery("alpha");
}

//["The robot asks a question of the human and executes it if yes"]
() = askQuestion(edu.tufts.hrilab.fol.Symbol ?listener, edu.tufts.hrilab.fol.Predicate ?innerAction){
  act:askQuestion(?listener, ?innerAction, 20000);
}

//["The robot asks a question of the human and executes it if yes"]
() = askQuestion(edu.tufts.hrilab.fol.Symbol ?listener, edu.tufts.hrilab.fol.Predicate ?innerAction, java.lang.Integer ?waitTime){
  java.util.List !bindings;
  Predicate !question;
  java.lang.String !generatedResponse;
  java.lang.Long !goalID;
  edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
  edu.tufts.hrilab.fol.Predicate !listenPred;
  edu.tufts.hrilab.fol.Predicate !innerDid;
  edu.tufts.hrilab.fol.Predicate !answerPred;
  java.lang.Boolean !val;
  // Below are just to parse the answer...
  java.util.List !answerBeliefs;
  java.util.HashMap !elem;
  edu.tufts.hrilab.fol.Variable !keyY = Y;
  Symbol !symY;
  String !answer;
  java.lang.Long !priority;
  //TODO: Should check the robot is not listening already, partiularily from the same human

  !bindings = op:newArrayList("java.lang.String");
  op:log("info", "1");
  // Changing this to use an itk, which is just a sentence.
  !innerDid = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "did(self, ?innerAction)");
  !question = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "questionYN(want(brad, self, infinitive(!innerDid)))");
  // The above statement shows up in dialoge manager like: UNKNOWN(dem,Brad,itk(dem,want(brad,self,infinitive(did(self,dance)))),{})
  // So they syntax should look more like this.

  !generatedResponse = act:submitNLGRequest(?actor, ?listener, !question, !bindings);
  act:sayText(!generatedResponse);

  op:log("info", ?listener);
  !listenPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "did(?actor, listen(!question, ?listener, ?waitTime))");
  op:log("info", "About to submit the YN goal.");
  act:submitGoal(!listenPred, !goalID);
  !goalStatus = act:joinOnGoal(!goalID, ?waitTime);
  op:log("info", "This is the goal status of listening");
  op:log("info", !goalStatus);
  !val = op:invokeMethod(!goalStatus, "isTerminated");
  op:log("info", !val);
  !val = op:invokeMethod(!goalStatus, "isFailure");
  op:log("info", !val);

  if(~op:invokeMethod(!goalStatus, "isTerminated")){
    // This goes off when the goal join timeout is reached instead
    op:log("info", "this is inside the not terminated if statement");
    !generatedResponse = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "listening(?actor, X)");
    act:retractBelief(!generatedResponse);
  } elseif (~op:invokeMethod(!goalStatus, isFailure)){
    op:log("info", "This is inside the not failure else statement.");
    // This goes off when there was a successful response given (yes, no) and saved in beliefs
    !answerPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "answerYN(!question, Y)");
    !innerDid = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "did(?actor, ?innerAction)");
    //TODO: Try to go though NLG as an ack
    act:sayText("Okay");
    !answerBeliefs = act:queryBelief(!answerPred);
    op:log("info", !answerBeliefs);
    foreach (!elem : !answerBeliefs) {
      !symY = op:invokeMethod(!elem, "get", !keyY);
      !answer = op:invokeMethod(!symY, "toString");
      op:log("info", !answer);
      if (op:invokeMethod(!answer, equals, "yes")){
        op:log("info", "Let's execute the action");
        !goalID = act:submitGoal(!innerDid);
        op:log("info", "Printing from gwen_trial, looking at goalID info.");
        op:log("info", !goalID);
        !priority = act:getGoalPriority(!goalID);
        op:log("info", "Priority");
        op:log("info", !priority);
        op:log("info", "Setting priority to 3");
        act:setGoalPriority(!goalID, 3, !priority);
        op:log("info", !priority);
        act:sayText("I will ?innerAction");
      } else {
        act:sayText("I will not ?innerAction");
      }
    }
    act:retractBelief(!answerPred);
  }
  op:log("info", "End of ask question");
}

//["This will ask a question but pass the result back instead of executing it"]
(java.lang.Boolean ?execute) = askQuestionForAnswer(edu.tufts.hrilab.fol.Symbol ?listener, edu.tufts.hrilab.fol.Predicate ?innerAction){
  ?execute = act:askQuestionForAnswer(?listener, ?innerAction, 20000);
}

//["This will ask a question but pass the result back instead of executing it"]
(java.lang.Boolean ?execute) = askQuestionForAnswer(edu.tufts.hrilab.fol.Symbol ?listener, edu.tufts.hrilab.fol.Predicate ?innerAction, java.lang.Integer ?waitTime){
    java.util.List !bindings;
    Predicate !question;
    java.lang.String !generatedResponse;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.fol.Predicate !listenPred;
    edu.tufts.hrilab.fol.Predicate !innerDid;
    edu.tufts.hrilab.fol.Predicate !answerPred;
    java.lang.Boolean !val;
    // Below are just to parse the answer...
    java.util.List !answerBeliefs;
    java.util.HashMap !elem;
    edu.tufts.hrilab.fol.Variable !keyY = Y;
    Symbol !symY;
    String !answer;
    java.lang.Long !priority;
    //TODO: Should check the robot is not listening already, partiularily from the same human

    ?execute = op:newObject("java.lang.Boolean", False);
    !bindings = op:newArrayList("java.lang.String");
    // Changing this to use an itk, which is just a sentence.
    !innerDid = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "did(self, ?innerAction)");
    !question = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "questionYN(want(brad, self, infinitive(!innerDid)))");
    // The above statement shows up in dialoge manager like: UNKNOWN(dem,Brad,itk(dem,want(brad,self,infinitive(did(self,dance)))),{})
    // So they syntax should look more like this.
    !generatedResponse = act:submitNLGRequest(?actor, ?listener, !question, !bindings);
    act:sayText(!generatedResponse);

    op:log("info", ?listener);
    !listenPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "did(?actor, listen(!question, ?listener, ?waitTime))");
    op:log("info", "About to submit the YN goal.");
    !goalID = act:submitGoal(!listenPred);
    !goalStatus = act:joinOnGoal(!goalID, ?waitTime);
    op:log("info", "This is the goal status of listening");
    op:log("info", !goalStatus);
    !val = op:invokeMethod(!goalStatus, "isTerminated");
    op:log("info", !val);
    !val = op:invokeMethod(!goalStatus, "isFailure");
    op:log("info", !val);

    if (~op:invokeMethod(!goalStatus, isTerminated)) {
      // This goes off when the goal join timeout is reached or a new goal is given
      op:log(debug, "this is inside the not terminated if statement");
      !generatedResponse = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "listening(?actor, X)");
      act:retractBelief(!generatedResponse);
      ?execute = op:newObject("java.lang.Boolean", False);
    } elseif (~op:invokeMethod(!goalStatus, isFailure, !val)) {
      op:log(debug, "This is inside the not failure else statement.");
      // This goes off when there was a successful response given (yes, no) and saved in beliefs
      !answerPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "answerYN(!question, Y)");
      !innerDid = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "did(?actor, ?innerAction)");
      //TODO: Try to go though NLG as an ack
      act:sayText("Okay");
      !answerBeliefs = act:queryBelief(!answerPred);
      op:log("info", !answerBeliefs);
      foreach (!elem : !answerBeliefs) {
        !symY = op:invokeMethod(!elem, "get", !keyY);
        !answer = op:invokeMethod(!symY, "toString");
        op:log("info", !answer);
        if (op:invokeMethod(!answer, "equals", "yes")) {
          act:sayText("Okay, I will start ?innerAction");
          ?execute = op:newObject("java.lang.Boolean", True);
        } else {
          act:sayText("Okay, I will not do ?innerAction");
          ?execute = op:newObject("java.lang.Boolean", False);
        }
      }
      act:retractBelief(!answerPred);
    }
    op:log("info", "End of ask question");
}

//["This will ask a question but pass the result back instead of executing it"]
(java.lang.Boolean ?execute) = askStringForAnswer(edu.tufts.hrilab.fol.Symbol ?listener, java.lang.String ?question, java.lang.Integer ?waitTime){
  java.util.List !bindings;
  java.lang.String !generatedResponse;
  java.lang.Long !goalID;
  edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
  edu.tufts.hrilab.fol.Predicate !listenPred;
  edu.tufts.hrilab.fol.Predicate !innerDid;
  edu.tufts.hrilab.fol.Predicate !waitPred;
  edu.tufts.hrilab.fol.Predicate !answerPred;
  java.lang.Boolean !val;
  // Below are just to parse the answer...
  java.util.List !answerBeliefs;
  java.util.HashMap !elem;
  edu.tufts.hrilab.fol.Variable !keyY = Y;
  Symbol !symY;
  String !answer;

  !bindings = op:newArrayList("java.lang.String");
  !waitPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "wait(stringAnswer)");
  act:sayText(?question);

  op:log("info", ?listener);
  !listenPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "did(?actor, listen(!waitPred, ?listener, ?waitTime))");
  op:log("info", "About to submit the YN goal.");
  !goalID = act:submitGoal(!listenPred);
  !goalStatus = act:joinOnGoal(!goalID, ?waitTime);
  op:log("info", "This is the goal status of listening");
  op:log("info", !goalStatus);
  !val = op:invokeMethod(!goalStatus, "isTerminated");
  op:log("info", !val);
  !val = op:invokeMethod(!goalStatus, "isFailure");
  op:log("info", !val);

   //This is all to parse listening output
  if (~op:invokeMethod(!goalStatus, isTerminated)) {
    // This goes off when the goal join timeout is reached or a new goal is given
    op:log(debug, "this is inside the not terminated if statement");
    !generatedResponse = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", " createPredicate", "listening(?actor, X)");
    act:retractBelief(!generatedResponse);
    ?execute = op:newObject("java.lang.Boolean", False);
   } elseif (~op:invokeMethod(!goalStatus, "isFailure")) {
    // This goes off when there was a successful response given (yes, no) and saved in beliefs
    !answerPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "answerYN(!waitPred, Y)");
    //TODO: Try to go though NLG as an ack
    act:sayText("Okay");
    !answerBeliefs = act:queryBelief(!answerPred);
    op:log("info", !answerBeliefs);
    foreach(!elem : !answerBeliefs) {
      !symY = op:invokeMethod(!elem, "get", !keyY);
      !answer = op:invokeMethod(!symY, "toString");
      op:log("info", !answer);
      if(op:invokeMethod(!answer, "equals", "yes")){
        ?execute = op:newObject("java.lang.Boolean", True);
      } else{
        ?execute = op:newObject("java.lang.Boolean", False);
      }
    }
    act:retractBelief(!answerPred);
  }
  if (op:isNull(?execute)) {
    ?execute = op:newObject("java.lang.Boolean", False);
  }
  op:log("info", "End of ask question string");
}

//["This will ask a question but pass the result back instead of executing it"]
(java.lang.Boolean ?execute) = askStringForAnswer(edu.tufts.hrilab.fol.Symbol ?listener, java.lang.String ?question){
  (?execute) = act:askStringForAnswer(?listener, ?question, 10000);
}

//["A robot asking the human to reassign work to another robot"]
() = askAssignmentQuestion(edu.tufts.hrilab.fol.Symbol ?listener, edu.tufts.hrilab.fol.Predicate ?innerAction, edu.tufts.hrilab.fol.Symbol ?assigneeRobot){
  java.util.List !bindings;
  Predicate !question;
  java.lang.String !generatedResponse;
  java.lang.Long !goalID;
  edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
  edu.tufts.hrilab.fol.Predicate !listenPred;
  edu.tufts.hrilab.fol.Predicate !innerDid;
  edu.tufts.hrilab.fol.Predicate !answerPred;
  java.lang.Boolean !val;
  // Below are just to parse the answer...
  java.util.List !answerBeliefs;
  java.util.HashMap !elem;
  edu.tufts.hrilab.fol.Variable !keyY = Y;
  Symbol !symY;
  String !answer;

  !bindings = op:newArrayList("java.lang.String");
  op:log("info", "1");
  // Changing this to use an itk, which is just a sentence.
  !innerDid = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "did(?assigneeRobot, ?innerAction)");
  !question = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "questionYN(want(brad, ?assigneeRobot, infinitive(!innerDid)))");
  // The above statement shows up in dialoge manager like: UNKNOWN(dem,Brad,itk(dem,want(brad,self,infinitive(did(self,dance)))),{})
  // So they syntax should look more like this.
  !generatedResponse = act:submitNLGRequest(?actor, ?listener, !question, !bindings);
  act:sayText(!generatedResponse);

  op:log("info", ?listener);
  !listenPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "did(?actor, listen(!question, ?listener, 20000))");
  op:log("info", "About to submit the YN goal.");
  !goalID = act:submitGoal(!listenPred);
  !goalStatus = act:joinOnGoal(!goalID, 20000);
  op:log("info", "This is the goal status of listening");
  op:log("info", !goalStatus);
  !val = op:invokeMethod(!goalStatus, "isTerminated");
  op:log("info", !val);
  !val = op:invokeMethod(!goalStatus, "isFailure");
  op:log("info", !val);

  if(~op:invokeMethod(!goalStatus, isTerminated)){
    // This goes off when the goal join timeout is reached instead
    op:log("info", "this is inside the not terminated if statement");
    !generatedResponse = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "listening(?actor, X)");
    act:retractBelief(!generatedResponse);
  } elseif (~op:invokeMethod(!goalStatus, isFailure)){
    op:log("info", "This is inside the not failure else statement.");
    // This goes off when there was a successful response given (yes, no) and saved in beliefs
    !answerPred = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "answerYN(!question, Y)");
    !innerDid = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "did(?assigneeRobot, ?innerAction)");
    //TODO: Try to go though NLG as an ack
    act:sayText("Okay");
    !answerBeliefs = act:queryBelief(!answerPred);
    op:log("info", !answerBeliefs);
    foreach(!elem : !answerBeliefs) {
      !symY = op:invokeMethod(!elem, "get", !keyY);
      !answer = op:invokeMethod(!symY, "toString");
      op:log("info", !answer);
      if(op:invokeMethod(!answer, "equals", "yes")){
        op:log("info", "Let's execute the action");
        !goalID = act:submitGoal(!innerDid);
        act:sayText("I assigned ?assigneeRobot to ?innerAction");
      } else{
        act:sayText("I will not assign ?assigneeRobot to ?innerAction");
      }
    }
    act:retractBelief(!answerPred);
  }
  op:log("info", "End of reassignment ask");
}

//["This robot is waiting for a response from a human teammate."]
() = listen(Predicate ?question, edu.tufts.hrilab.fol.Symbol ?listener, java.lang.Integer ?waitTime){
  //When this function is running, getRobotGoals returns nothing.
  //This is called after a NLG request and sayText is called
  //Current recommended wait time is 20000 (10 seconds)
  java.lang.String !generatedResponse;
  java.lang.Boolean !answered;
  java.lang.Boolean !sameQuestion;
  java.lang.Integer !loopTime = 100;
  java.lang.Long !longLoopTime = 100;
  java.lang.Integer !timeElapsed = 0;
  Predicate !beliefAnswer;
  java.util.List !beliefs;
  java.util.HashMap !elem;
  edu.tufts.hrilab.fol.Variable !keyX = X;
  edu.tufts.hrilab.fol.Variable !keyY = Y;
  Symbol !symX;
  String !symXString;
  Symbol !symY;
  String !answer;
  Predicate !potentialQuestion;
  Predicate !questionYNBelief;
  java.util.List !goals;
  java.lang.Integer !goalSize;
  java.lang.Integer !initialGoalSize;

  op:log("info", "This is the question passed along to the listener: ");
  op:log("info", ?question);
  op:log("info", "This is the listener human passed along to the listener: ");
  op:log("info", ?listener);

  !generatedResponse = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "listening(?actor, ?question)");
  act:assertBelief(!generatedResponse);
  !answered = op:newObject("java.lang.Boolean", false);
  !beliefAnswer = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "answerYN(X, Y)");
  !goals = act:getCurrentGoals(?actor);
  !initialGoalSize = op:invokeMethod(!goals, "size");

  // While the question is not answered or there is still time left on the timeout
  while (~op:==(!answered, true) && op:le(!timeElapsed, ?waitTime)) {
    !goals = act:getCurrentGoals(?actor);
    !goalSize = op:invokeMethod(!goals, "size");
    // op:log("info", "This is the goal size in a listen's loop without other stuff going on: !goalSize");
    // This number represents one more than the number of goals when a question action is called during newAutonomy
    // TODO: Reimplement canceling listening if a new goal is submitted.
    act:queryBelief(!beliefAnswer, !beliefs);

    op:log("info", !timeElapsed);
    foreach(!elem : !beliefs) {
      op:log("info", "I am in the foreach loop finally");
      op:log("info", !elem);
      !symY = op:invokeMethod(!elem, "get", !keyY);
      op:log("info", "!");
      !answer = op:invokeMethod(!symY, "toString");
      op:log("info", "!");
      //The question is not being passed through, so this is just standin.
      !sameQuestion = op:newObject("java.lang.Boolean", true);
      if (op:==(!sameQuestion, true)){
        // The second time I call this in one run it errors weirdly. It does it for both yes and no in teh same call
        //     [java] [E] [     edu.tufts.hrilab.slug.common.Utterance]: [getBoundSemantics] semantics is not of type Term: no
        //       [java] [E] [     edu.tufts.hrilab.slug.common.Utterance]: [getBoundSemantics] semantics is not of type Term: no
        op:log("info", "!");
        !answered = op:newObject("java.lang.Boolean", true);
        act:retractBelief(!generatedResponse);
        act:retractBelief(!beliefAnswer);
        !beliefAnswer = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "answerYN(?question, !answer)");
        act:assertBelief(!beliefAnswer);
      }
    }
    !timeElapsed = op:invokeStaticMethod("java.lang.Integer", "sum", !loopTime, !timeElapsed);
    act:autonomySleep(!longLoopTime);
  }
  act:retractBelief(!generatedResponse);

  //This is just for if I cannot get the return value from the joinOnGoal
  op:log("info", "The loop exited.");
  if (op:ge(!timeElapsed, ?waitTime)){
    op:log("info", "The wait time was exceeded.");
    exit(FAIL, "not(answerYN(?question))");
  } else{
    !questionYNBelief = op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "createPredicate", "questionYN(?actor, ?listener, ?question, !answer)");
    act:assertBelief(!questionYNBelief);
  }
  op:log("info", "I have reached the end of listen.");
}