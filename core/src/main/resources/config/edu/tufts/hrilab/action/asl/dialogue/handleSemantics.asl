import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.slug.dialogue.Question;
import java.lang.Integer;
import edu.tufts.hrilab.action.goal.PriorityTier;

() = handleSemantics["Handle semantics (direct or indirect)"](Symbol ?speaker, Symbol ?addressee, Term ?semantics, Term ?suppSemantics, Symbol ?semanticType) {
    // ?semanticType: either direct or indirect   not sure if this is the best variable name

    Term !currSemantics;
    Integer !semanticsSize;
    java.util.Set !freeVars;
    Integer !varsSize;
    Predicate !varNames;
    Predicate !failureEffects;
    Predicate !becausePredicate;
    Predicate !state;
    java.util.Map !matchingQuestions;
    Question !matchingQuestion;

    op:log("debug", "[handleSemantics] entering action script for actor: ?actor");

    // handle semantics
    !semanticsSize = op:invokeMethod(?semantics, "size");
    op:log("debug", "[handleSemantics] semantics size: !semanticsSize");

    !freeVars = op:invokeMethod(?suppSemantics, "getVars");
    op:log("debug", "[handleSemantics] vars: !freeVars");
    !varsSize = op:invokeMethod(!freeVars, "size");
    op:log("debug", "[handleSemantics] varsSize: !varsSize");

    if (op:gt(!semanticsSize, 1)) {
      // if more than one binding option for the semantics
      // TODO: contextualize clarifications into their poorly formed referring expressions
      op:log("debug", "Generating multiple bindings error");
      !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.slug.dialogue.ResponseGeneration", "generateClarificationMultipleMeanings", ?actor, ?speaker, ?semantics, ?suppSemantics);
      //act:generateResponse(?speaker, !failureEffects, direct);
      // TODO: For now asking a question without expecting a response in order to have correct utterance type for LLM NLG
      // Generally need to generate an output utterance that is meaningful to the user and get in return the proper semantic
      //  bindings from the answer bindings (or fail gracefully)
      act:askQuestion(?speaker, !failureEffects, nonexistent(X), 1, 5);
    } elseif (op:gt(!varsSize, 0)) {
      // if any unbound variables in supplemental semantics
      // TODO: contextualize clarifications into their poorly formed referring expressions
      op:log("debug", "Generating variable binding error");
      !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.slug.dialogue.ResponseGeneration", "generateFailureUnknownMeaning", ?actor, ?suppSemantics);
      act:generateResponse(?speaker, !failureEffects, direct);
    } else {
      try {
        !currSemantics = op:invokeMethod(?semantics, "get", 0);
        op:log("debug", "passing off semantics: !currSemantics");
        !matchingQuestions = act:getMatchingQuestions(?speaker,?addressee,!currSemantics);
        // TODO:brad:the way things are right now this is the only way we can do this.
        //      Ideally we could move this logic into an action selector, so we can
        //      remove the if else and have it  just be the one state spec
        if (op:invokeMethod(!matchingQuestions,"isEmpty")) {
          goal:handled(?actor, ?addressee, !currSemantics, ?semanticType);
        } else {
            op:log("debug", "have matching questions, !matchingQuestions");
            !matchingQuestion = act:selectQuestionToAnswer(!matchingQuestions, ?speaker);
            if (op:isNull(!matchingQuestion)) {
                op:log("debug", "No matching question was selected to answer");
                return;
            }
            op:log("debug", "selected question to answer, !matchingQuestion");
            act:answerQuestion(!matchingQuestion,!currSemantics);
        }
      } catch(!failureConditions) {
        !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "handleSemantics(?actor,!currSemantics)");
        !becausePredicate = act:createBecausePredicate(!state, ?speaker, !failureConditions);
        !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(handleSemantics(?actor,!currSemantics),!becausePredicate)");
        act:generateResponse(?speaker, !failureEffects, direct);
      }
    }
}

() = handleError["Handle error semantics"](Symbol ?speaker, Symbol ?addressee, Predicate ?errorSemantics, Symbol ?semanticType) {
    Predicate !response;

    effects : {
      success infer : handled(?actor,?addressee,error(?speaker,?errorSemantics),?semanticType);
    }

    !response = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "ack(sorry,?errorSemantics)");
    act:generateResponse(?speaker, !response, ?semanticType);
}

() = handleAck["Handle ack semantics"](Symbol ?speaker, Symbol ?addressee, Symbol  ?semanticType) {
    effects : {
      success infer : handled(?actor,?addressee,ack(?speaker,okay(?speaker,?actor)),?semanticType);
    }
}

() = handleGreeting["Handle greeting semantics"](Symbol ?speaker, Symbol ?addressee, Symbol ?greeting, Symbol  ?semanticType) {
    Predicate !greetingResponse;

    effects : {
      success infer : handled(?actor,?addressee,greeting(?speaker,?addressee,?greeting),?semanticType);
    }

    op:log("debug", "handleGreeting: ?actor ?speaker ?addressee ?greeting");

    if (op:equals(?addressee, ?actor)) {
        !greetingResponse = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "greeting(?addressee,?speaker,?greeting)");
        act:generateResponse(?speaker, !greetingResponse, ?semanticType);
    }
}

() = handleWantBel["reply to wantBel semantics"](Symbol ?speaker, Symbol ?addressee, Predicate ?fact, Symbol  ?semanticType) {
    Predicate !state;

    effects : {
      success infer : handled(?actor,?addressee,wantBel(?speaker,?addressee,?fact),?semanticType);
    }

    try {
      act:believeFact(?speaker, ?addressee, ?fact, ?semanticType);
      act:believeFact(?speaker, ?actor, ?fact, indirect);
    } catch(!failureConditions) {
      if (op:equals(?addressee, ?actor)) {
        !state = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "bel(?actor,?fact)");
        !becausePredicate = act:createBecausePredicate(!state, ?speaker, !failureConditions);
        !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(bel(?actor,?fact),!becausePredicate)");
        act:generateResponse(?speaker, !failureEffects, direct);
      }
    }
}

() = handleITK["reply to itk semantics"](Symbol ?speaker, Symbol ?addressee, Predicate ?query, Symbol  ?semanticType) {
    Predicate !defaultResponse;
    java.util.List !beliefs;
    java.lang.Boolean !retVal;
    java.util.Map !firstBinding;
    java.util.Set !queryVariables;
    Predicate !knowsQuery;
    Symbol !queryActor;

    effects : {
      success infer : handled(?actor,?addressee,itk(?speaker,?query),?semanticType);
    }

    if (op:equals(?addressee, ?actor)) {
      // try to get into a state of knowing about ?query.
      !queryActor = act:getActorFromQuery(?query);

      // TODO: Issue here is that 'knows' and 'observes' goals don't adhere to potential constraints set by the ExecutionManager.
      //    In the PickAndPlace domain we first did relevant belief queries and reported the system's current belief to the user.
      //    Then asked the user if they wanted the system to go verify its beliefs. If the user said yes but there was a
      //    resource constraint violation with an active goal, then the system denied to verify and reported the conflict.
      //    Do we want something similar here in general? Conditional behavior based on EM subclass and underlying actions
      //    resource requirements could be tricky. Would need to extract the selected action and expose a method indicating
      //    if it causes a resource conflict or not.
      // form goal for ?actor to know ?query, so results can be relayed back to ?speaker
      if (goal:knows(?actor,?query)) {
        op:log("debug", "handleITK: knows goal succeeded.");
        !knowsQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knows(?actor,?query)");
        //TODO: undo this change -> big issue that goals are submitted not through the Execution Manager
        //      Making hacky change to get desired behavior on short notice while covering up this bug
        return;
      } elseif (goal:observe(?query)) {
        op:log("debug", "handleITK: observation goal succeeded.");
        !knowsQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "?query");
      } elseif (~op:isNull(!queryActor)) {
        //TODO: change back this block of code when fixed. Bug somewhere causes askAbout to be called even if !queryActor is null
        //      (&& not short circuiting but written as if it should)
        op:log("debug", "handleITK: askAbout goal enter. !queryActor");
        if (act:askAbout(!queryActor, ?query, !knowsQuery)) {
            op:log("debug", "handleITK: askAbout goal succeeded.");
        } else {
            op:log("debug", "handleITK: knows and obs goal failed, just looking in belief.");
            !knowsQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "?query");
        }
      }
      else {
        op:log("debug", "handleITK: knows and obs goal failed, just looking in belief.");
        !knowsQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "?query");
      }

      // query belief for results
      !beliefs = act:queryBelief(!knowsQuery);
      op:log("debug", "Queried for ?query; result: !beliefs");

      if (op:invokeMethod(!beliefs, "isEmpty")) {
        !queryVariables = op:invokeMethod(?query, "getVars");
        if (op:invokeMethod(!queryVariables, "isEmpty")) {
          !defaultResponse = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "ack(no)");
        } else {
          !defaultResponse = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "?query");
        }

        act:generateResponse(?speaker, !defaultResponse, direct);
      } else {
        !firstBinding = op:invokeMethod(!beliefs, "get", 0);
        if (op:invokeMethod(!firstBinding, "isEmpty")) {

          //performing verbosity check by querying listener expertise
          !defaultResponse = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "novice(?speaker)");
          if (act:querySupport(!defaultResponse)) {
            !defaultResponse = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "ack(yes,?query)");
          } else {
            !defaultResponse = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "ack(yes)");
          }
          act:generateResponse(?speaker, !defaultResponse, direct);
        } else {
          act:generateResponse(?speaker, ?query, !beliefs, direct);
        }
      }
    }
}

() = handleWantDefault["reply to want semantics"](Symbol ?speaker, Symbol ?addressee, Predicate ?state, Symbol ?semanticType) {
    effects : {
      success infer : handled(?actor,?addressee,want(?speaker,?state),?semanticType);
    }

    edu.tufts.hrilab.action.goal.PriorityTier !priorityTier = act:getPriorityTierForGoal(?state);
    act:handleWant(?speaker,?addressee,?state,?semanticType,!priorityTier);
}

() = handleWantPriority["reply to want semantics"](Symbol ?speaker, Symbol ?addressee, Predicate ?state, Symbol ?priorityTierSymbol, Symbol ?semanticType) {
    effects : {
      success infer : handled(?actor,?addressee,want(?speaker,?state,?priorityTierSymbol),?semanticType);
    }

    java.lang.String !priorityString = op:getName(?priorityTierSymbol);
    PriorityTier !priorityTier = op:invokeStaticMethod("edu.tufts.hrilab.action.goal.PriorityTier", "fromString", !priorityString);
    act:handleWant(?speaker,?addressee,?state,?semanticType,!priorityTier);
}

() = handleWant["reply to want semantics"](Symbol ?speaker, Symbol ?addressee, Predicate ?state, Symbol ?semanticType, PriorityTier ?priorityTier = UNINITIALIZED) {
    Predicate !defaultResponse;
    Predicate !failureEffects;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.ActionStatus !actionStatus;
    edu.tufts.hrilab.action.ActionStatus !status;
    Predicate !becausePredicate;
    Predicate !failureReasonPredicate;

    if (op:equals(?addressee, ?actor)) {

      try {
        !defaultResponse = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "ack(okay)");
        act:achieveState(?speaker, ?state, !defaultResponse, !defaultResponse, ?semanticType, ?priorityTier);
      } catch(FAIL_OVERALLCONDITIONS, !failureConditions) {
        !becausePredicate = act:createBecausePredicate(?state, ?speaker, !failureConditions);
        !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(continue(?state),!becausePredicate)");
        act:generateResponse(?speaker, !failureEffects, direct);
      } catch(FAIL_PRECONDITIONS, !failureConditions) {
        !becausePredicate = act:createBecausePredicate(?state, ?speaker, !failureConditions);
        //performing verbosity check by querying listener expertise
        !defaultResponse = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "novice(?speaker)");
        if (act:querySupport(!defaultResponse)) {
          !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(currently(?state),!becausePredicate)");
        } else {
          !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(?state,!becausePredicate)");
        }
        act:generateResponse(?speaker, !failureEffects, direct);
      } catch(FAIL_OBLIGATIONS, !failureConditions) {
        !becausePredicate = act:createBecausePredicate(?state, ?speaker, !failureConditions);
        !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "shouldnot(?state,!becausePredicate)");
        act:generateResponse(?speaker, !failureEffects, direct);
      } catch(!failureConditions) {
        !becausePredicate = act:createBecausePredicate(?state, ?speaker, !failureConditions);
        !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(?state,!becausePredicate)");
        act:generateResponse(?speaker, !failureEffects, direct);
      }
    }
}

() = handleRecovery["reply to recovery semantics"](Symbol ?speaker, Predicate ?state, Predicate ?recovery, Symbol  ?semanticType) {
    Predicate !defaultResponse;
    Predicate !failureEffects;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.ActionStatus !actionStatus;
    edu.tufts.hrilab.action.ActionStatus !status;
    Predicate !becausePredicate;
    Predicate !failureReasonPredicate;
    Predicate !recoveryPred;

    effects : {
      success infer : handled(?actor,recover(want(?speaker,?state),?recovery),?semanticType);
    }

    try {
      !defaultResponse = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "ack(okay)");
      act:achieveState(?speaker, ?recovery, !defaultResponse, !defaultResponse, indirect);
    } catch(FAIL_PRECONDITONS, !failureConditions) {
      op:log("debug", "catching recovery before the new state is handled");
      !becausePredicate = act:createBecausePredicate(?state, ?speaker, !failureConditions);
      !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "willnot(?state,!becausePredicate)");
      act:generateResponse(?speaker, !failureEffects, direct);
      op:log("debug", "exiting itkL");
      return;
    } catch(!failureConditions) {
      op:log("debug", "catching recovery before the new state is handled");
      !becausePredicate = act:createBecausePredicate(?state, ?speaker, !failureConditions);
      !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(?state,!becausePredicate)");
      act:generateResponse(?speaker, !failureEffects, direct);
      op:log("debug", "exiting itkL");
      return;
    }

    try {
      !defaultResponse = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "ack(okay)");
      act:achieveState(?speaker, ?state, !defaultResponse, !defaultResponse, ?semanticType);
    } catch(FAIL_OVERALLCONDITIONS, !failureConditions) {
      !becausePredicate = act:createBecausePredicate(?state, ?speaker, !failureConditions);
      !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(continue(?state),!becausePredicate)");
      act:generateResponse(?speaker, !failureEffects, direct);
    } catch(!failureConditions) {
      !becausePredicate = act:createBecausePredicate(?state, ?speaker, !failureConditions);
      !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cannot(?state,!becausePredicate)");
      act:generateResponse(?speaker, !failureEffects, direct);
      op:log("debug", "prompting further error recovery instruction");
      //!recoveryPred = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "ack(what should I do next)");
      //act:generateResponse(?speaker, !recoveryPred, direct);
    }

}

() = believeFact["believe that ?fact is true from agent ?speaker"](Symbol ?speaker, Symbol ?addressee, Predicate ?fact, Symbol ?semanticType, Predicate ?responseSemantics = ack(okay)) {
    conditions : {
      pre : trust(?addressee,?speaker);
    }
    effects : {
      success infer: believes(?addressee,?fact);
    }

    // TODO: EAK: isnt this is problematic for multi-agent cases? why do we want ?fact AND believes(?addressee, ?fact)
    //brad: workaround for bel() wrapper issues with pddl
    act:believeFactHelper(?fact);

    if (op:equals(?addressee,?actor)) {
      act:generateResponse(?speaker, ?responseSemantics, ?semanticType);
    }
}

// Is this universally desired?
// Properly retract negated facts rather than asserting negated form, also unwrap 'and' predicates.
() = believeFactHelper(Predicate ?fact) {
    Symbol !factName = op:getName(?fact);
    java.util.List !args;
    Symbol !arg;
    Term !negatedFact;

    if (op:equals(!factName,and)) {
        !args = op:getArgs(?fact);
        foreach(!arg : !args) {
            act:believeFactHelper(!arg);
        }
    } else {
        if (op:invokeMethod(?fact,"isNegated")) {
            !negatedFact = op:invokeMethod(?fact,"toUnnegatedForm");
            act:retractBelief(!negatedFact);
            op:log("debug", "retracted belief !negatedFact.");
        } else {
            act:assertBelief(?fact);
            op:log("debug", "Now believes ?fact.");
        }
    }
}

() = believeRule["believe that ?rule is true from agent ?speaker"](Symbol ?speaker, Predicate ?head, Predicate ?body, Symbol ?semanticType, Predicate ?responseSemantics = ack(okay)) {
    conditions : {
      pre : trust(?actor,?speaker);
    }
    java.util.List !args;
    java.lang.String !predName;
    java.util.Set !bodySet;
    Predicate !goal;
    java.lang.Integer !argsSize;

    !predName = op:invokeMethod(?body, "getName");
    !bodySet = op:newHashSet("edu.tufts.hrilab.fol.Predicate");
    if (op:equals(!predName,"and")) {
      !args = op:invokeMethod(?body, "getArgs");
      op:invokeMethod(!bodySet, "addAll", !args);
    } else {
      op:invokeMethod(!bodySet, "add", ?body);
    }
    act:assertSimpleRule(!head, !bodySet);
    act:generateResponse(?speaker, ?responseSemantics, ?semanticType);
}

() = achieveState["try to achieve desired state"](Symbol ?speaker, Predicate ?state, Predicate ?tentativeAcceptSemantics, Predicate ?acceptSemantics, Symbol ?semanticType, PriorityTier ?priorityTier = UNINITIALIZED) {
    Predicate !defaultResponse;
    Predicate !failureEffects;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.ActionStatus !actionStatus;
    edu.tufts.hrilab.action.justification.Justification !failureConditions;
    Predicate !becausePredicate;
    java.lang.Boolean !madeTentativeAccept = false;
    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;
    java.lang.Boolean !waitingForResponse;
    Predicate !responsePredicate;
    java.lang.Long !priority;

    conditions : {
      or : {
        pre infer : supervisor_of(?speaker,?actor);
        pre infer : isAdminGoal(?state);
        pre infer : isOpenGoal(?state);
      }
      or : {
        pre infer : supervisor_of(?speaker,?actor);
        pre infer : admin_of(?speaker,?actor);
        pre infer : isOpenGoal(?state);
      }
      or : {
        pre infer : not(isAdminGoal(?state));
        pre infer : admin_of(?speaker,?actor);
        pre infer : isOpenGoal(?state);
      }
      //obligation infer : not(isUnsafe(?state));
      //obligation infer : not(isForbidden(?state));
    }

    // Q: The motivation behind using SKIPPENDING (i.e. lack of resource conflict) to be the conditional here was to still
    //  allow the user to do things like ask the system for relevant information to inform question answers, modify task
    //  execution, or supply information to the system. Should there be a different check made?
    //      handleITK and handleWantBel go through separate pipelines, so maybe this is an accurate check.
    // If the agent being tasked with a goal which doesn't skip the queue is currently awaiting user response to a question
    //  it has asked, reject the goal and request an answer to the question first (In practice found that this case leads
    //  to users frequently being confused about system state. Also allows for potential of overlapping questions to be asked).
    if (~op:equalsValue(?priorityTier,SKIPPENDING)) {
        //TODO: Should this be modified to check if the agent is waiting for a response to a question specifically asked by this speaker? This would require
        //      a solution to the issue of all questions actually being addressed to the correct entity - which in many cases requires attributing from within all actions which entity's command caused this action to be performed
        if (~op:equalsValue(?priorityTier,URGENT)) {
            if (tsc:isAgentWaitingForResponse(?actor)) {
                //act:generateResponseFromString(?speaker, "Please answer my question before sending further instructions");
                act:generateResponse(?speaker, please(before(youSendFurtherInstruction,answer(myQuestion))));
                return;
            }
        }
    }

    //Can also reimplement ignoreTentativeAccept filter if desired
    if(~act:checkIgnoreTentativeAccept(?state)){
        act:generateResponse(?speaker, ?tentativeAcceptSemantics, ?semanticType);
    }
    !madeTentativeAccept = op:newObject("java.lang.Boolean", "true");

    !priority = act:getPriorityForGoal(?state);
    !goalID = act:submitGoal(?state,!execType,?priorityTier,!priority,?speaker);
    //!goalID = act:submitGoal(?state,!execType,?priorityTier);
    !goalStatus = act:joinOnGoal(!goalID, 1000);
//    if (~op:invokeMethod(!goalStatus, "isTerminated")) {
//      op:log("debug", "tentative accept for ?state with status !goalStatus");
//      act:generateResponse(?speaker, ?tentativeAcceptSemantics, ?semanticType);
//      !madeTentativeAccept = op:newObject("java.lang.Boolean", "true");
//    }

    op:log("debug", "waiting to get into state: ?state goalId: !goalID");
    !goalStatus = act:joinOnGoal(!goalID);
    op:log("debug", "?state status !goalStatus");
    if (op:invokeMethod(!goalStatus, "isFailure")) {
      op:log("debug", "Action failed, goal: !goalID");
      !failureConditions = act:getGoalFailConditions(!goalID);
      !actionStatus = act:getActionStatus(!goalID);
      op:log("debug", "action status: !actionStatus");
      exit(!actionStatus, !failureConditions);
    } elseif (op:==(!madeTentativeAccept, false)) {
      op:log("debug", "?state achieved with status !goalStatus");
      act:generateResponse(?speaker, ?acceptSemantics, ?semanticType);
    }
}

(Question ?matchingQuestion) = selectQuestionToAnswer(java.util.Map ?questions, Symbol ?speaker) {
    java.util.List !agentQuestions;
    java.lang.Integer !numAgents = 0;
    java.util.Set !agentSet;
    Symbol !key;
    Symbol !agent;
    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Variable !x = "X";
    Symbol !response;
    java.lang.String !numberString;

    op:log("debug", "[selectQuestionToAnswer] in method");

    //Check ?actor first
    !agentQuestions = op:invokeMethod(?questions,"get",?actor);
    ?matchingQuestion = act:selectQuestionToAnswerHelper(!agentQuestions, ?speaker);
    if (~op:isNull(?matchingQuestion)) {
        op:log("debug", "[selectQuestionToAnswer] single matching question for actor, returning");
        //Have disambiguous question for direct ?actor
        return;
    } else {
        //Only reaches this block if ?actor did not have any matching questions
        op:log("debug", "[selectQuestionToAnswer] no matching question for actor, searching team members");
        !agentSet = op:invokeMethod(?questions,"keySet");
        foreach (!key : !agentSet) {
            !numAgents = op:++(!numAgents);
        }
        //!numAgents = op:invokeMethod(!agentSet,"size");
        if (op:equals(!numAgents,1)) {
            op:log("debug", "[selectQuestionToAnswer] single team member has matching questions, answering for child");
            //Only one agent has asked questions, disambiguate for that agent
            foreach(!agent : !agentSet) {
                !agentQuestions = op:invokeMethod(?questions,"get",!agent);
                ?matchingQuestion = act:selectQuestionToAnswerHelper(!agentQuestions, ?speaker);
                if (~op:isNull(?matchingQuestion)) {
                    return;
                }
            }
        } else {
            //Multiple agents have asked questions, determine which agent the user is meaning to address
            op:log("debug", "[selectQuestionToAnswer] multiple team members have matching questions, selecting amongst them");
            act:generateResponseFromString(?speaker, "Multiple agents have asked questions relevant to your response. Possible candidates are:");
            foreach(!agent : !agentSet) {
                act:generateResponseFromString(?speaker, !agent);
            }
            //TODO: should be updated to handle potential question overlap properly
            act:suspendWaitForResponse(agent(X),?actor,false);
            !bindings = act:askQuestionFromString(?speaker,"Which one did you mean to address?", agent(X));
            act:resumeWaitForResponse(agent(X),?actor);
            if (~op:invokeMethod(!bindings,"isEmpty")) {
                !response = op:get(!bindings,!x);
                !response = op:invokeMethod(!response,"toUntyped");
                !agentQuestions = op:invokeMethod(?questions,"get",!response);
                if (op:isNull(!agentQuestions)) {
                    act:generateResponseFromString(?speaker, "agent !response was not a valid option");
                } else {
                    ?matchingQuestion = act:selectQuestionToAnswerHelper(!agentQuestions, ?speaker);
                    return;
                }
            }
        }

    }
    exit(FAIL,not(know(?actor,whichQuestionYouAreAnswering)));
}

(Question ?matchingQuestion) = selectQuestionToAnswerHelper(java.util.List ?agentQuestions, Symbol ?speaker) {
    Symbol !agent;
    java.lang.Integer !numQuestions = 0;
    Predicate !errorPred;
    //Predicate !disambiguationPred;
    java.lang.Integer !questionIndex;
    java.lang.Integer !spokenIndex;
    java.lang.String !questionString;
    Question !question;
    java.util.Map !bindings;
    edu.tufts.hrilab.fol.Variable !x = "X";
    Symbol !response;
    java.lang.String !numberString;
    java.lang.Integer !questionNumber;

    if (op:isNull(?agentQuestions)) {
        return;
    } elseif (op:isEmpty(?agentQuestions)) {
        return;
    }
    op:log("debug", "[selectQuestionToAnswerHelper] in method ?agentQuestions");

    //TODO: still need to figure out multilingual solution
    foreach(!question : ?agentQuestions) {
        !numQuestions = op:++(!numQuestions);
    }
    //!numQuestions = op:invokeMethod(?agentQuestions,"size");
    if (op:equals(!numQuestions,1)) {
        op:log("debug", "[selectQuestionToAnswerHelper] single matching question, returning");
        //One matching question for direct agent, select
        ?matchingQuestion = op:get(?agentQuestions,0);
        return;
    } elseif (op:gt(!numQuestions,1)) {
        op:log("debug", "[selectQuestionToAnswerHelper] multiple matching question, asking disambiguation question");
        //Multiple matching questions for direct agent, disambiguate between them
        //act:generateResponseFromString(?speaker, "It is unclear which question you are answering. Possible candidates are:");
        act:generateResponse(?speaker, and(report(itIsUnclearWhichQuestionYouAreAnswering),promptList(possibleCandidatesAre)));
        for (!questionIndex=0; !questionIndex lt !numQuestions; !questionIndex ++) {
            !question = op:get(?agentQuestions,!questionIndex);
            !questionString = op:invokeMethod(!question,"getQuestionString");
            !spokenIndex = op:++(!questionIndex);
            act:generateResponseFromString(?speaker, "Option !spokenIndex: !questionString");
        }
        //TODO: should be updated to handle potential question overlap properly
        act:suspendWaitForResponse(option(X),?actor,false);
        //!bindings = act:askQuestionFromString(?speaker, "Which question did you mean to answer?", option(X));
        !bindings = act:askQuestion(?speaker, meanToAnswer(which(question)), option(X));
        act:resumeWaitForResponse(option(X),?actor);
        if (~op:invokeMethod(!bindings,"isEmpty")) {
            !response = op:get(!bindings,!x);
            !numberString = op:getName(!response);
            !questionNumber = op:newObject("java.lang.Integer", !numberString);
            !questionNumber = op:--(!questionNumber);
            if (op:lt(!questionNumber,!numQuestions)) {
                ?matchingQuestion = op:get(?agentQuestions,!questionNumber);
                return;
            } else {
                //act:generateResponseFromString(?speaker, "Question !questionNumber was not a valid option");
                act:generateResponse(?speaker, not(validOption(question(!questionNumber))));
            }
        }
    }
    op:log("debug", "[selectQuestionToAnswerHelper] unclear which question was being answered");
    exit(FAIL,not(know(?actor,whichQuestionYouAreAnswering)));
}