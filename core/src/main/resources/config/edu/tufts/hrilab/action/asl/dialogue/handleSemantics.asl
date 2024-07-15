import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import java.lang.Integer;

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
      !failureEffects = op:invokeStaticMethod("edu.tufts.hrilab.slug.dialogue.ResponseGeneration", "generateClarificationMultipleMeanings", ?actor, ?speaker, ?semantics);
      // TODO: this should use askQuestion instead of generateResponse
      act:generateResponse(?speaker, !failureEffects, direct);
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
        // TODO:brad:the way things are right now this is the only way we can do this.
        //      Ideally we could move this logic into an action selector, so we can
        //      remove the if else and have it  just be the one state spec
        if(act:isQuestionResponse(?speaker,?addressee,!currSemantics)){
          act:answerQuestion(?speaker,?addressee,!currSemantics);
        } else{
          goal:handled(?actor, ?addressee, !currSemantics, ?semanticType);
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

      // form goal for ?actor to know ?query, so results can be relayed back to ?speaker
      if (goal:knows(?actor,?query)) {
        op:log("debug", "handleITK: knows goal succeeded.");
        !knowsQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "knows(?actor,?query)");
      } elseif (goal:observe(?query)) {
        op:log("debug", "handleITK: observation goal succeeded.");
        !knowsQuery = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "?query");
      } elseif (~op:isNull(!queryActor) && act:askAbout(!queryActor, ?query, !knowsQuery)) {
        op:log("debug", "handleITK: askAbout goal succeeded.");
      } else {
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
    java.lang.String !priorityString = op:invokeMethod(!priorityTier,"name");
    Symbol !priority = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", !priorityString);
    act:handleWant(?speaker,?addressee,?state,?semanticType,!priority);
}

() = handleWantPriority["reply to want semantics"](Symbol ?speaker, Symbol ?addressee, Predicate ?state, Symbol ?priority, Symbol ?semanticType) {
    effects : {
      success infer : handled(?actor,?addressee,want(?speaker,?state,?priority),?semanticType);
    }

    act:handleWant(?speaker,?addressee,?state,?semanticType,?priority);
}

() = handleWant["reply to want semantics"](Symbol ?speaker, Symbol ?addressee, Predicate ?state, Symbol ?semanticType, Symbol ?priority = uninitialized) {
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
        act:achieveState(?speaker, ?state, !defaultResponse, !defaultResponse, ?semanticType, ?priority);
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
    act:assertBelief(?fact);
    op:log("debug", "Now believes ?fact.");

    if (op:equals(?addressee,?actor)) {
      act:generateResponse(?speaker, ?responseSemantics, ?semanticType);
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

() = achieveState["try to achieve desired state"](Symbol ?speaker, Predicate ?state, Predicate ?tentativeAcceptSemantics, Predicate ?acceptSemantics, Symbol ?semanticType, Symbol ?priority = uninitialized) {
    Predicate !defaultResponse;
    Predicate !failureEffects;
    java.lang.Long !goalID;
    edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
    edu.tufts.hrilab.action.ActionStatus !actionStatus;
    edu.tufts.hrilab.action.justification.Justification !failureConditions;
    Predicate !becausePredicate;
    java.lang.Boolean !madeTentativeAccept = false;
    edu.tufts.hrilab.action.execution.ExecutionType !execType = ACT;

    conditions : {
      or : {
        pre infer : is_supervisor(?speaker,?actor);
        pre infer : isAdminGoal(?state);
        pre infer : isOpenGoal(?state);
      }
      or : {
        pre infer : is_supervisor(?speaker,?actor);
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


    //Can also reimplement ignoreTentativeAccept filter if desired
    if(~act:checkIgnoreTentativeAccept(?state)){
        act:generateResponse(?speaker, ?tentativeAcceptSemantics, ?semanticType);
    }
    !madeTentativeAccept = op:newObject("java.lang.Boolean", "true");


    !goalID = act:submitGoal(?state,!execType,?priority);
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
