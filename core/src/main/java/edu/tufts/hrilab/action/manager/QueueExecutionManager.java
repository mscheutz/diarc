/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class QueueExecutionManager extends ExecutionManager {
  private static final Logger log = LoggerFactory.getLogger(QueueExecutionManager.class);

  public QueueExecutionManager() {
    super();
  }

  protected void init(StateMachine sm, RootContext rootContext, Collection<String> groups) {
    super.init(sm, rootContext,groups);
  }

  @Override
  protected void handleConflictingLowerPriorityGoal(Goal g) {
    //QueueEM: Simply leave in the pending queue rather than rejecting
    //This goal should already have PENDING status (unless it was superseded), but setting status explicitly here in
    // order to trigger goalStatusUpdate listeners. Otherwise, listeners would have no idea a goal was submitted but
    // blocking in the queue.
    if (g.getStatus() == GoalStatus.PENDING) {
      g.setStatus(GoalStatus.PENDING);
    }

    //TODO: do we want these status notifications? <- should make configurable based on a notion of "system verbosity"
    //PickAndPlace2024 - Report that the goal is being added to the queue
    Symbol actor = g.getActor();
    if (!actor.getName().equals("self")) {
      //Currently need to use semantics rather than string for NLG for flexibility between EN/JP
      //String statusReport = null;
      Predicate statusReportSemantics = null;
      List<Predicate> conflictingGoals = getSystemGoalsPredicates(g.getActor(), true);
      if (!conflictingGoals.isEmpty()) {
        if (conflictingGoals.get(0).getName().equals("active")) {
          statusReportSemantics = Factory.createPredicate("addingThatToMyCurrentQueueOfTasks()");
          //statusReport = "okay, adding to my current queue of tasks";
        } else if (conflictingGoals.get(0).getName().equals("suspended")) {
          //Could add currently suspended goal predicate to semantics and corresponding examples to NLG prompt if we want to report what the goal is
          statusReportSemantics = Factory.createPredicate("myCurrentTaskIsSuspendedAddingThisToMyQueue()");
          //statusReport = "okay, my current task is suspended. Adding this to my queue";
        }
        //if (statusReport != null) {
        if (statusReportSemantics != null) {
          Utterance utt = new Utterance.Builder()
                  .setSpeaker(g.getActor())
                  .setAddressee(g.getSubmitter())
                  .setUtteranceType(UtteranceType.REPLY)
                  .setIsInputUtterance(false)
                  //.setWords(List.of(statusReport.split(" ")))
                  .setSemantics(statusReportSemantics)
                  .build();
          try {
            utt = TRADE.getAvailableService(new TRADEServiceConstraints().name("convertSemanticsToText").argTypes(Utterance.class)).call(Utterance.class, utt);
            TRADE.getAvailableService(new TRADEServiceConstraints().name("updateDialogueHistory").argTypes(Utterance.class)).call(void.class, utt);
            TRADE.getAvailableService(new TRADEServiceConstraints().name("sayText").argTypes(Utterance.class)).call(void.class, utt);
          } catch (TRADEException e) {
            log.error("[handleConflictingLowerPriorityGoal]", e);
          }
        }
      }
    }
  }

  //TODO: extend definition of resources past just whole agents/teams
  @Override
  protected Set<Resource> getRequiredResourcesForGoal(Goal goal) {
    if (goal.getRequiredResources() != null) {
      return goal.getRequiredResources();
    } else {
      Set<Resource> resourceSet = new HashSet<>();

      //Iterate over all gathered agents and pool agent wide resources
      for (Symbol agent : getDescendants(goal.getActor())) {
        //If the goal skips the queue, it has no resource dependencies.
        if (goal.getPriorityTier() != PriorityTier.SKIPPENDING) {
          if (!goal.isLearningGoal()) {
            //If it is a not learning goal, requires the learning lock to be available.
            resourceSet.add(agentTeams.get(agent).getResource(Factory.createSymbol("learning")));
          }
          //Requires placeholder agent-wide lock to be available
          resourceSet.add(agentTeams.get(agent).getResource(agent));
          //TODO: are skippending goals allowed for frozen agents? Needs to be some sort of exception at least in order
          //      to command the agent to unfreeze
          //Always requires the agent to be unfrozen
          resourceSet.add(agentTeams.get(agent).getResource(Factory.createSymbol("freeze")));
        }
      }
      //Store required resources in goal to prevent constantly recomputing set
      goal.setRequiredResources(resourceSet);

      return resourceSet;
    }
  }
  @Override
  protected Set<Resource> getHeldResourcesForGoal(Goal goal) {
    if (goal.getHeldResources() != null) {
      return goal.getHeldResources();
    } else {
      Set<Resource> resourceSet = new HashSet<>();

      //Iterate over all gathered agents and pool agent wide resources
      for (Symbol agent : getDescendants(goal.getActor())) {
        //If the goal skips the queue, it holds no resources
        if (goal.getPriorityTier() != PriorityTier.SKIPPENDING) {
          if (!goal.isLearningGoal()) {
            //TODO: Not sure if we want to ever teach teaching or teach freeze? If not, get rid of above conditional block
            if (goal.getPredicate().getName().equals("learnAction")) {
              //Action learning itself takes the learning lock - prevents non-learning goals from being submitted
              resourceSet.add(agentTeams.get(agent).getResource(Factory.createSymbol("learning")));
              continue;
            } else if (goal.getPredicate().getName().equals("freeze")) {
              //Freeze itself takes the freeze lock - prevents everything else from being submitted
              resourceSet.add(agentTeams.get(agent).getResource(Factory.createSymbol("freeze")));
              continue;
            }
          }
          //In all other cases, add agent resource so multiple goals cannot overlap
          resourceSet.add(agentTeams.get(agent).getResource(agent));
        }
      }
      //Store required resources in goal to prevent constantly recomputing set
      goal.setHeldResources(resourceSet);

      return resourceSet;
    }
  }
}
