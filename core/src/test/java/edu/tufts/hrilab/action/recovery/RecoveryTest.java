/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.justification.AndJustification;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.junit.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RecoveryTest {
    private GoalManagerComponent component;
    private static Logger log = LoggerFactory.getLogger(RecoveryTest.class);
    private String aslPath = "/config/edu/tufts/hrilab/action/asl/recovery/";

    @After
    public void shutdown(){
        component.shutdownComponent();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Helper method to submit a goal, wait for it to finish, and then check the status.
     * @param goalPredicate
     * @param desiredStatus
     */
    private void executeAndCheckGoal(Predicate goalPredicate, GoalStatus desiredStatus) {
        Long goalId = component.submitGoal(goalPredicate);
        component.joinOnGoal(goalId);
        Assert.assertEquals(desiredStatus, component.getGoalStatus(goalId));
    }

    /**
     * Helper method for shouldAttemptRecoverytest to check if the desired recovery policy was in fact executed
     */
    private void checkRecovery(Predicate goalPred, String recoveryActionName, ActionStatus desiredStatus) {
        if (goalPred.getName().equals("goal")) {
            goalPred = (Predicate) goalPred.get(1);
        }
        Long goalId = component.getGoalId(goalPred);
        Goal goal = component.getGoal(goalId);
        Context recoveryActionContext = goal.getCurrentContext().getParentContext();
        if (recoveryActionContext instanceof ActionContext) {
            Assert.assertEquals(recoveryActionContext.getCommand(), recoveryActionName);
            Assert.assertEquals(recoveryActionContext.getStatus(),desiredStatus);
            return;
        }
        Assert.fail("Recovery policy not executed");
    }

    //This test might not be the most helpful past checking whether every case works as a set
    //  (i.e. not sure if it will pinpoint the specific error because so many cases overlap)
    //ASL file for these tests is also pretty confusing, could probably just test the function directly
    /**
     * Tests checking whether recovery is properly triggered based on failure information and policy constraints
     */
    @Test
    public void shouldAttemptRecoveryTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl");
        String filename = aslPath + "shouldAttemptRecoveryTest.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        //GoalRecovery
        //Goals match -> recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: Recovery policy applies through goals match");
        Predicate goalPredicate = Factory.createPredicate("goal(self:agent,stateTwo(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        checkRecovery(goalPredicate, "actionTwoRecovery", ActionStatus.SUCCESS);

        //failedActions match -> recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: Recovery policy applies through failed action match");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateThree(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        checkRecovery(goalPredicate, "actionThreeRecovery", ActionStatus.SUCCESS);

        //failureReasons match -> recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: Recovery policy applies through failure reason match");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateFour(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        checkRecovery(goalPredicate, "actionFourRecovery", ActionStatus.SUCCESS);

        //actionStatuses match -> recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: Recovery policy applies through action status match");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateFive(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        //TODO: race condition with overallStatus monitor triggering and action completing?
        checkRecovery(goalPredicate, "actionFiveRecovery", ActionStatus.SUCCESS);

        //Nothing matches -> don't recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: No recovery policy defined");
        goalPredicate = Factory.createPredicate("actionOne(self:agent,fail)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

        //excludedGoals match and failedActions match -> don't recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: Failed goal matches recovery policy failedActions and excludedGoals");
        goalPredicate = Factory.createPredicate("actionThree(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

        //excludedFailureReasons match and failedActions match  -> don't recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: Failed goal matches recovery policy failedActions and excludedFailureReasons");
        goalPredicate = Factory.createPredicate("actionSix(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

        //excludedFailedActions match and actionStatuses match -> don't recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: Failed goal matches recovery policy actionStatuses and excludedFailedActions");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateSeven(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

        //ActionRecovery
        //Catching action matches -> recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: ActionRecovery catchingAction matches");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateEight(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        checkRecovery(goalPredicate, "actionEightRecovery", ActionStatus.SUCCESS);

        //Catching action does not match -> don't recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: ActionRecovery catchingAction does not match");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateNine(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

        //Catching action matches, but excluded also matches -> don't recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: ActionRecovery catchingAction and excluded reason both match");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateTen(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

        //No explicit catching action defined and goals match -> recover
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: ActionRecovery no catchingAction defined, goals match");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateEleven(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        checkRecovery(goalPredicate, "actionElevenRecovery", ActionStatus.SUCCESS);

        //Multiple recovery policies match -> better recovery policy is selected (more matches)
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: Multiple policies match");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateTwelve(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        checkRecovery(goalPredicate, "betterActionTwelveRecovery", ActionStatus.SUCCESS);

        //No explicit catching action defined, and no other match -> don't recover (or else would recover on literally every failure)
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: ActionRecovery no catchingAction defined, no other matches");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateThirteen(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

        //Unachievable goal state, can't plan
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: GoalRecovery cannot plan to goal state");
        goalPredicate = Factory.createPredicate("goal(self:agent,stateFourteen(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);

        //Goal is direct action instruction, recovery policy defined to apply to top level action context
        log.info("\n\n\n[shouldAttemptRecoveryTest] Test case: ActionRecovery top level action");
        goalPredicate = Factory.createPredicate("testActionSuccess(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
    }

    /**
     *Tests goal recovery where recovery is directly to the desired goal state
     */
    @Test
    public void goalRecoveryToGoalStateTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "goalRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[goalRecoveryToGoalStateTest]");
        //Submit state based goal for holding, recovery should cause goal to succeed
        Predicate goalPredicate = Factory.createPredicate("goal(self:agent,holding(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        goalPredicate = (Predicate) goalPredicate.get(1);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        //PickUp - should have success status
        Context pickUpContext = goal.getRootContext();
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 2);
        Assert.assertEquals(pickUpContext.getStatus(), ActionStatus.SUCCESS);

        //First child: goal fails because we cannot plan
        Context failureContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(failureContext.getStatus(), ActionStatus.FAIL_NOTFOUND);

        //Second child: recovery context (Recovery policy has holding(self:agent) as an effect)
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryGoalContext.getCommand(), "goal(pickupFullRecovery(self:agent,[],[found(self:agent,holding(self:agent)),holding(self:agent)))");
        Assert.assertEquals(recoveryGoalContext.getStatus(), ActionStatus.SUCCESS);

        //There are no more children because the goal was achieved
    }

    /**
     * Tests goal recovery where recovery results in a plannable state
     */
    @Test
    public void goalRecoveryToPlannableStateTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "goalRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");


        log.info("\n\n\n[goalRecoveryToPlannableStateTest]");
        //Submit goal for pickUp - recovery should cause goal to succeed
        Predicate goalPredicate = Factory.createPredicate("pickUp(self:agent,success)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        //PickUp context - 3 children
        Context pickUpContext = goal.getRootContext();
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 3);

        //First child: First execution attempt - should fail due to gripperOpen precondition not being satisfied
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(pickupContext.getCommand(), "pickUp");
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Second child: Recovery context (policy has gripperOpen(self:agent) as an effect)
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryGoalContext.getCommand(), "goal(pickupPartialRecovery(self:agent,[pickup(self:agent), pickUp(self:agent,success),[gripperOpen(self:agent),pickUp(self:agent,success)))");
        Assert.assertEquals(recoveryGoalContext.getStatus(), ActionStatus.SUCCESS);

        //Third child: Second execution attempt - should now succeed
        pickupContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(pickupContext.getCommand(), "pickUp");
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.SUCCESS);
    }

    /**
     * Tests goal recovery where the recovery policy itself fails
     */
    @Test
    public void goalRecoveryFailsTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "goalRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[goalRecoveryFailsTest]");
        //Submit pickUp goal that will fail, should have failed status
        Predicate goalPredicate = Factory.createPredicate("pickUp(self:agent,failure)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        //PickUp context: should have original failure status and justification (can't pickup without gripperOpen precondition)
        Context pickUpContext = goal.getRootContext();
        Assert.assertEquals(pickUpContext.getStatus(),ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickUpContext.getJustification(),new AndJustification(new ConditionJustification(false, Factory.createPredicate("gripperOpen(self:agent)"))));
        //Should only have 2 children - no more execution after failed recovery
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 2);

        //First child: failed pickup with original failure reasons
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(pickupContext.getStatus(),ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickupContext.getJustification(),new AndJustification(new ConditionJustification(false, Factory.createPredicate("gripperOpen(self:agent)"))));

        //Second child: Recovery context itself should fail with FAIL_RECOVERY status
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryGoalContext.getCommand(), "goal(pickupFailRecovery(self:agent,[pickup(self:agent), pickUp(self:agent,failure),[gripperOpen(self:agent),pickUp(self:agent,failure)))");
        Assert.assertEquals(recoveryGoalContext.getStatus(), ActionStatus.FAIL_RECOVERY);

        //Child of recovery context should have had actual failure reasons of the recovery policy execution propagated up
        Context recoveryActionContext = recoveryGoalContext.getChildContexts().get(0);
        Assert.assertEquals(recoveryActionContext.getStatus(), ActionStatus.FAIL_NOTFOUND);
        Assert.assertEquals(recoveryActionContext.getJustification().getFailureReason().get(0), Factory.createPredicate("not(found(self:agent,fakeAction(self:agent)))"));
    }

    /**
     * Tests goal recovery where the recovery policy intentionally exits with EXIT_RECOVERY status
     */
    @Test
    public void goalRecoveryExitRecoveryTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "goalRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[goalRecoveryExitRecoveryTest]");
        //Submit pickup goal that will be picked up by the correct recovery policy. Should fail after recovery exits
        Predicate goalPredicate = Factory.createPredicate("pickUp(self:agent,exit)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        //pickUp should fail with the original failure reason and justification
        Context pickUpContext = goal.getRootContext();
        Assert.assertEquals(pickUpContext.getStatus(),ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickUpContext.getJustification(),new AndJustification(new ConditionJustification(false, Factory.createPredicate("gripperOpen(self:agent)"))));
        //Should have 2 children, no execution after exiting recovery
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 2);

        //First child: failed pickup with original failure reasons
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(pickupContext.getStatus(),ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickupContext.getJustification(),new AndJustification(new ConditionJustification(false, Factory.createPredicate("gripperOpen(self:agent)"))));

        //Recovery context itself should fail with exit_recovery status
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryGoalContext.getCommand(), "goal(pickupExitRecovery(self:agent,[pickup(self:agent), pickUp(self:agent,exit),[gripperOpen(self:agent),pickUp(self:agent,exit)))");
        Assert.assertEquals(recoveryGoalContext.getStatus(), ActionStatus.EXIT_RECOVERY);

        //Recovery child should fail with exit_recovery
        Context recoveryActionContext = recoveryGoalContext.getChildContexts().get(0);
        Assert.assertEquals(recoveryActionContext.getStatus(), ActionStatus.EXIT_RECOVERY);
    }

    /**
     * Tests action recovery where a catchingAction is defined and the failed context is a direct child of that action
     * Also implicitly tests postRecoveryExecution SUCCESS
     */
    @Test
    public void actionRecoveryCatchDefinedChildTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "actionRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[actionRecoveryCatchDefinedChildTest]");
        //Submit holding goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("goal(self:agent,holding(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        goalPredicate = (Predicate) goalPredicate.get(1);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        Context goalContext = goal.getRootContext();
        Assert.assertEquals(goalContext.getChildContexts().size(), 1);
        //Plan generated for goal
        Context plannedContext = goalContext.getChildContexts().get(0);
        Assert.assertEquals(plannedContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(plannedContext.getCommand(), "planned");
        Assert.assertEquals(plannedContext.getChildContexts().size(), 2);

        //First step in plan, opengripper. Should have success status after successful recovery
        Context opengripperContext = plannedContext.getChildContexts().get(0);
        Assert.assertEquals(opengripperContext.getStatus(), ActionStatus.SUCCESS);
        //Should have four children
        Assert.assertEquals(opengripperContext.getChildContexts().size(), 4);

        //Child one: log statement

        //Child two: Failed nonexistent action
        Context failnotfoundContext = opengripperContext.getChildContexts().get(1);
        Assert.assertEquals(failnotfoundContext.getStatus(), ActionStatus.FAIL_NOTFOUND);

        //Child three: Recovery context
        Context recoveryGoalContext = opengripperContext.getChildContexts().get(2);
        Assert.assertEquals(recoveryGoalContext.getCommand(), "goal(openGripperActionRecovery(self:agent,[],[found(self:agent,fakeAction(self:agent)),holding(self:agent)))");
        Assert.assertEquals(recoveryGoalContext.getStatus(), ActionStatus.SUCCESS);
        Context recoveryActionContext = recoveryGoalContext.getChildContexts().get(0);
        Assert.assertEquals(recoveryActionContext.getStatus(), ActionStatus.SUCCESS);

        //postrecovery execution defined as postrecovery_success -> pickup continues from step after failed action
        //Child four: log statement
        Context logContext = opengripperContext.getChildContexts().get(3);
        Assert.assertEquals(logContext.getStatus(), ActionStatus.SUCCESS);

        //Second step in plan executed
        Context pickupContext = plannedContext.getChildContexts().get(1);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.SUCCESS);

    }

    /**
     * Tests action recovery where a catching action is defined and the failed context is a deeper descendent of that action
     */
    @Test
    public void actionRecoveryCatchDefinedGrandchildTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "actionRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[actionRecoveryCatchDefinedGrandChildTest]");
        //Submit pickup goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("pickUp(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 1);
        Context pickUpContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(pickUpContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickUpContext.getCommand(), "pickUp");
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 3);

        //First child: opengripper - fails due to nonexistent action being called, no direct recovery
        Context opengripperContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(opengripperContext.getStatus(), ActionStatus.FAIL_NOTFOUND);
        Assert.assertEquals(opengripperContext.getChildContexts().size(), 2);

        //Second child: recovery context - failure properly caught higher up tree in pickUp rather than opengripper
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryGoalContext.getCommand(), "goal(pickUpOpengripperActionRecovery(self:agent,[opengripper(self:agent),[found(self:agent,fakeAction(self:agent)),pickUp(self:agent)))");
        Assert.assertEquals(recoveryGoalContext.getStatus(), ActionStatus.SUCCESS);
        Context recoveryActionContext = recoveryGoalContext.getChildContexts().get(0);
        Assert.assertEquals(recoveryActionContext.getStatus(), ActionStatus.SUCCESS);

        //Third child: next step of pickUp - postRecovery execution defined as postrecovery_success
        Context pickupContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickupContext.getCommand(), "pickup");
    }

    /**
     * Tests action recovery where no explicit catchingAction is defined (only failure information)
     * Also implicitly tests postRecoveryExecution RETRY
     */
    @Test
    public void actionRecoveryCatchUndefinedChildTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl");
        String filename = aslPath + "actionRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[actionRecoveryCatchUndefinedChildTest]");
        //Submit pickUpFail goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("pickUpFail(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 1);

        Context pickupFailContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(pickupFailContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickupFailContext.getCommand(), "pickUpFail");
        Assert.assertEquals(pickupFailContext.getChildContexts().size(), 5);

        //First child: closegripper

        //Second child: pickup fails due to gripperOpen precondition
        Context pickupContext = pickupFailContext.getChildContexts().get(1);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Third child: recovery context (gripperOpen success effect)
        Context recoveryGoalContext = pickupFailContext.getChildContexts().get(2);
        Assert.assertEquals(recoveryGoalContext.getCommand(), "goal(pickupActionRecoveryUndefinedCatching(self:agent,[pickup(self:agent),[gripperOpen(self:agent),pickUpFail(self:agent)))");
        Assert.assertEquals(recoveryGoalContext.getStatus(), ActionStatus.SUCCESS);

        //PostRecovery execution defined as RETRY - re-execute failed pickup context
        //Fourth child: retry of pickup, now successful
        Context pickupRetryContext = pickupFailContext.getChildContexts().get(3);
        Assert.assertEquals(pickupRetryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickupRetryContext.getCommand(), "pickup");

        //Fifth child: continue execution of original script, log statement
    }

    /**
     * Test action recovery postExecutionRecovery PARENT_RETRY behavior
     */
    @Test
    public void actionRecoveryParentRetryTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "actionRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[actionRecoveryParentRetryTest]");
        //Submit pickUp goal which will trigger the correct recovery policy, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("pickUpFailV2(self:agent,parentretry)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 1);

        //pickUpFail context should have 3 children
        Context pickupFailContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(pickupFailContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickupFailContext.getCommand(), "pickUpFailV2");
        Assert.assertEquals(pickupFailContext.getChildContexts().size(), 3);

        //First child: successful pickUpV2 execution
        Context pickUpContext = pickupFailContext.getChildContexts().get(0);
        Assert.assertEquals(pickUpContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 3);

        //Second child: recovered pickUpV2 execution
        pickUpContext = pickupFailContext.getChildContexts().get(1);
        Assert.assertEquals(pickUpContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 6);

        //First inner child: log statement

        //Second inner child: Fail opengripper due to holding precondition
        Context openGripperContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(openGripperContext.getCommand(), "openGripperV2");
        Assert.assertEquals(openGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Third inner child: recovery policy - retracts holding predicate (i.e. held item is put down somewhere)
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(recoveryGoalContext.getCommand(), "goal(pickupParentRetryActionRecovery(self:agent,[openGripperV2(self:agent),[not(holdingV2(self:agent)),pickUpFailV2(self:agent,parentretry)))");
        Assert.assertEquals(recoveryGoalContext.getStatus(), ActionStatus.SUCCESS);

        //postRecoveryExecution is PARENT_RETRY -> means openGripperV2 failed, but we will re-execute parent context (pickUpV2)

        //Fourth inner child: initial log statement again (first step of pickUpV2)

        //Fifth inner child: opengripper re-attempt
        Context openGripperRetryContext = pickUpContext.getChildContexts().get(4);
        Assert.assertEquals(openGripperRetryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(openGripperRetryContext.getCommand(), "openGripperV2");

        //Sixth inner child: pickup we never initially reached
        Context pickupContext = pickUpContext.getChildContexts().get(5);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickupContext.getCommand(), "pickupv2");

        //Third child: success log statement
        Assert.assertEquals(pickupFailContext.getChildContexts().get(2).getCommand(), "log");
    }

    /**
     * Tests action recovery postExecutionRecovery PARENT_SUCCESS behavior
     */
    @Test
    public void actionRecoveryParentSuccessTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "actionRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[actionRecoveryParentSuccessTest]");
        //Submit pickUp goal which will trigger the correct recovery policy, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("pickUpFailV2(self:agent,parentsuccess)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 1);

        //pickUpFail context should have 3 children
        Context pickupFailContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(pickupFailContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickupFailContext.getCommand(), "pickUpFailV2");
        Assert.assertEquals(pickupFailContext.getChildContexts().size(), 3);

        //First child: first pickupv2 call, should succeed directly
        Context pickupContext = pickupFailContext.getChildContexts().get(0);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickupContext.getChildContexts().size(), 3);

        //Second child: second pickupv2 call, should succeed after recovery
        pickupContext = pickupFailContext.getChildContexts().get(1);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(pickupContext.getChildContexts().size(), 3);

        //First inner child: log statement

        //Second inner child: opengripper fails due to holding precondition
        Context openGripperContext = pickupContext.getChildContexts().get(1);
        Assert.assertEquals(openGripperContext.getCommand(), "openGripperV2");
        Assert.assertEquals(openGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Third inner child: recovery policy
        Context recoveryGoalContext = pickupContext.getChildContexts().get(2);
        Assert.assertEquals(recoveryGoalContext.getCommand(), "goal(pickupParentSuccessActionRecovery(self:agent,[openGripperV2(self:agent),[not(holdingV2(self:agent)),pickUpFailV2(self:agent,parentsuccess)))");
        Assert.assertEquals(recoveryGoalContext.getStatus(), ActionStatus.SUCCESS);

        //postRecoveryExecution is PARENT_SUCCESS
        //Should have no more inner children, pickUpV2 should be considered terminated as a success

        //Third child: log statement of pickUpFailV2
    }

    /**
     * Tests cascading recovery - where recovery fails at a lower level before succeeding at a higher level
     * (opengripper action recovery fails, but holding(X) goal recovery succeeds)
     */
    @Test
    public void cascadingRecoveryTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "cascadingRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        //Submit holding goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("goal(self:agent,holding(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        goalPredicate = (Predicate) goalPredicate.get(1);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        //Goal context - should have 3 chlidren
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 3);

        //First child: initial planning attempt - fail due to nonexistent action in opengripper
        Context plannedContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(plannedContext.getStatus(), ActionStatus.FAIL_NOTFOUND);
        Assert.assertEquals(plannedContext.getCommand(), "planned");
        Assert.assertEquals(plannedContext.getChildContexts().size(), 1);

        //First inner child: failed opengripper context
        Context openGripperContext = plannedContext.getChildContexts().get(0);
        Assert.assertEquals(openGripperContext.getStatus(), ActionStatus.FAIL_NOTFOUND);
        Assert.assertEquals(openGripperContext.getCommand(), "opengripper");
        Assert.assertEquals(openGripperContext.getChildContexts().size(), 3);

        //First opengripper inner child: log

        //Second opengripper inner child: nonexistent action, failure context

        //Third opengripper inner child: action recovery context for opengripper (fails by naively calling opengripper again)
        Context failedRecoveryContext = openGripperContext.getChildContexts().get(2);
        Assert.assertEquals(failedRecoveryContext.getStatus(), ActionStatus.FAIL_RECOVERY);
        //Recovery context should have FAIL_RECOVERY status while child should have FAIL_NOTFOUND
        Assert.assertEquals(failedRecoveryContext.getCommand(), "goal(openGripperActionRecoveryFail(self:agent,[],[found(self:agent,fakeAction(self:agent)),holding(self:agent)))");
        Assert.assertEquals(failedRecoveryContext.getChildContexts().get(0).getStatus(), ActionStatus.FAIL_NOTFOUND);

        //Second child: Higher level pickup goal recovery
        Context goalRecoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals(goalRecoveryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(goalRecoveryContext.getCommand(), "goal(pickupPartialRecovery(self:agent,[opengripper(self:agent), openGripperActionRecoveryFail(self:agent,\"[]\",\"[found(self:agent,fakeAction(self:agent))]\",holding(self:agent)), opengripper(self:agent), planned(self:agent),[found(self:agent,fakeAction(self:agent)),holding(self:agent)))");

        //Third child: Recovery complete, replan
        plannedContext = rootContext.getChildContexts().get(2);
        Assert.assertEquals(plannedContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(plannedContext.getCommand(), "planned");
    }

    /**
     * Tests recovery case where recovery occurs for siblings of the same parent context
     */
    @Test
    public void multipleActionRecoveryTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "multipleActionRecoveryTest.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        //Submit pickup goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("pickUp(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();

        //pickUp context should have three children
        Context pickUpContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 3);
        Assert.assertEquals(pickUpContext.getStatus(), ActionStatus.SUCCESS);

        //First child: opengripper - recovers from FAIL_NOTFOUND
        Context opengripperContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(opengripperContext.getChildContexts().size(), 4);
        Context recoveryContext = opengripperContext.getChildContexts().get(2);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(openGripperActionRecovery(self:agent,[],[found(self:agent,fakeAction(self:agent)),pickUp(self:agent)))");

        //Second child: pickup - recovers from FAIL_NOTFOUND
        Context pickupContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(pickupContext.getChildContexts().size(), 3);
        recoveryContext = pickupContext.getChildContexts().get(2);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(pickupActionRecovery(self:agent,[],[found(self:agent,fakeAction(self:agent)),pickUp(self:agent)))");

        //Third child: success log statement
    }

    /**
     * Tests successful nested recovery (recovery of recovery).
     */
    @Test
    public void nestedRecoverySuccessTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "nestedRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        //Submit pickUp goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("pickUp(self:agent,success)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();

        //pickup context, 3 children
        Context pickUpContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 3);
        Assert.assertEquals(pickUpContext.getStatus(), ActionStatus.SUCCESS);

        //(Tree looks this way because of catchingAction and postRecovery definitions)
        //First child: fail pickup because of gripperOpen precondition
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(pickupContext.getChildContexts().size(), 0);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Second child: recovery for failed pickup (asserts gripperOpen) Should succeed after own recovery
        Context recoveryContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryContext.getChildContexts().size(), 1);
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);
        Context recoveryActionContext = recoveryContext.getChildContexts().get(0);
        Assert.assertEquals(recoveryActionContext.getChildContexts().size(), 4);
        Assert.assertEquals(recoveryActionContext.getStatus(), ActionStatus.SUCCESS);

        //First inner child: recovery fails itself due to preconditions of called action
        Context failActionContext = recoveryActionContext.getChildContexts().get(0);
        Assert.assertEquals(failActionContext.getCommand(), "failAction");
        Assert.assertEquals(failActionContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Second inner child: recovery of recovery policy - succeeds with postrecovery_retry
        recoveryContext = recoveryActionContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(failActionRecovery(self:agent,[failAction(self:agent),[failActionPre(self:agent),pickupNestedRecoverySuccess(self:agent,[pickup(self:agent),[gripperOpen(self:agent),pickUp(self:agent,success))))");
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);

        //Third inner child: failed action re-executed successfully
        failActionContext = recoveryActionContext.getChildContexts().get(2);
        Assert.assertEquals(failActionContext.getCommand(), "failAction");
        Assert.assertEquals(failActionContext.getStatus(), ActionStatus.SUCCESS);

        //Fourth inner child: success log

        //Third child: Successful pickup post recovery
        pickupContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.SUCCESS);
    }

    /**
     * Tests whether nested recovery is correctly prevented when the same failure reasons are encountered in the recovery
     * policy as the ones that caused it to trigger in the first place
     */
    @Test
    public void nestedRecoveryFailureTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "nestedRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        //Submit pickUp goal that will fail after recovery terminates itself
        Predicate goalPredicate = Factory.createPredicate("pickUp(self:agent,fail)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        //PickUp should have two children and fail
        Context rootContext = goal.getRootContext();
        Context pickUpContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(pickUpContext.getCommand(), "pickUp");
        Assert.assertEquals(pickUpContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 2);

        //First child: failed pickup context - on gripperOpen precondition
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(pickupContext.getCommand(), "pickup");
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Second child: Failed recovery context
        Context recoveryContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(pickupNestedRecoveryFail(self:agent,[pickup(self:agent),[gripperOpen(self:agent),pickUp(self:agent,fail)))");
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.FAIL_RECOVERY);
        Assert.assertEquals(recoveryContext.getChildContexts().size(), 1);
        recoveryContext = recoveryContext.getChildContexts().get(0);
        Assert.assertEquals(recoveryContext.getChildContexts().size(), 1);

        //First inner child: failed pickup context - recovery policy simply called the failed action again without
        // changing anything. FAIL_PRECONDITIONS with the same justification. A valid recovery policy exists but it
        // should not be applied (because it is the same one)
        pickupContext = recoveryContext.getChildContexts().get(0);
        Assert.assertEquals(pickupContext.getChildContexts().size(), 0);
        Assert.assertEquals(pickupContext.getCommand(), "pickup");
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //nested recovery prevented, no more execution
    }


    //Q: Are incorrect recovery policy definitions something we want to explicitly detect and protect against programatically?
    //  In general, I'm sure other scenarios where this can occur exist. How much of this is developer misuse/bad policy implementation vs unsafe recovery implementation?
    /**
     * Tests that poorly defined recovery policies don't break execution. In this case, if an action recovery policy has
     * 'retry' postrecovery execution defined but doesn't actually do anything to help solve the failure (would normally
     * lead to an infinite recovery loop)
     */
    @Test
    public void faultyRetryRecoveryDefinitionTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "nestedRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        Predicate goalPredicate = Factory.createPredicate("pickUpV2(self:agent,child)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();

        //Action should fail after recovery is halted after first duplicate failure/recovery detection
        Context pickUpContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(pickUpContext.getCommand(), "pickUpV2");
        Assert.assertEquals(pickUpContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 4);

        Context logContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(logContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(logContext.getCommand(), "log");

        //First pickup failure
        Context pickupContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickupContext.getCommand(), "pickupv2");

        //Recovery with postrecovery_retry
        Context recoveryContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(pickupNestedRecoveryFaulty(self:agent,[pickupv2(self:agent),[gripperOpenV2(self:agent),pickUpV2(self:agent,child)))");

        //Reattempted pickup, identical failure
        pickupContext = pickUpContext.getChildContexts().get(3);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickupContext.getCommand(), "pickupv2");

        //Fail rather than enter duplicate recovery
    }
    /**
     * Tests that poorly defined recovery policies don't break execution. In this case, if an action recovery policy has
     * 'parent_retry' postrecovery execution defined but doesn't actually do anything to help solve the failure (would
     * normally lead to an infinite recovery loop)
     */
    @Test
    public void faultyParentRetryRecoveryDefinitionTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "nestedRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        Predicate goalPredicate = Factory.createPredicate("pickUpV2(self:agent,parent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();

        //Action should fail after recovery is halted after first duplicate failure/recovery detection
        Context pickUpContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(pickUpContext.getCommand(), "pickUpV2");
        Assert.assertEquals(pickUpContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 5);

        //First step: log
        Context logContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(logContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(logContext.getCommand(), "log");

        //Second step: first pickup failure
        Context pickupContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickupContext.getCommand(), "pickupv2");

        //Third step: empty recovery with postrecovery_parent_retry
        Context recoveryContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(pickupNestedParentRecoveryFaulty(self:agent,[pickupv2(self:agent),[gripperOpenV2(self:agent),pickUpV2(self:agent,parent)))");

        //Fourth step: first pickUp step again, log
        logContext = pickUpContext.getChildContexts().get(3);
        Assert.assertEquals(logContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(logContext.getCommand(), "log");

        //Fifth step: second pickUp step again, pickup failure
        pickupContext = pickUpContext.getChildContexts().get(4);
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(pickupContext.getCommand(), "pickupv2");

        //Fail rather than enter duplicate recovery
    }

    /**
     * Tests that when goal recovery 'successfully' completes but doesn't actually solve the issue, the system
     * prevents recovery from occurring on the same goal due to the same failure.
     */
    @Test
    public void ineffectiveGoalRecoveryTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "goalRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");


        log.info("\n\n\n[ineffectiveGoalRecoveryTest]");
        //Submit goal for pickUp - recovery should cause goal to succeed
        Predicate goalPredicate = Factory.createPredicate("pickUp(self:agent,ineffective)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        //PickUp context - 3 children
        Context pickUpContext = goal.getRootContext();
        Assert.assertEquals(pickUpContext.getChildContexts().size(), 3);

        //First child: First execution attempt - should fail due to gripperOpen precondition not being satisfied
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(pickupContext.getCommand(), "pickUp");
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Second child: Recovery context (doesn't practically do anything)
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryGoalContext.getCommand(), "goal(pickupIneffectiveRecovery(self:agent,[pickup(self:agent), pickUp(self:agent,ineffective),[gripperOpen(self:agent),pickUp(self:agent,ineffective)))");
        Assert.assertEquals(recoveryGoalContext.getStatus(), ActionStatus.SUCCESS);

        //Third child: Second execution attempt - should fail in the same way as the first
        pickupContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(pickupContext.getCommand(), "pickUp");
        Assert.assertEquals(pickupContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //No more execution should occur
    }

    /**
     * Tests action recovery where there is no parent action context to catch the failure. This is a special case
     *  where the failure needs to be caught in the root goal context and apply action recovery
     */
    @Test
    public void actionRecoveryRetryFromRootGoalTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.UtilitarianActionSelector");
        String filename = aslPath + "shouldAttemptRecoveryTest.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[actionRecoveryRetryFromRootGoalTest]");
        //Submit pickUpFail goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("testActionRetry(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 3);

        //First attempt: fail
        Context testActionContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(testActionContext.getCommand(),"testActionRetry");
        Assert.assertEquals(testActionContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Action recovery applied under goal context
        Context recoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(testActionRetryRecovery(self:agent,[testActionRetry(self:agent),[testActionPre(self:agent),testActionRetry(self:agent)))");
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);

        //Retry attempt: success
        testActionContext = rootContext.getChildContexts().get(2);
        Assert.assertEquals(testActionContext.getCommand(),"testActionRetry");
        Assert.assertEquals(testActionContext.getStatus(), ActionStatus.SUCCESS);
    }

    /**
     * Tests action recovery where there is no parent action context to catch the failure. This is a special case
     *  where the failure needs to be caught in the root goal context and apply action recovery
     */
    @Test
    public void actionRecoverySuccessFromRootGoalTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.UtilitarianActionSelector");
        String filename = aslPath + "shouldAttemptRecoveryTest.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[actionRecoverySuccessFromRootGoalTest]");
        //Submit pickUpFail goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("testActionSuccess(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 2);

        //First attempt: fail
        Context testActionContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(testActionContext.getCommand(),"testActionSuccess");
        Assert.assertEquals(testActionContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Action recovery applied under goal context
        Context recoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(testActionSuccessRecovery(self:agent,[testActionSuccess(self:agent),[testActionPre(self:agent),testActionSuccess(self:agent)))");
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);

        //postrecovery_success -> no more execution
    }

    /**
     * Tests that recovery does not abort execution when encountering the same failure on different steps of an action
     * (i.e. does not detect separate first attempt action failures as duplicate failures/failed recovery)
     */
    @Test
    public void sameFailureDifferentStepTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "multipleActionRecoveryTest.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[sameFailureDifferentStepTest]");
        //Submit closeGripperTwice goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("closeGripperThrice(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 1);

        Context closeGripperThriceContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(closeGripperThriceContext.getChildContexts().size(), 9);
        Assert.assertEquals(closeGripperThriceContext.getStatus(), ActionStatus.SUCCESS);

        //First attempt: failure because gripper is not open
        Context closeGripperContext = closeGripperThriceContext.getChildContexts().get(0);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //First attempt recovery: sets precondition
        Context recoveryContext = closeGripperThriceContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(closeGripperActionRecovery(self:agent,[closeGripper(self:agent),[gripperOpen(self:agent),closeGripperThrice(self:agent)))");
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);

        //Second attempt: success
        closeGripperContext = closeGripperThriceContext.getChildContexts().get(2);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.SUCCESS);

        //Same sequence for second closeGripper
        //First attempt: failure because gripper is not open
        closeGripperContext = closeGripperThriceContext.getChildContexts().get(3);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //First attempt recovery: sets precondition
        recoveryContext = closeGripperThriceContext.getChildContexts().get(4);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(closeGripperActionRecovery(self:agent,[closeGripper(self:agent),[gripperOpen(self:agent),closeGripperThrice(self:agent)))");
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);

        //Second attempt: success
        closeGripperContext = closeGripperThriceContext.getChildContexts().get(5);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.SUCCESS);

        //Same sequence for third closeGripper
        //First attempt: failure because gripper is not open
        closeGripperContext = closeGripperThriceContext.getChildContexts().get(6);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //First attempt recovery: sets precondition
        recoveryContext = closeGripperThriceContext.getChildContexts().get(7);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(closeGripperActionRecovery(self:agent,[closeGripper(self:agent),[gripperOpen(self:agent),closeGripperThrice(self:agent)))");
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);

        //Second attempt: success
        closeGripperContext = closeGripperThriceContext.getChildContexts().get(8);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.SUCCESS);
    }

    /**
     * Tests that recovery does not abort execution when encountering the same failure on different steps of an action
     * (i.e. does not detect separate first attempt action failures as duplicate failures/failed recovery)
     * This tests this case when under the context of execution from parent_retry postrecovery
     */
    @Test
    public void sameFailureDifferentStepParentRetryTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "multipleActionRecoveryTest.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[sameFailureDifferentStepParentRetryTest]");
        //Submit closeGripperTwice goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("testActionSequence(self:agent,once)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 1);

        Context testActionSequenceContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(testActionSequenceContext.getChildContexts().size(), 15);
        Assert.assertEquals(testActionSequenceContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //First attempt trace, fail on first closeGripper
        Context logContext = testActionSequenceContext.getChildContexts().get(0);
        Assert.assertEquals(logContext.getCommand(), "log");

        logContext = testActionSequenceContext.getChildContexts().get(1);
        Assert.assertEquals(logContext.getCommand(), "log");

        Context closeGripperContext = testActionSequenceContext.getChildContexts().get(2);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Recovery
        Context recoveryContext = testActionSequenceContext.getChildContexts().get(3);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(testActionSequenceActionRecovery(self:agent,[closeGripper(self:agent),[gripperOpen(self:agent),testActionSequence(self:agent,once)))");

        //Second attempt trace, fail on second closeGripper
        logContext = testActionSequenceContext.getChildContexts().get(4);
        Assert.assertEquals(logContext.getCommand(), "log");

        logContext = testActionSequenceContext.getChildContexts().get(5);
        Assert.assertEquals(logContext.getCommand(), "log");

        closeGripperContext = testActionSequenceContext.getChildContexts().get(6);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.SUCCESS);

        logContext = testActionSequenceContext.getChildContexts().get(7);
        Assert.assertEquals(logContext.getCommand(), "log");

        closeGripperContext = testActionSequenceContext.getChildContexts().get(8);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        recoveryContext = testActionSequenceContext.getChildContexts().get(9);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(testActionSequenceActionRecovery(self:agent,[closeGripper(self:agent),[gripperOpen(self:agent),testActionSequence(self:agent,once)))");

        //Third attempt trace: fail on second closeGripper again
        logContext = testActionSequenceContext.getChildContexts().get(10);
        Assert.assertEquals(logContext.getCommand(), "log");

        logContext = testActionSequenceContext.getChildContexts().get(11);
        Assert.assertEquals(logContext.getCommand(), "log");

        closeGripperContext = testActionSequenceContext.getChildContexts().get(12);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.SUCCESS);

        logContext = testActionSequenceContext.getChildContexts().get(13);
        Assert.assertEquals(logContext.getCommand(), "log");

        closeGripperContext = testActionSequenceContext.getChildContexts().get(14);
        Assert.assertEquals(closeGripperContext.getCommand(), "closeGripper");
        Assert.assertEquals(closeGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        //Duplicate failure, exit recovery and fail top level goal
    }

    /**
     * Same test as above, but with maximumAttempts = 2. Should allow one more sequence of parent_retry attempts
     */
    @Test
    public void sameFailureDifferentStepParentRetryTwoAttemptsTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "multipleActionRecoveryTest.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[sameFailureDifferentStepParentRetryTwoAttemptsTest]");
        //Submit closeGripperTwice goal, should succeed due to recovery
        Predicate goalPredicate = Factory.createPredicate("testActionSequence(self:agent,twice)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 1);

        //Not writing out entire tree, should be same as the test above but with an extra attempt failing after 6 more steps
        Context testActionSequenceContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(testActionSequenceContext.getChildContexts().size(), 21);
        Assert.assertEquals(testActionSequenceContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
    }

    @Test
    public void overridePostRecoveryExecutionTypeTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.UtilitarianActionSelector");
        String filename = aslPath + "actionRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[overridePostRecoveryExecutionTypeTest]");
        Predicate goalPredicate = Factory.createPredicate("pickUpFailV2(self:agent,override)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
    }

    /**
     * Tests definition and application of 'maximumAttempts' for the action recovery retry case
     */
    @Test
    public void multipleRecoveryAttemptsTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.UtilitarianActionSelector");
        String filename = aslPath + "actionRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[multipleRecoveryAttemptsTest]");
        Predicate goalPredicate = Factory.createPredicate("pickUpFailV2(self:agent,multiple)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 1);

        //Recovery policy is defined with maximumAttempts of 3, should be applied 3 and result in a total of 4 action failures
        Context pickUpFailV2Context = rootContext.getChildContexts().get(0);
        Context pickUpV2Context = pickUpFailV2Context.getChildContexts().get(1);
        Assert.assertEquals(pickUpV2Context.getChildContexts().size(), 8);
        Assert.assertEquals(pickUpV2Context.getStatus(), ActionStatus.FAIL_PRECONDITIONS);

        Context logContext = pickUpV2Context.getChildContexts().get(0);
        Assert.assertEquals(logContext.getCommand(), "log");

        // First attempt failure/recovery
        Context openGripperContext = pickUpV2Context.getChildContexts().get(1);
        Assert.assertEquals(openGripperContext.getCommand(), "openGripperV2");
        Assert.assertEquals(openGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Context recoveryContext = pickUpV2Context.getChildContexts().get(2);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(multipleAttemptsRecoveryPolicy(self:agent,[openGripperV2(self:agent),[not(holdingV2(self:agent)),pickUpFailV2(self:agent,multiple)))");

        // Second attempt failure/recovery
        openGripperContext = pickUpV2Context.getChildContexts().get(3);
        Assert.assertEquals(openGripperContext.getCommand(), "openGripperV2");
        Assert.assertEquals(openGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        recoveryContext = pickUpV2Context.getChildContexts().get(4);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(multipleAttemptsRecoveryPolicy(self:agent,[openGripperV2(self:agent),[not(holdingV2(self:agent)),pickUpFailV2(self:agent,multiple)))");

        // Third attempt failure/recovery
        openGripperContext = pickUpV2Context.getChildContexts().get(5);
        Assert.assertEquals(openGripperContext.getCommand(), "openGripperV2");
        Assert.assertEquals(openGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        recoveryContext = pickUpV2Context.getChildContexts().get(6);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(multipleAttemptsRecoveryPolicy(self:agent,[openGripperV2(self:agent),[not(holdingV2(self:agent)),pickUpFailV2(self:agent,multiple)))");

        //Fourth failure, no more recovery
        openGripperContext = pickUpV2Context.getChildContexts().get(7);
        Assert.assertEquals(openGripperContext.getCommand(), "openGripperV2");
        Assert.assertEquals(openGripperContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
    }

    /**
     * Tests that action recovery RETRY behavior correctly handles cyclical failures.
     * Recovery should fail appropriately rather than loop infinitely.
     */
    @Test
    public void cyclicalActionRecoveryRetryTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.UtilitarianActionSelector");
        String filename = aslPath + "actionRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[cyclicalActionRecoveryRetryTest]");
        Predicate goalPredicate = Factory.createPredicate("cyclicalActionRecoveryTestAction(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 5);

        //First attempt: failed due to precondition 1
        Context actionContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(actionContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(actionContext.getJustification().getFailureReason().get(0), Factory.createPredicate("not(gripperOpen(self:agent))"));

        //Recovery policy 1: set precondition 1
        Context recoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(cyclicalActionRecoveryTestActionRecoveryPolicy(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[gripperOpen(self:agent),cyclicalActionRecoveryTestAction(self:agent)))");

        //Second attempt: failed due to precondition 2
        actionContext = rootContext.getChildContexts().get(2);
        Assert.assertEquals(actionContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(actionContext.getJustification().getFailureReason().get(0), Factory.createPredicate("not(fillerPredicate(self:agent))"));

        //Recovery policy 2: set precondition 2, unset precondition 1
        recoveryContext = rootContext.getChildContexts().get(3);
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(cyclicalActionRecoveryTestActionRecoveryPolicyTwo(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[fillerPredicate(self:agent),cyclicalActionRecoveryTestAction(self:agent)))");

        //Third attempt: failed due to precondition 1 -> reach max_attempts for policy and exit recovery
        actionContext = rootContext.getChildContexts().get(4);
        Assert.assertEquals(actionContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(actionContext.getJustification().getFailureReason().get(0), Factory.createPredicate("not(gripperOpen(self:agent))"));
    }

    /**
     * Tests that action recovery RETRY behavior correctly handles cyclical failures.
     * Recovery attempts should properly be cleaned up and not persist to later steps
     */
    @Test
    public void cyclicalActionRecoveryRetryTestV2() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.UtilitarianActionSelector");
        String filename = aslPath + "actionRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[cyclicalActionRecoveryRetryTestV2]");
        Predicate goalPredicate = Factory.createPredicate("executeCyclicalTestActionTwice(self:agent)");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(rootContext.getChildContexts().size(), 1);

        Context executeActionTwiceContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(executeActionTwiceContext.getChildContexts().size(), 12);

        Context logContext = executeActionTwiceContext.getChildContexts().get(0);
        Assert.assertEquals(logContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(logContext.getCommand(), "log");

        //Same sequence as test above, except second recovery policy doesn't unset precondition 1
        Context actionContext = executeActionTwiceContext.getChildContexts().get(1);
        Assert.assertEquals(actionContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(actionContext.getCommand(), "cyclicalActionRecoveryTestAction");
        Context recoveryContext = executeActionTwiceContext.getChildContexts().get(2);
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(cyclicalActionRecoveryTestActionRecoveryPolicy(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[gripperOpen(self:agent),executeCyclicalTestActionTwice(self:agent)))");
        actionContext = executeActionTwiceContext.getChildContexts().get(3);
        Assert.assertEquals(actionContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(actionContext.getCommand(), "cyclicalActionRecoveryTestAction");
        recoveryContext = executeActionTwiceContext.getChildContexts().get(4);
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(cyclicalActionRecoveryTestActionRecoveryPolicyTwoSucceed(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[fillerPredicate(self:agent),executeCyclicalTestActionTwice(self:agent)))");
        actionContext = executeActionTwiceContext.getChildContexts().get(5);
        Assert.assertEquals(actionContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(actionContext.getCommand(), "cyclicalActionRecoveryTestAction");

        logContext = executeActionTwiceContext.getChildContexts().get(6);
        Assert.assertEquals(logContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(logContext.getCommand(), "log");

        //Same sequence for second instance of same action. Policy application count should properly be cleared such
        //  that this is allowed to successfully recovery rather than exit due to max_attempts
        actionContext = executeActionTwiceContext.getChildContexts().get(7);
        Assert.assertEquals(actionContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(actionContext.getCommand(), "cyclicalActionRecoveryTestAction");
        recoveryContext = executeActionTwiceContext.getChildContexts().get(8);
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(cyclicalActionRecoveryTestActionRecoveryPolicy(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[gripperOpen(self:agent),executeCyclicalTestActionTwice(self:agent)))");
        actionContext = executeActionTwiceContext.getChildContexts().get(9);
        Assert.assertEquals(actionContext.getStatus(), ActionStatus.FAIL_PRECONDITIONS);
        Assert.assertEquals(actionContext.getCommand(), "cyclicalActionRecoveryTestAction");
        recoveryContext = executeActionTwiceContext.getChildContexts().get(10);
        Assert.assertEquals(recoveryContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(recoveryContext.getCommand(), "goal(cyclicalActionRecoveryTestActionRecoveryPolicyTwoSucceed(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[fillerPredicate(self:agent),executeCyclicalTestActionTwice(self:agent)))");
        actionContext = executeActionTwiceContext.getChildContexts().get(11);
        Assert.assertEquals(actionContext.getStatus(), ActionStatus.SUCCESS);
        Assert.assertEquals(actionContext.getCommand(), "cyclicalActionRecoveryTestAction");
    }
}
