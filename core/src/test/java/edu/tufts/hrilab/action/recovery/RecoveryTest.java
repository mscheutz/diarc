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
            Assert.assertEquals(recoveryActionName, recoveryActionContext.getCommand());
            Assert.assertEquals(desiredStatus, recoveryActionContext.getStatus());
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
        Assert.assertEquals(2, pickUpContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.SUCCESS, pickUpContext.getStatus());

        //First child: goal fails because we cannot plan
        Context failureContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.FAIL_NOTFOUND, failureContext.getStatus());

        //Second child: recovery context (Recovery policy has holding(self:agent) as an effect)
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals("goal(pickupFullRecovery(self:agent,[],[found(self:agent,holding(self:agent)),holding(self:agent)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryGoalContext.getStatus());

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
        Assert.assertEquals(3, pickUpContext.getChildContexts().size());

        //First child: First execution attempt - should fail due to gripperOpen precondition not being satisfied
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals("pickUp", pickupContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());

        //Second child: Recovery context (policy has gripperOpen(self:agent) as an effect)
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals("goal(pickupPartialRecovery(self:agent,[pickup(self:agent), pickUp(self:agent,success),[gripperOpen(self:agent),pickUp(self:agent,success)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryGoalContext.getStatus());

        //Third child: Second execution attempt - should now succeed
        pickupContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals("pickUp", pickupContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, pickupContext.getStatus());
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
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickUpContext.getStatus());
        Assert.assertEquals(pickUpContext.getJustification(),new AndJustification(new ConditionJustification(false, Factory.createPredicate("gripperOpen(self:agent)"))));
        //Should only have 2 children - no more execution after failed recovery
        Assert.assertEquals(2, pickUpContext.getChildContexts().size());

        //First child: failed pickup with original failure reasons
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());
        Assert.assertEquals(pickupContext.getJustification(),new AndJustification(new ConditionJustification(false, Factory.createPredicate("gripperOpen(self:agent)"))));

        //Second child: Recovery context itself should fail with FAIL_RECOVERY status
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals("goal(pickupFailRecovery(self:agent,[pickup(self:agent), pickUp(self:agent,failure),[gripperOpen(self:agent),pickUp(self:agent,failure)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_RECOVERY, recoveryGoalContext.getStatus());

        //Child of recovery context should have had actual failure reasons of the recovery policy execution propagated up
        Context recoveryActionContext = recoveryGoalContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.FAIL_NOTFOUND, recoveryActionContext.getStatus());
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
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickUpContext.getStatus());
        Assert.assertEquals(pickUpContext.getJustification(),new AndJustification(new ConditionJustification(false, Factory.createPredicate("gripperOpen(self:agent)"))));
        //Should have 2 children, no execution after exiting recovery
        Assert.assertEquals(2, pickUpContext.getChildContexts().size());

        //First child: failed pickup with original failure reasons
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());
        Assert.assertEquals(pickupContext.getJustification(),new AndJustification(new ConditionJustification(false, Factory.createPredicate("gripperOpen(self:agent)"))));

        //Recovery context itself should fail with exit_recovery status
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals("goal(pickupExitRecovery(self:agent,[pickup(self:agent), pickUp(self:agent,exit),[gripperOpen(self:agent),pickUp(self:agent,exit)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.EXIT_RECOVERY, recoveryGoalContext.getStatus());

        //Recovery child should fail with exit_recovery
        Context recoveryActionContext = recoveryGoalContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.EXIT_RECOVERY, recoveryActionContext.getStatus());
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
        Assert.assertEquals(1, goalContext.getChildContexts().size());
        //Plan generated for goal
        Context plannedContext = goalContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, plannedContext.getStatus());
        Assert.assertEquals("planned", plannedContext.getCommand());
        Assert.assertEquals(2, plannedContext.getChildContexts().size());

        //First step in plan, opengripper. Should have success status after successful recovery
        Context opengripperContext = plannedContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, opengripperContext.getStatus());
        //Should have four children
        Assert.assertEquals(4, opengripperContext.getChildContexts().size());

        //Child one: log statement

        //Child two: Failed nonexistent action
        Context failnotfoundContext = opengripperContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.FAIL_NOTFOUND, failnotfoundContext.getStatus());

        //Child three: Recovery context
        Context recoveryGoalContext = opengripperContext.getChildContexts().get(2);
        Assert.assertEquals("goal(openGripperActionRecovery(self:agent,[],[found(self:agent,fakeAction(self:agent)),holding(self:agent)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryGoalContext.getStatus());
        Context recoveryActionContext = recoveryGoalContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryActionContext.getStatus());

        //postrecovery execution defined as postrecovery_success -> pickup continues from step after failed action
        //Child four: log statement
        Context logContext = opengripperContext.getChildContexts().get(3);
        Assert.assertEquals(ActionStatus.SUCCESS, logContext.getStatus());

        //Second step in plan executed
        Context pickupContext = plannedContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.SUCCESS, pickupContext.getStatus());

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
        Assert.assertEquals(1, rootContext.getChildContexts().size());
        Context pickUpContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, pickUpContext.getStatus());
        Assert.assertEquals("pickUp", pickUpContext.getCommand());
        Assert.assertEquals(3, pickUpContext.getChildContexts().size());

        //First child: opengripper - fails due to nonexistent action being called, no direct recovery
        Context opengripperContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.FAIL_NOTFOUND, opengripperContext.getStatus());
        Assert.assertEquals(2, opengripperContext.getChildContexts().size());

        //Second child: recovery context - failure properly caught higher up tree in pickUp rather than opengripper
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals("goal(pickUpOpengripperActionRecovery(self:agent,[opengripper(self:agent),[found(self:agent,fakeAction(self:agent)),pickUp(self:agent)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryGoalContext.getStatus());
        Context recoveryActionContext = recoveryGoalContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryActionContext.getStatus());

        //Third child: next step of pickUp - postRecovery execution defined as postrecovery_success
        Context pickupContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(ActionStatus.SUCCESS, pickupContext.getStatus());
        Assert.assertEquals("pickup", pickupContext.getCommand());
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
        Assert.assertEquals(1, rootContext.getChildContexts().size());

        Context pickupFailContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, pickupFailContext.getStatus());
        Assert.assertEquals("pickUpFail", pickupFailContext.getCommand());
        Assert.assertEquals(5, pickupFailContext.getChildContexts().size());

        //First child: closegripper

        //Second child: pickup fails due to gripperOpen precondition
        Context pickupContext = pickupFailContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());

        //Third child: recovery context (gripperOpen success effect)
        Context recoveryGoalContext = pickupFailContext.getChildContexts().get(2);
        Assert.assertEquals("goal(pickupActionRecoveryUndefinedCatching(self:agent,[pickup(self:agent),[gripperOpen(self:agent),pickUpFail(self:agent)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryGoalContext.getStatus());

        //PostRecovery execution defined as RETRY - re-execute failed pickup context
        //Fourth child: retry of pickup, now successful
        Context pickupRetryContext = pickupFailContext.getChildContexts().get(3);
        Assert.assertEquals(ActionStatus.SUCCESS, pickupRetryContext.getStatus());
        Assert.assertEquals("pickup", pickupRetryContext.getCommand());

        //Fifth child: continue execution of original script, log statement
    }

    /**
     * Test action recovery postExecutionRecovery PARENT_RETRY behavior
     */
    @Ignore
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
        Assert.assertEquals(1, rootContext.getChildContexts().size());

        //pickUpFail context should have 3 children
        Context pickupFailContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, pickupFailContext.getStatus());
        Assert.assertEquals("pickUpFailV2", pickupFailContext.getCommand());
        Assert.assertEquals(3, pickupFailContext.getChildContexts().size());

        //First child: successful pickUpV2 execution
        Context pickUpContext = pickupFailContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, pickUpContext.getStatus());
        Assert.assertEquals(3, pickUpContext.getChildContexts().size());

        //Second child: recovered pickUpV2 execution
        pickUpContext = pickupFailContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.SUCCESS, pickUpContext.getStatus());
        Assert.assertEquals(6, pickUpContext.getChildContexts().size());

        //First inner child: log statement

        //Second inner child: Fail opengripper due to holding precondition
        Context openGripperContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals("openGripperV2", openGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, openGripperContext.getStatus());

        //Third inner child: recovery policy - retracts holding predicate (i.e. held item is put down somewhere)
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals("goal(pickupParentRetryActionRecovery(self:agent,[openGripperV2(self:agent),[not(holdingV2(self:agent)),pickUpFailV2(self:agent,parentretry)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryGoalContext.getStatus());

        //postRecoveryExecution is PARENT_RETRY -> means openGripperV2 failed, but we will re-execute parent context (pickUpV2)

        //Fourth inner child: initial log statement again (first step of pickUpV2)

        //Fifth inner child: opengripper re-attempt
        Context openGripperRetryContext = pickUpContext.getChildContexts().get(4);
        Assert.assertEquals(ActionStatus.SUCCESS, openGripperRetryContext.getStatus());
        Assert.assertEquals("openGripperV2", openGripperRetryContext.getCommand());

        //Sixth inner child: pickup we never initially reached
        Context pickupContext = pickUpContext.getChildContexts().get(5);
        Assert.assertEquals(ActionStatus.SUCCESS, pickupContext.getStatus());
        Assert.assertEquals("pickupv2", pickupContext.getCommand());

        //Third child: success log statement
        Assert.assertEquals("log", pickupFailContext.getChildContexts().get(2).getCommand());
    }

    /**
     * Tests action recovery postExecutionRecovery PARENT_SUCCESS behavior
     */
    @Ignore
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
        Assert.assertEquals(1, rootContext.getChildContexts().size());

        //pickUpFail context should have 3 children
        Context pickupFailContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, pickupFailContext.getStatus());
        Assert.assertEquals("pickUpFailV2", pickupFailContext.getCommand());
        Assert.assertEquals(3, pickupFailContext.getChildContexts().size());

        //First child: first pickupv2 call, should succeed directly
        Context pickupContext = pickupFailContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, pickupContext.getStatus());
        Assert.assertEquals(3, pickupContext.getChildContexts().size());

        //Second child: second pickupv2 call, should succeed after recovery
        pickupContext = pickupFailContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.SUCCESS, pickupContext.getStatus());
        Assert.assertEquals(3, pickupContext.getChildContexts().size());

        //First inner child: log statement

        //Second inner child: opengripper fails due to holding precondition
        Context openGripperContext = pickupContext.getChildContexts().get(1);
        Assert.assertEquals("openGripperV2", openGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, openGripperContext.getStatus());

        //Third inner child: recovery policy
        Context recoveryGoalContext = pickupContext.getChildContexts().get(2);
        Assert.assertEquals("goal(pickupParentSuccessActionRecovery(self:agent,[openGripperV2(self:agent),[not(holdingV2(self:agent)),pickUpFailV2(self:agent,parentsuccess)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryGoalContext.getStatus());

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
        Assert.assertEquals(3, rootContext.getChildContexts().size());

        //First child: initial planning attempt - fail due to nonexistent action in opengripper
        Context plannedContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.FAIL_NOTFOUND, plannedContext.getStatus());
        Assert.assertEquals("planned", plannedContext.getCommand());
        Assert.assertEquals(1, plannedContext.getChildContexts().size());

        //First inner child: failed opengripper context
        Context openGripperContext = plannedContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.FAIL_NOTFOUND, openGripperContext.getStatus());
        Assert.assertEquals("opengripper", openGripperContext.getCommand());
        Assert.assertEquals(3, openGripperContext.getChildContexts().size());

        //First opengripper inner child: log

        //Second opengripper inner child: nonexistent action, failure context

        //Third opengripper inner child: action recovery context for opengripper (fails by naively calling opengripper again)
        Context failedRecoveryContext = openGripperContext.getChildContexts().get(2);
        Assert.assertEquals(ActionStatus.FAIL_RECOVERY, failedRecoveryContext.getStatus());
        //Recovery context should have FAIL_RECOVERY status while child should have FAIL_NOTFOUND
        Assert.assertEquals("goal(openGripperActionRecoveryFail(self:agent,[],[found(self:agent,fakeAction(self:agent)),holding(self:agent)))", failedRecoveryContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_NOTFOUND, failedRecoveryContext.getChildContexts().get(0).getStatus());

        //Second child: Higher level pickup goal recovery
        Context goalRecoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.SUCCESS, goalRecoveryContext.getStatus());
        Assert.assertEquals("goal(pickupPartialRecovery(self:agent,[opengripper(self:agent), planned(self:agent),[found(self:agent,fakeAction(self:agent)),holding(self:agent)))", goalRecoveryContext.getCommand());

        //Third child: Recovery complete, replan
        plannedContext = rootContext.getChildContexts().get(2);
        Assert.assertEquals(ActionStatus.SUCCESS, plannedContext.getStatus());
        Assert.assertEquals("planned", plannedContext.getCommand());
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
        Assert.assertEquals(5, pickUpContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.SUCCESS, pickUpContext.getStatus());

        //First child: opengripper - FAIL_NOTFOUND
        Context opengripperContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(2, opengripperContext.getChildContexts().size());

        //Second child - opngripper recovery
        Context recoveryContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals("goal(openGripperActionRecovery(self:agent,[opengripper(self:agent),[found(self:agent,fakeAction(self:agent)),pickUp(self:agent)))", recoveryContext.getCommand());

        //Third child: pickup - FAIL_NOTFOUND
        Context pickupContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(2, pickupContext.getChildContexts().size());

        //Third child - pickup recovery
        recoveryContext = pickUpContext.getChildContexts().get(3);
        Assert.assertEquals("goal(pickupActionRecovery(self:agent,[pickupv2(self:agent),[found(self:agent,fakeAction(self:agent)),pickUp(self:agent)))", recoveryContext.getCommand());

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
        Assert.assertEquals(3, pickUpContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.SUCCESS, pickUpContext.getStatus());

        //(Tree looks this way because of catchingAction and postRecovery definitions)
        //First child: fail pickup because of gripperOpen precondition
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(0, pickupContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());

        //Second child: recovery for failed pickup (asserts gripperOpen) Should succeed after own recovery
        Context recoveryContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(1, recoveryContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());
        Context recoveryActionContext = recoveryContext.getChildContexts().get(0);
        Assert.assertEquals(4, recoveryActionContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryActionContext.getStatus());

        //First inner child: recovery fails itself due to preconditions of called action
        Context failActionContext = recoveryActionContext.getChildContexts().get(0);
        Assert.assertEquals("failAction", failActionContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, failActionContext.getStatus());

        //Second inner child: recovery of recovery policy - succeeds with postrecovery_retry
        recoveryContext = recoveryActionContext.getChildContexts().get(1);
        Assert.assertEquals("goal(failActionRecovery(self:agent,[failAction(self:agent),[failActionPre(self:agent),pickupNestedRecoverySuccess(self:agent,[pickup(self:agent),[gripperOpen(self:agent),pickUp(self:agent,success))))", recoveryContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());

        //Third inner child: failed action re-executed successfully
        failActionContext = recoveryActionContext.getChildContexts().get(2);
        Assert.assertEquals("failAction", failActionContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, failActionContext.getStatus());

        //Fourth inner child: success log

        //Third child: Successful pickup post recovery
        pickupContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(ActionStatus.SUCCESS, pickupContext.getStatus());
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
        Assert.assertEquals("pickUp", pickUpContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickUpContext.getStatus());
        Assert.assertEquals(2, pickUpContext.getChildContexts().size());

        //First child: failed pickup context - on gripperOpen precondition
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals("pickup", pickupContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());

        //Second child: Failed recovery context
        Context recoveryContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals("goal(pickupNestedRecoveryFail(self:agent,[pickup(self:agent),[gripperOpen(self:agent),pickUp(self:agent,fail)))", recoveryContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_RECOVERY, recoveryContext.getStatus());
        Assert.assertEquals(1, recoveryContext.getChildContexts().size());
        recoveryContext = recoveryContext.getChildContexts().get(0);
        Assert.assertEquals(1, recoveryContext.getChildContexts().size());

        //First inner child: failed pickup context - recovery policy simply called the failed action again without
        // changing anything. FAIL_PRECONDITIONS with the same justification. A valid recovery policy exists but it
        // should not be applied (because it is the same one)
        pickupContext = recoveryContext.getChildContexts().get(0);
        Assert.assertEquals(0, pickupContext.getChildContexts().size());
        Assert.assertEquals("pickup", pickupContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());

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
        Assert.assertEquals("pickUpV2", pickUpContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickUpContext.getStatus());
        Assert.assertEquals(4, pickUpContext.getChildContexts().size());

        Context logContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, logContext.getStatus());
        Assert.assertEquals("log", logContext.getCommand());

        //First pickup failure
        Context pickupContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());
        Assert.assertEquals("pickupv2", pickupContext.getCommand());

        //Recovery with postrecovery_retry
        Context recoveryContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());
        Assert.assertEquals("goal(pickupNestedRecoveryFaulty(self:agent,[pickupv2(self:agent),[gripperOpenV2(self:agent),pickUpV2(self:agent,child)))", recoveryContext.getCommand());

        //Reattempted pickup, identical failure
        pickupContext = pickUpContext.getChildContexts().get(3);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());
        Assert.assertEquals("pickupv2", pickupContext.getCommand());

        //Fail rather than enter duplicate recovery
    }
    /**
     * Tests that poorly defined recovery policies don't break execution. In this case, if an action recovery policy has
     * 'parent_retry' postrecovery execution defined but doesn't actually do anything to help solve the failure (would
     * normally lead to an infinite recovery loop)
     */
    @Ignore
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
        Assert.assertEquals("pickUpV2", pickUpContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickUpContext.getStatus());
        Assert.assertEquals(5, pickUpContext.getChildContexts().size());

        //First step: log
        Context logContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, logContext.getStatus());
        Assert.assertEquals("log", logContext.getCommand());

        //Second step: first pickup failure
        Context pickupContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());
        Assert.assertEquals("pickupv2", pickupContext.getCommand());

        //Third step: empty recovery with postrecovery_parent_retry
        Context recoveryContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());
        Assert.assertEquals("goal(pickupNestedParentRecoveryFaulty(self:agent,[pickupv2(self:agent),[gripperOpenV2(self:agent),pickUpV2(self:agent,parent)))", recoveryContext.getCommand());

        //Fourth step: first pickUp step again, log
        logContext = pickUpContext.getChildContexts().get(3);
        Assert.assertEquals(ActionStatus.SUCCESS, logContext.getStatus());
        Assert.assertEquals("log", logContext.getCommand());

        //Fifth step: second pickUp step again, pickup failure
        pickupContext = pickUpContext.getChildContexts().get(4);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());
        Assert.assertEquals("pickupv2", pickupContext.getCommand());

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
        Assert.assertEquals(3, pickUpContext.getChildContexts().size());

        //First child: First execution attempt - should fail due to gripperOpen precondition not being satisfied
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals("pickUp", pickupContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());

        //Second child: Recovery context (doesn't practically do anything)
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals("goal(pickupIneffectiveRecovery(self:agent,[pickup(self:agent), pickUp(self:agent,ineffective),[gripperOpen(self:agent),pickUp(self:agent,ineffective)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryGoalContext.getStatus());

        //Third child: Second execution attempt - should fail in the same way as the first
        pickupContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals("pickUp", pickupContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());

        //No more execution should occur
    }

    /**
     * Tests that when goal recovery 'successfully' completes but doesn't actually solve the issue, the system
     * prevents recovery from occurring on the same goal due to the same failure.
     * Variation where numAttempts is set to two
     */
    @Test
    public void ineffectiveGoalRecoveryMultipleAttemptsTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "goalRecoveryTests.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");


        log.info("\n\n\n[ineffectiveGoalRecoveryMultipleAttemptsTest]");
        //Submit goal for pickUp - recovery should cause goal to succeed
        Predicate goalPredicate = Factory.createPredicate("pickUp(self:agent,ineffectiveV2)");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId(goalPredicate);
        Goal goal = component.getGoal(goalId);

        //PickUp context - 3 children
        Context pickUpContext = goal.getRootContext();
        Assert.assertEquals(5, pickUpContext.getChildContexts().size());

        //First child: First execution attempt - should fail due to gripperOpen precondition not being satisfied
        Context pickupContext = pickUpContext.getChildContexts().get(0);
        Assert.assertEquals("pickUp", pickupContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());

        //Second child: Recovery context (doesn't practically do anything)
        Context recoveryGoalContext = pickUpContext.getChildContexts().get(1);
        Assert.assertEquals("goal(pickupIneffectiveRecoveryV2(self:agent,[pickup(self:agent), pickUp(self:agent,ineffectiveV2),[gripperOpen(self:agent),pickUp(self:agent,ineffectiveV2)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryGoalContext.getStatus());

        //Third child: Second execution attempt - should fail in the same way as the first
        pickupContext = pickUpContext.getChildContexts().get(2);
        Assert.assertEquals("pickUp", pickupContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());

        //Second child: Recovery context (doesn't practically do anything)
        recoveryGoalContext = pickUpContext.getChildContexts().get(3);
        Assert.assertEquals("goal(pickupIneffectiveRecoveryV2(self:agent,[pickup(self:agent), pickUp(self:agent,ineffectiveV2),[gripperOpen(self:agent),pickUp(self:agent,ineffectiveV2)))", recoveryGoalContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryGoalContext.getStatus());

        //Third child: Second execution attempt - should fail in the same way as the first
        pickupContext = pickUpContext.getChildContexts().get(4);
        Assert.assertEquals("pickUp", pickupContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickupContext.getStatus());

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
        Assert.assertEquals(3, rootContext.getChildContexts().size());

        //First attempt: fail
        Context testActionContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals("testActionRetry", testActionContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, testActionContext.getStatus());

        //Action recovery applied under goal context
        Context recoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals("goal(testActionRetryRecovery(self:agent,[testActionRetry(self:agent),[testActionPre(self:agent),testActionRetry(self:agent)))", recoveryContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());

        //Retry attempt: success
        testActionContext = rootContext.getChildContexts().get(2);
        Assert.assertEquals("testActionRetry", testActionContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, testActionContext.getStatus());
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
        Assert.assertEquals(2, rootContext.getChildContexts().size());

        //First attempt: fail
        Context testActionContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals("testActionSuccess", testActionContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, testActionContext.getStatus());

        //Action recovery applied under goal context
        Context recoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals("goal(testActionSuccessRecovery(self:agent,[testActionSuccess(self:agent),[testActionPre(self:agent),testActionSuccess(self:agent)))", recoveryContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());

        //postrecovery_continue -> no more execution
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
        Assert.assertEquals(1, rootContext.getChildContexts().size());

        Context closeGripperThriceContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(9, closeGripperThriceContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.SUCCESS, closeGripperThriceContext.getStatus());

        //First attempt: failure because gripper is not open
        Context closeGripperContext = closeGripperThriceContext.getChildContexts().get(0);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, closeGripperContext.getStatus());

        //First attempt recovery: sets precondition
        Context recoveryContext = closeGripperThriceContext.getChildContexts().get(1);
        Assert.assertEquals("goal(closeGripperActionRecovery(self:agent,[closeGripper(self:agent),[gripperOpen(self:agent),closeGripperThrice(self:agent)))", recoveryContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());

        //Second attempt: success
        closeGripperContext = closeGripperThriceContext.getChildContexts().get(2);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, closeGripperContext.getStatus());

        //Same sequence for second closeGripper
        //First attempt: failure because gripper is not open
        closeGripperContext = closeGripperThriceContext.getChildContexts().get(3);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, closeGripperContext.getStatus());

        //First attempt recovery: sets precondition
        recoveryContext = closeGripperThriceContext.getChildContexts().get(4);
        Assert.assertEquals("goal(closeGripperActionRecovery(self:agent,[closeGripper(self:agent),[gripperOpen(self:agent),closeGripperThrice(self:agent)))", recoveryContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());

        //Second attempt: success
        closeGripperContext = closeGripperThriceContext.getChildContexts().get(5);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, closeGripperContext.getStatus());

        //Same sequence for third closeGripper
        //First attempt: failure because gripper is not open
        closeGripperContext = closeGripperThriceContext.getChildContexts().get(6);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, closeGripperContext.getStatus());

        //First attempt recovery: sets precondition
        recoveryContext = closeGripperThriceContext.getChildContexts().get(7);
        Assert.assertEquals("goal(closeGripperActionRecovery(self:agent,[closeGripper(self:agent),[gripperOpen(self:agent),closeGripperThrice(self:agent)))", recoveryContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());

        //Second attempt: success
        closeGripperContext = closeGripperThriceContext.getChildContexts().get(8);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, closeGripperContext.getStatus());
    }

    /**
     * Tests that recovery does not abort execution when encountering the same failure on different steps of a script
     * (i.e. does not detect separate first attempt action failures as duplicate failures/failed recovery)
     * This tests this case when under the context of execution from retry postrecovery
     */
    @Test
    public void sameFailureDifferentStepRetryTest() {
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
        Assert.assertEquals(5, rootContext.getChildContexts().size());

        Context testActionSequenceContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(3, testActionSequenceContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, testActionSequenceContext.getStatus());

        //First attempt trace, fail on first closeGripper
        Context logContext = testActionSequenceContext.getChildContexts().get(0);
        Assert.assertEquals("log", logContext.getCommand());

        logContext = testActionSequenceContext.getChildContexts().get(1);
        Assert.assertEquals("log", logContext.getCommand());

        Context closeGripperContext = testActionSequenceContext.getChildContexts().get(2);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, closeGripperContext.getStatus());

        //Recovery
        Context recoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals("goal(testActionSequenceActionRecovery(self:agent,[closeGripper(self:agent), testActionSequence(self:agent,once),[gripperOpen(self:agent),testActionSequence(self:agent,once)))", recoveryContext.getCommand());

        //Second attempt trace, fail on second closeGripper
        testActionSequenceContext = rootContext.getChildContexts().get(2);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, testActionSequenceContext.getStatus());

        logContext = testActionSequenceContext.getChildContexts().get(0);
        Assert.assertEquals("log", logContext.getCommand());

        logContext = testActionSequenceContext.getChildContexts().get(1);
        Assert.assertEquals("log", logContext.getCommand());

        closeGripperContext = testActionSequenceContext.getChildContexts().get(2);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, closeGripperContext.getStatus());

        logContext = testActionSequenceContext.getChildContexts().get(3);
        Assert.assertEquals("log", logContext.getCommand());

        closeGripperContext = testActionSequenceContext.getChildContexts().get(4);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, closeGripperContext.getStatus());

        //Second recovery - made progress in script, so don't exit yet
        recoveryContext = rootContext.getChildContexts().get(3);
        Assert.assertEquals("goal(testActionSequenceActionRecovery(self:agent,[closeGripper(self:agent), testActionSequence(self:agent,once),[gripperOpen(self:agent),testActionSequence(self:agent,once)))", recoveryContext.getCommand());

        //Third attempt trace: fail on second closeGripper again
        testActionSequenceContext = rootContext.getChildContexts().get(4);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, testActionSequenceContext.getStatus());

        logContext = testActionSequenceContext.getChildContexts().get(0);
        Assert.assertEquals("log", logContext.getCommand());

        logContext = testActionSequenceContext.getChildContexts().get(1);
        Assert.assertEquals("log", logContext.getCommand());

        closeGripperContext = testActionSequenceContext.getChildContexts().get(2);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.SUCCESS, closeGripperContext.getStatus());

        logContext = testActionSequenceContext.getChildContexts().get(3);
        Assert.assertEquals("log", logContext.getCommand());

        closeGripperContext = testActionSequenceContext.getChildContexts().get(4);
        Assert.assertEquals("closeGripper", closeGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, closeGripperContext.getStatus());

        //Duplicate failure without progress, exit recovery and fail top level goal
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
        //Not writing out entire tree, should be same as the test above but with an extra recovery attempt
        Assert.assertEquals(7, rootContext.getChildContexts().size());
    }

    /**
     * Tests that recovery does not abort execution when encountering the same failure on different steps of a goal plan
     */
    @Test
    public void sameFailureDifferentStepGoalRecoveryTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
        String filename = aslPath + "multipleActionRecoveryTest.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[sameFailureDifferentStepGoalRecoveryTest]");
        Predicate goalPredicate = Factory.createPredicate("goal(self:agent,achievedGoal(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);
        Long goalId = component.getGoalId((Predicate)goalPredicate.get(1));
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(5, rootContext.getChildContexts().size());

        Context plannedContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(1, plannedContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, plannedContext.getStatus());

        //First attempt trace, fail on first closeGripper
        Context recoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals("goal(failActionRecovery(self:agent,[failAction(self:agent), actionone(self:agent), planned(self:agent),[impossibleToSatisfy(self:agent),achievedGoal(self:agent)))", recoveryContext.getCommand());

        plannedContext = rootContext.getChildContexts().get(2);
        Assert.assertEquals(2, plannedContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, plannedContext.getStatus());

        //First attempt trace, fail on first closeGripper
        recoveryContext = rootContext.getChildContexts().get(3);
        Assert.assertEquals("goal(failActionRecovery(self:agent,[failAction(self:agent), actionone(self:agent), planned(self:agent),[impossibleToSatisfy(self:agent),achievedGoal(self:agent)))", recoveryContext.getCommand());

        plannedContext = rootContext.getChildContexts().get(4);
        Assert.assertEquals(2, plannedContext.getChildContexts().size());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, plannedContext.getStatus());

        //Duplicate failure, exit recovery and fail top level goal
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
        Assert.assertEquals(1, rootContext.getChildContexts().size());

        //Recovery policy is defined with maximumAttempts of 3, should be applied 3 and result in a total of 4 action failures
        Context pickUpFailV2Context = rootContext.getChildContexts().get(0);
        Context pickUpV2Context = pickUpFailV2Context.getChildContexts().get(1);
        Assert.assertEquals(8, pickUpV2Context.getChildContexts().size());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, pickUpV2Context.getStatus());

        Context logContext = pickUpV2Context.getChildContexts().get(0);
        Assert.assertEquals("log", logContext.getCommand());

        // First attempt failure/recovery
        Context openGripperContext = pickUpV2Context.getChildContexts().get(1);
        Assert.assertEquals("openGripperV2", openGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, openGripperContext.getStatus());
        Context recoveryContext = pickUpV2Context.getChildContexts().get(2);
        Assert.assertEquals("goal(multipleAttemptsRecoveryPolicy(self:agent,[openGripperV2(self:agent),[not(holdingV2(self:agent)),pickUpFailV2(self:agent,multiple)))", recoveryContext.getCommand());

        // Second attempt failure/recovery
        openGripperContext = pickUpV2Context.getChildContexts().get(3);
        Assert.assertEquals("openGripperV2", openGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, openGripperContext.getStatus());
        recoveryContext = pickUpV2Context.getChildContexts().get(4);
        Assert.assertEquals("goal(multipleAttemptsRecoveryPolicy(self:agent,[openGripperV2(self:agent),[not(holdingV2(self:agent)),pickUpFailV2(self:agent,multiple)))", recoveryContext.getCommand());

        // Third attempt failure/recovery
        openGripperContext = pickUpV2Context.getChildContexts().get(5);
        Assert.assertEquals("openGripperV2", openGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, openGripperContext.getStatus());
        recoveryContext = pickUpV2Context.getChildContexts().get(6);
        Assert.assertEquals("goal(multipleAttemptsRecoveryPolicy(self:agent,[openGripperV2(self:agent),[not(holdingV2(self:agent)),pickUpFailV2(self:agent,multiple)))", recoveryContext.getCommand());

        //Fourth failure, no more recovery
        openGripperContext = pickUpV2Context.getChildContexts().get(7);
        Assert.assertEquals("openGripperV2", openGripperContext.getCommand());
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, openGripperContext.getStatus());
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
        Assert.assertEquals(5, rootContext.getChildContexts().size());

        //First attempt: failed due to precondition 1
        Context actionContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, actionContext.getStatus());
        Assert.assertEquals(actionContext.getJustification().getFailureReason().get(0), Factory.createPredicate("not(gripperOpen(self:agent))"));

        //Recovery policy 1: set precondition 1
        Context recoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());
        Assert.assertEquals("goal(cyclicalActionRecoveryTestActionRecoveryPolicy(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[gripperOpen(self:agent),cyclicalActionRecoveryTestAction(self:agent)))", recoveryContext.getCommand());

        //Second attempt: failed due to precondition 2
        actionContext = rootContext.getChildContexts().get(2);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, actionContext.getStatus());
        Assert.assertEquals(actionContext.getJustification().getFailureReason().get(0), Factory.createPredicate("not(fillerPredicate(self:agent))"));

        //Recovery policy 2: set precondition 2, unset precondition 1
        recoveryContext = rootContext.getChildContexts().get(3);
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());
        Assert.assertEquals("goal(cyclicalActionRecoveryTestActionRecoveryPolicyTwo(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[fillerPredicate(self:agent),cyclicalActionRecoveryTestAction(self:agent)))", recoveryContext.getCommand());

        //Third attempt: failed due to precondition 1 -> reach max_attempts for policy and exit recovery
        actionContext = rootContext.getChildContexts().get(4);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, actionContext.getStatus());
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
        Assert.assertEquals(1, rootContext.getChildContexts().size());

        Context executeActionTwiceContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(12, executeActionTwiceContext.getChildContexts().size());

        Context logContext = executeActionTwiceContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.SUCCESS, logContext.getStatus());
        Assert.assertEquals("log", logContext.getCommand());

        //Same sequence as test above, except second recovery policy doesn't unset precondition 1
        Context actionContext = executeActionTwiceContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, actionContext.getStatus());
        Assert.assertEquals("cyclicalActionRecoveryTestAction", actionContext.getCommand());
        Context recoveryContext = executeActionTwiceContext.getChildContexts().get(2);
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());
        Assert.assertEquals("goal(cyclicalActionRecoveryTestActionRecoveryPolicy(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[gripperOpen(self:agent),executeCyclicalTestActionTwice(self:agent)))", recoveryContext.getCommand());
        actionContext = executeActionTwiceContext.getChildContexts().get(3);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, actionContext.getStatus());
        Assert.assertEquals("cyclicalActionRecoveryTestAction", actionContext.getCommand());
        recoveryContext = executeActionTwiceContext.getChildContexts().get(4);
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());
        Assert.assertEquals("goal(cyclicalActionRecoveryTestActionRecoveryPolicyTwoSucceed(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[fillerPredicate(self:agent),executeCyclicalTestActionTwice(self:agent)))", recoveryContext.getCommand());
        actionContext = executeActionTwiceContext.getChildContexts().get(5);
        Assert.assertEquals(ActionStatus.SUCCESS, actionContext.getStatus());
        Assert.assertEquals("cyclicalActionRecoveryTestAction", actionContext.getCommand());

        logContext = executeActionTwiceContext.getChildContexts().get(6);
        Assert.assertEquals(ActionStatus.SUCCESS, logContext.getStatus());
        Assert.assertEquals("log", logContext.getCommand());

        //Same sequence for second instance of same action. Policy application count should properly be cleared such
        //  that this is allowed to successfully recovery rather than exit due to max_attempts
        actionContext = executeActionTwiceContext.getChildContexts().get(7);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, actionContext.getStatus());
        Assert.assertEquals("cyclicalActionRecoveryTestAction", actionContext.getCommand());
        recoveryContext = executeActionTwiceContext.getChildContexts().get(8);
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());
        Assert.assertEquals("goal(cyclicalActionRecoveryTestActionRecoveryPolicy(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[gripperOpen(self:agent),executeCyclicalTestActionTwice(self:agent)))", recoveryContext.getCommand());
        actionContext = executeActionTwiceContext.getChildContexts().get(9);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, actionContext.getStatus());
        Assert.assertEquals("cyclicalActionRecoveryTestAction", actionContext.getCommand());
        recoveryContext = executeActionTwiceContext.getChildContexts().get(10);
        Assert.assertEquals(ActionStatus.SUCCESS, recoveryContext.getStatus());
        Assert.assertEquals("goal(cyclicalActionRecoveryTestActionRecoveryPolicyTwoSucceed(self:agent,[cyclicalActionRecoveryTestAction(self:agent),[fillerPredicate(self:agent),executeCyclicalTestActionTwice(self:agent)))", recoveryContext.getCommand());
        actionContext = executeActionTwiceContext.getChildContexts().get(11);
        Assert.assertEquals(ActionStatus.SUCCESS, actionContext.getStatus());
        Assert.assertEquals("cyclicalActionRecoveryTestAction", actionContext.getCommand());
    }


    //TODO: Discuss whether side effects of making this possible are desired or not. Unclear what the correct answer is.
    // ^Side effect referenced is that it is now possible to cycle through recovery policies if more than one valid policy
    //  is present. I.e. try first policy, if fails then try second policy, and so on.
    // Also, this case is only relevant when a single action directly satisfies a supplied goal state (whether through
    //  postcondition match or direct action signature), otherwise there is a layer of abstraction with the 'planned' action.
    //  Should there ever actually be a goal recovery policy defined for this case which should not just be action recovery?
    /**
     * Tests case where there is both an action recovery policy and goal recovery policy defined for the same goal at
     * the same level (i.e. action recovery for an action which is a direct child of the goal context). Expects application
     * of action recovery first, then application of goal recovery when the prior fails.
     */
    @Test
    public void rootActionAndGoalRecoveryDefinedTest() {
        component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl -beliefinitfile recoverytests.pl -selector edu.tufts.hrilab.action.selector.UtilitarianActionSelector");
        String filename = aslPath + "dualRecoveryTest.asl";
        Database.getInstance().loadDatabaseFromFile(filename);
        Database.getInstance().loadDatabaseFromFile(aslPath + "recoveryTestsPlanningActions.asl");

        log.info("\n\n\n[rootActionAndGoalRecoveryDefinedTest]");
        //Predicate goalPredicate = Factory.createPredicate("pickUpWrapper(self:agent)");
        Predicate goalPredicate = Factory.createPredicate("goal(self:agent,pickUpWrapperSuccess(self:agent))");
        executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
        Long goalId = component.getGoalId((Predicate) goalPredicate.get(1));
        Goal goal = component.getGoal(goalId);
        Context rootContext = goal.getRootContext();
        Assert.assertEquals(4, rootContext.getChildContexts().size());

        Context failedPickupContext = rootContext.getChildContexts().get(0);
        Assert.assertEquals(ActionStatus.FAIL_PRECONDITIONS, failedPickupContext.getStatus());
        Assert.assertEquals("pickupwrapper", failedPickupContext.getCommand());

        Context failedActionRecoveryContext = rootContext.getChildContexts().get(1);
        Assert.assertEquals(ActionStatus.FAIL_RECOVERY, failedActionRecoveryContext.getStatus());
        Assert.assertEquals("goal(pickUpWrapperActionRecovery(self:agent,[pickup(self:agent), pickupwrapper(self:agent),[gripperOpen(self:agent),pickUpWrapperSuccess(self:agent)))", failedActionRecoveryContext.getCommand());

        Context successfulGoalRecoveryContext = rootContext.getChildContexts().get(2);
        Assert.assertEquals(ActionStatus.SUCCESS, successfulGoalRecoveryContext.getStatus());
        Assert.assertEquals("goal(pickUpWrapperGoalRecovery(self:agent,[],[found(self:agent,nonexistentAction(self:agent)),pickUpWrapperSuccess(self:agent)))", successfulGoalRecoveryContext.getCommand());

        Context successfulPickupContext = rootContext.getChildContexts().get(3);
        Assert.assertEquals(ActionStatus.SUCCESS, successfulPickupContext.getStatus());
        Assert.assertEquals("pickupwrapper", successfulPickupContext.getCommand());
    }
}
