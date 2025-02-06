/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Predicate;

import java.util.Calendar;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Implementation Notes:
 *     This context internally contains both the logic to sleep until the scheduled time and execute the scheduled action.
 *     It would make sense for this context to spawn two children contexts, one of which to handle the sleep and the
 *     other to be an ActionContext for the underlying action. It would make interactions with this context more natural.
 *
 *     The current issue is that the resources occupied by this goal needs to be disjoint from the resources occupied by
 *     the actual underlying goal (this goal needs to be active immediately, the underlying goal needs to adhere to the
 *     EM subclass' constraints. The other option of making this goal pending until the scheduled time directly makes
 *     interaction with persistent contexts strange and indirectly seems like it would require much more special logic
 *     in other places).
 *
 *     As such, the scheduled action itself is enacted through an internal submitGoal call (and then blocking and status
 *     propagation through joinOnGoal) in the scheduled task. This makes interaction with this goal a bit confusing
 *     because there is both the scheduled goal and the inner goal as independent entities with separate goalIds, but
 *     things have been implemented so that any interaction with either goal (i.e. interruption) affects both goals.
 *        //Unique exception is suspending inner goal will functionally suspend this goal but won't have its status reflected
 */

/**
 * This represents the semantics of a scheduled action.
 */
public class ScheduledContext extends Context {
    //Associated scheduled goal information
    Predicate scheduledGoalPred;
    long scheduledGoalId = -1;
    //Scheduling timing details
    private long delay;
    private TimeUnit timeUnit;
    private Date time;
    //Method of scheduling: true if using "delay" and "timeunit", false if using absolute time
    boolean relative = true;
    //Scheduled task objects
    TimerTask scheduledTask;
    Timer scheduledTaskTimer;
    ReentrantLock taskLock = new ReentrantLock();
    Condition taskComplete = taskLock.newCondition();
    private Lock doStepLock = new ReentrantLock();
    //True if the scheduled task is currently executing. False while sleeping before scheduled time and after execution.
    boolean scheduledTaskExecuting = false;
    private long timerStart = -1;

    public ScheduledContext(Context c, StateMachine sm, Predicate goalPred, long delay, TimeUnit timeUnit) {
        super(c, sm, "scheduled");
        this.delay = delay;
        this.timeUnit = timeUnit;
        scheduledGoalPred = goalPred;
        createScheduledTask();
    }

    public ScheduledContext(Context c, StateMachine sm, Predicate goalPred, Date time) {
        super(c, sm, "scheduled");
        this.time = time;
        relative = false;
        scheduledGoalPred = goalPred;
        createScheduledTask();
    }

    private void createScheduledTask() {
        scheduledTaskTimer = new Timer(true);
        scheduledTask = new TimerTask() {
            @Override
            public void run() {
                taskLock.lock();
                scheduledTaskExecuting = true;
                taskLock.unlock();
                timerStart = System.currentTimeMillis();
                try {
                    try {
                        scheduledGoalId = TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal").argTypes(Predicate.class)).call(Long.class, scheduledGoalPred);
                        GoalStatus goalStatus = TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class)).call(GoalStatus.class, scheduledGoalId);
                        if (goalStatus.isFailure()) {
                            ActionStatus actionStatus = TRADE.getAvailableService(new TRADEServiceConstraints().name("getActionStatus").argTypes(Long.class)).call(ActionStatus.class, scheduledGoalId);
                            Justification failConditions = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalFailConditions").argTypes(Long.class)).call(Justification.class, scheduledGoalId);
                            setStatus(actionStatus,failConditions);
                        }
                    } catch (TRADEException e) {
                        log.error("[createScheduledTask] exception submitting and joining on goal: {}", scheduledGoalPred, e);
                        setStatus(ActionStatus.FAIL);
                    }
                } finally {
                    taskLock.lock();
                    scheduledTaskExecuting = false;
                    taskComplete.signal();
                    taskLock.unlock();
                }
            }
        };
    }

    //Should there be different methods for updateScheduledDelay/time and updateCurrentDelay/Time?
    /**
     * Updates the scheduled delay of this context. If the action is currently executing, updates the delay for potential
     * future executions if this is a child of a persistent context and otherwise does nothing. If that action has not
     * begun execution, resets the scheduled task timer to occur on the supplied delay.
     */
    public void updateScheduledDelay(long delay, TimeUnit timeUnit) {
        //Always update these fields in case this context is a persistent goal's child and will be reused
        this.delay = delay;
        this.timeUnit = timeUnit;

        //If the task has not started execution yet, schedule the next execution to occur on this new delay
        if (!scheduledTaskExecuting && !doStepLock.tryLock()) {
            scheduledTaskTimer.cancel();
            scheduledTask.cancel();
            createScheduledTask();
            scheduledTaskTimer.schedule(scheduledTask, timeUnit.toMillis(delay));

            //Other option is to have the next execution be on this delay relative to original submission
            ////Q: What happens here if the new time is lower and the scheduled time is in the past? Does it execute immediately?
            //scheduledTaskTimer.schedule(scheduledTask, timerStart + timeUnit.toMillis(delay));
        }
    }

    /**
     * Updates the scheduled time of this context. If the action is currently executing, updates the time for potential
     * future executions if this is a child of a persistent context and otherwise does nothing. If that action has not
     * begun execution, resets the scheduled task timer to occur at the supplied time.
     */
    public void updateScheduledTime(Date time) {
        this.time = time;

        if (!scheduledTaskExecuting && !doStepLock.tryLock()) {
            scheduledTaskTimer.cancel();
            scheduledTask.cancel();
            createScheduledTask();
            //TODO: Are we expecting the day to be passed in correctly or just a time of day? Do we need to check if the time is in the past?
            scheduledTaskTimer.schedule(scheduledTask, time);
        }
    }

    @Override
    public void doStep() {
        switch (this.getExecType()) {
            case SIMULATE_ACT:
            case ACT:
                doStepLock.lock();
                try {
                    if (getStatus() == ActionStatus.RESUME && scheduledTaskExecuting) {
                        //Goal was suspended, resume inner goal and wait for completion signal from task blocking on joinOnGoal
                        try {
                            TRADE.getAvailableService(new TRADEServiceConstraints().name("resumeGoal").argTypes(Long.class)).call(boolean.class, scheduledGoalId);
                        } catch (TRADEException e) {
                            log.error("[ScheduledContext] exception setting scheduled goal status through TRADE", e);
                            //TODO: what justification to return in this case?
                            setStatus(ActionStatus.FAIL);
                        }
                    } else {
                        //Schedule inner goal to be submitted at the appropriate time
                        if (relative) {
                            timerStart = System.currentTimeMillis();
                            scheduledTaskTimer.schedule(scheduledTask, timeUnit.toMillis(delay));
                        } else {
                            scheduledTaskTimer.schedule(scheduledTask, time);
                        }
                    }
                    //Wait on goal to complete execution - signal in scheduled task after joinOnGoal returns
                    taskLock.lock();
                    try {
                        taskComplete.await();
                    } catch (InterruptedException e) {
                        log.error("[ScheduledContext] exception waiting on completion of scheduled task");
                    }
                    taskLock.unlock();
                } finally {
                    doStepLock.unlock();
                }
            default:
                break;
        }
    }

    @Override
    public ScheduledContext copy(Context newParent) {
        ScheduledContext newScheduledContext;
        if (relative) {
            //TODO: Should copying this have the current relative delay or the starting relative delay? (Or current only for one cycle)
            //long remainingDelay = timeUnit.toMillis(delay) - (System.currentTimeMillis() - timerStart);
            newScheduledContext = new ScheduledContext(newParent, newParent.stateMachine, scheduledGoalPred, delay, timeUnit);
        } else {
            newScheduledContext = new ScheduledContext(newParent, newParent.stateMachine, scheduledGoalPred, time);
        }

        copyInternal(newScheduledContext);
        return newScheduledContext;
    }

    @Override
    protected void resetConcreteContext() {
        scheduledTaskExecuting = false;
        timerStart = -1;
        if (!relative && System.currentTimeMillis() > time.getTime()) {
            //TODO: is this desired? Want it to work this way if wrapped in a persistent goal
            //If the task is scheduled for a Date which has already passed, advance the day
            Calendar cal = Calendar.getInstance();
            cal.setTime(time);
            cal.add(Calendar.DATE, 1);
            time = cal.getTime();
        }
        createScheduledTask();
    }


    @Override
    public synchronized boolean setStatus(ActionStatus eStatus, Justification justification) {
        if (eStatus == ActionStatus.CANCEL) {
            if (scheduledTaskExecuting) {
                //Cancelling this goal when the scheduled task is active -> cancel that task.
                try {
                    boolean canceled = TRADE.getAvailableService(new TRADEServiceConstraints().name("cancelGoal").argTypes(Long.class)).call(boolean.class, scheduledGoalId);
                    if (!canceled) {
                        log.warn("[ScheduledContext] unable to cancel scheduled goal, not setting status");
                        return false;
                    }
                } catch (TRADEException e) {
                    log.error("[ScheduledContext] exception setting scheduled goal status through TRADE", e);
                    //TODO: what justification to return in this case?
                    return setStatus(ActionStatus.FAIL);
                }
            } else {
                //Cancelling this goal before the scheduled task is active -> unblock 'sleep' in doStep and cancel the task
                scheduledTaskTimer.cancel();
                scheduledTask.cancel();
                taskLock.lock();
                taskComplete.signal();
                taskLock.unlock();
            }
        } else if (eStatus == ActionStatus.SUSPEND) {
            if (scheduledTaskExecuting) {
                //Task is active -> suspend it and unblock in doStep by signalling taskComplete
                //(The TimerTask will still be running and waiting on joinOnGoal to return. This will not occur until the
                //  original goal terminates after resume or cancel. In resume, merge into the still running timer task.)
                try {
                    boolean suspended = TRADE.getAvailableService(new TRADEServiceConstraints().name("suspendGoal").argTypes(Long.class)).call(boolean.class, scheduledGoalId);
                    if (!suspended) {
                        log.warn("[ScheduledContext] unable to suspend scheduled goal, not setting status");
                        return false;
                    }
                } catch (TRADEException e) {
                    log.error("[ScheduledContext] exception setting scheduled goal status through TRADE", e);
                    //TODO: what justification?
                    return setStatus(ActionStatus.FAIL);
                }
                taskLock.lock();
                taskComplete.signal();
                taskLock.unlock();
            } else {
                log.warn("[setStatus] cannot suspend scheduled goal which has not begun execution yet. Not updating status");
                return false;
            }
        }
        //Letting resume merging/logic occur in doStep

        return super.setStatus(eStatus,justification);
    }

    // Does it ever make sense to have this as a descendant of a noInterrupt context? Should these just return true?
    @Override
    public boolean canSuspendInterruptibly() {
        return isInterruptible();
    }

    @Override
    public boolean canCancelInterruptibly() {
        return isInterruptible();
    }
}
