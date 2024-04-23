/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.annotations;

import edu.tufts.hrilab.action.ConditionType;

import java.lang.annotation.*;

/**
 * Annotation used to specify a condition for an action. Can be repeated.
 *
 * We use the conjunctive normal form (CNF). Each @Condition annotation has to hold true.
 * Within the @Condition annotation, at least one predicate has to hold true (disjunction).
 * Put it simply, we're doing ANDs of ORs: (C1 and (C2.1 or C2.2) and C3).
 *
 * By default, each predicate part of the condition will tentatively be observed first,
 * and, if not found, inferred in the StateMachine/Belief Component. It is
 * possible to force the observation of a predicate by adding it to the observable array.
 * In this case, the predicate will only be observed. If it can't be found, the
 * StateMachine/Belief will not be checked. It is also possible to force a predicate to be
 * only checked in Belief by adding it to the inferable array.
 *
 * Examples:
 *
 * - Standard condition. The free variable (?x) will be bound during checking (if found).
 *\     @Condition(condition={"on(table,?x)"}, type = ConditionType.PRE)
 *
 * - Condition with two predicates. At least one of them has to hold true (disjunction).
 *\     @Condition(condition={"on(table,?x)","on(shelf,?x)"}, type = ConditionType.PRE)
 *
 * - Same as above but on(table,?x) has to be observed, e.g. through Vision.
 *\     @Condition(condition={"on(table,?x)","on(shelf,?x)"}, type = ConditionType.PRE, observable={"on(table,?x)"})
 *
 * - This time on(table,?x) has to be inferred (in StateMachine/Belief) and will not be observed.
 *\     @Condition(condition={"on(table,?x)","on(shelf,?x)"}, type = ConditionType.PRE, inferable={"on(table,?x)"})
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
@Repeatable(ConditionsContainer.class)
public @interface Condition {
  String[] condition();
  ConditionType type();
  String[] observable() default {};
  String[] inferable() default {};
}
