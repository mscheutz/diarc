/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.annotations;

import edu.tufts.hrilab.action.EffectType;

import java.lang.annotation.*;

/**
 * Annotation used to specify an effect of an action. Can be repeated.
 *
 * The predicates in the effect array will be added to the StateMachine/Belief.
 *
 * A predicate can be specified as observable, by adding it to the observable
 * array. In this case, it will first be observed that the effect does indeed
 * apply by calling an observer, e.g. Vision. If the predicate cannot be observed,
 * it will not be added to the StateMachine/Belief and the corresponding action
 * will fail.
 *
 * An effect can retract a predicate by placing it withing a not() operator, e.g.
 * not(on(table,mug)) will retract on(table,mug).
 *
 * Examples:
 *
 * - Standard effect. The free variables (?locA, ?locB) will be bound before assertion/retraction.
 *\     @Effect(effect={"at(car,?locB), not(at(car,?locA))"}, type = EffectType.SUCCESS)
 *
 * - Same as above but at(car,?locB) has to be observed before assertion, e.g. through Vision.
 *\     @Effect(effect={"at(car,?locB), not(at(car,?locA))"}, type = EffectType.SUCCESS, observable={"at(car,?locB)"})
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
@Repeatable(EffectsContainer.class)
public @interface Effect {
  String[] effect();
  EffectType type();
  String[] observable() default {};
}
