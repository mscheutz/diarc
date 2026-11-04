/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.annotations;

import java.lang.annotation.*;

/**
 * Annotation used to specify an observation that an action can make. Cannot be repeated.
 * <p>
 * Should only be used with java methods of the following form:
 * {@code List<Map<Variable,Symbol>> method(Term term);}
 * <p>
 * i.e., the method has to take a single Term (or Predicate) as an argument and return
 * a list of maps from Variable to Symbol. Methods that do not respect this will not
 * be available as observers in action and a warning will be shown.
 * <p>
 * Examples:
 * <p>
 * - Annotation for an action that will observe if ?x is on the table or on the shelf.
 *\   @Observes({"on(table,?x)", "on(shelf,?x)"})
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface Observes {
  String[] value();
}
