/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.annotations;

import edu.tufts.hrilab.action.ActionStatus;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface RecoveryPolicy {
  String actor() default "";
  String goal() default "";
  String failedAction() default "";
  String[] failureReasons() default {};
  ActionStatus actionStatus();
}