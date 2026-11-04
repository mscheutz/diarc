/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.annotations;

import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface OnInterrupt {
  String onCancelServiceCall() default "";
  String onSuspendServiceCall() default "";
  String onResumeServiceCall() default "";
}
