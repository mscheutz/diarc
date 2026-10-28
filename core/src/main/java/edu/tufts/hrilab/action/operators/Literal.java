/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.operators;

import java.lang.annotation.*;

/** Disable legacy text interpolation for a String input that represents literal data. */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.PARAMETER)
public @interface Literal { }
