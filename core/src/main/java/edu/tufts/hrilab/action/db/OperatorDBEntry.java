/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.db;

import edu.tufts.hrilab.action.ActionBinding;

import java.lang.reflect.Method;
import java.lang.reflect.Parameter;
import java.util.ArrayList;
import java.util.List;

public class OperatorDBEntry extends DBEntry {

  private Method implementation;

  public OperatorDBEntry(String n, Method m) {
    super(n, null, getRolesFromMethod(m), m.isVarArgs());
    implementation = m;
  }

  /**
   * Gets the roles from the java Method implementing the operator.
   *
   * @param m method
   * @return roles
   */
  private static List<ActionBinding> getRolesFromMethod(Method m) {
    List<ActionBinding> roles = new ArrayList<>();
    for (Parameter p : m.getParameters()) {
      if (p.isVarArgs()) {
        roles.add(new ActionBinding.Builder(p.getName(), p.getType()).setIsVarArg(true).build());
      } else {
        roles.add(new ActionBinding.Builder(p.getName(), p.getType()).build());
      }
    }
    if (m.getReturnType() != void.class) {
      roles.add(new ActionBinding.Builder("ret", m.getReturnType()).setIsReturn(true).build());
    }
    return roles;
  }

  /**
   * Get java Method implementing this operator
   *
   * @return method
   */
  public Method getImplementation() {
    return implementation;
  }

}
