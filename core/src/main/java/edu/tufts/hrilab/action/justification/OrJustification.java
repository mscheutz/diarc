
/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.justification;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class OrJustification extends Justification {
  private List<Justification> justifications;

  public OrJustification() {
    super(false);
    justifications = new ArrayList<>();
  }

  public OrJustification(Justification justification) {
    super(false);
    justifications = new ArrayList<>();
    justifications.add(justification);
    setValue(justification.getValue());
  }

  @Override
  public List<Predicate> getPredicates() {
    List<Predicate> predicates = new ArrayList<>();
    for (Justification justification : justifications) {
      predicates.addAll(justification.getPredicates());
    }
    return predicates;
  }

  @Override
  public List<Map<Variable, Symbol>> getBindings() {
    List<Map<Variable, Symbol>> bindings = new ArrayList<>();
    for (Justification justification : justifications) {
      bindings.addAll(justification.getBindings());
    }
    return bindings;
  }

  @Override
  public List<Predicate> getFailedConditions() {
    List<Predicate> failurePredicates = new ArrayList<>();
    if (!getValue()) {
      for (Justification justification : justifications) {
        failurePredicates.addAll(justification.getFailedConditions());
      }
    }
    return failurePredicates;
  }

  @Override
  public List<Predicate> getFailureReason() {
    List<Predicate> failurePredicates = new ArrayList<>();
    if (!getValue()) {
      for (Justification justification : justifications) {
        failurePredicates.addAll(justification.getFailureReason());
      }
    }
    return failurePredicates;
  }

  @Override
  public List<Map<Variable, Symbol>> getFailureBindings() {
    List<Map<Variable, Symbol>> bindings = new ArrayList<>();
    if (!getValue()) {
      for (Justification justification : justifications) {
        if (!justification.getValue()) {
          bindings.addAll(justification.getFailureBindings());
        }
      }
    }
    return bindings;
  }
  
  @Override 
  public Set<Symbol> getBindings(Variable arg) {
    Set<Symbol> availableBindings = null;
    // loop through all justifications
    for (Justification just : justifications) {
      // get bindings of arg for justification
      Set<Symbol> tmpBindings = just.getBindings(arg);
      // check to see if g even used
      if (tmpBindings != null && !(tmpBindings.isEmpty())) {
        if (availableBindings == null) {
          availableBindings = new HashSet<>();
        }
        availableBindings.addAll(tmpBindings);
      }
    }
    return availableBindings;
  }

  public void addJustification(Justification justification) {
    justifications.add(justification);
    if (justification.getValue()) {
      setValue(true);
    }
  }

  public List<Justification> getJustifications() {
    return justifications;
  }

  @Override
  public String toString() {
    if (justifications.isEmpty()) {
      return "justifiction(empty)";
    } else if (justifications.size() == 1) {
      return justifications.get(0).toString();
    } else {
      StringBuilder sb = new StringBuilder("or(");
      justifications.forEach(j -> sb.append(j.toString()).append(","));
      sb.replace(sb.length() - 1, sb.length(), ")");
      return sb.toString();
    }
  }
}
