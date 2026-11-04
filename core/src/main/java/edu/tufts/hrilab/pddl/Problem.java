/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pddl;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

public class Problem {

  private static final Logger log = LoggerFactory.getLogger(Problem.class);
  private final Map<String, Symbol> objects = new HashMap<>();
  private final List<Predicate> inits = new ArrayList<>();
  private final List<Predicate> goals = new ArrayList<>();
  private final List<Predicate> metrics = new ArrayList<>();

  // Turns the object into a PDDL formatted string
  public String generate(String name) {
    StringBuilder prefix = new StringBuilder();
    prefix.append("(define (problem ").append(name).append("problem").append(")\n");
    prefix.append("(:domain ").append(name).append("domain").append(")\n");
    StringJoiner problem = new StringJoiner("\n", prefix, "\n)");

    //objects
    StringJoiner objectString = new StringJoiner("\n", "(:objects\n", "\n)\n");
    for (Symbol object : objects.values()) {
      objectString.add("\t" + Generator.generateTyped(object));
    }
    problem.add(objectString.toString());

    //init
    StringJoiner initString = new StringJoiner("\n", "(:init\n", "\n)\n");
    for (Predicate init : inits) {
      initString.add("\t" + Generator.generate(init));
    }
    problem.add(initString.toString());

    //goal
    if (!goals.isEmpty()) {
      StringJoiner goalString;
      String goalPrefix, goalSuffix;
      if (goals.size() == 1) {
        goalPrefix = "(:goal\n";
        goalSuffix = "\n)";
      } else {
        goalPrefix = "(:goal (and\n";
        goalSuffix = "\n))";
      }

      goalString = new StringJoiner("\n", goalPrefix, goalSuffix);
      for (Predicate goal : goals) {
        goalString.add("\t" + Generator.generate(goal));
      }
      problem.add(goalString.toString());
    }

    //metric
    if (!metrics.isEmpty()) {
      StringJoiner metricString;
      String metricPrefix, metricSuffix;
      metricPrefix = "(:metric\n";
      metricSuffix = "\n)";

      metricString = new StringJoiner("\n", metricPrefix, metricSuffix);
      for (Predicate metric : metrics) {
        metricString.add("\t" + Generator.generate(metric).substring(1, Generator.generate(metric).length() - 1));
      }
      problem.add(metricString.toString());
    }
    return problem.toString();
  }

  public List<Symbol> getObjects() {
    return new ArrayList<>(objects.values());
  }

  public Symbol getObject(String objectName) {
    return objects.get(objectName);
  }

  public List<Predicate> getInits() {
    return inits;
  }

  public List<Predicate> getGoal() {
    return goals;
  }

  public void addObject(Symbol object) {
    if (objects.containsKey(object.getName())) {
      log.warn("Object of same name already exists. Ignoring: " + object);
    } else {
      objects.put(object.getName(), object);
    }
  }

  public void addInit(Predicate init) {
    if (!inits.contains(init)) {
      this.inits.add(init);
    }
  }

  public void addGoal(Predicate goal) {
    this.goals.add(goal);
  }

  protected boolean containsObjectName(String name) {
    return objects.containsKey(name);
  }

  public void addMetric(Predicate metric) {
    this.metrics.add(metric);
  }
}
