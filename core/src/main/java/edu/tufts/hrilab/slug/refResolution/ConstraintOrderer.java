/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import edu.tufts.hrilab.fol.Term;

import java.time.Duration;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class ConstraintOrderer {
  public abstract boolean order(Property a, Property b);

  public Comparator<Property> toComparator() {
    return (a, b) -> order(a, b) ? -1 : (order(b, a) ? 1 : 0);
  }
}

class PropertyTemplate {
  private final String name;
  private final int arity;
  private final double likelihood;
  private final Duration latency;
  private final double pFloor;
  private final double pCeiling;
  private final String header;

  public PropertyTemplate(String name, int arity, double likelihood, Duration latency, double pFloor, double pCeiling, String header) {
    this.name = name;
    this.arity = arity;
    this.likelihood = likelihood;
    this.latency = latency;
    this.pFloor = pFloor;
    this.pCeiling = pCeiling;
    this.header = header;
  }

  public PropertyTemplate(String name, int arity, double likelihood, Duration latency, double pFloor, double pCeiling) {
    this(name, arity, likelihood, latency, pFloor, pCeiling, "");
  }

  public String getName() { return name; }
  public int getArity() { return arity; }
  public double getLikelihood() { return likelihood; }
  public Duration getLatency() { return latency; }
  public double getPFloor() { return pFloor; }
  public double getPCeiling() { return pCeiling; }
  public String getHeader() { return header; }
}

class SizeOrderer extends ConstraintOrderer {
  private int argSize(Property a) {
    return ((Term) a.predicateForm().get(0)).size();
  }

  @Override
  public boolean order(Property a, Property b) {
    return argSize(a) < argSize(b);
  }
}

class ReverseSizeOrderer extends ConstraintOrderer {
  private int argSize(Property a) {
    return ((Term) a.predicateForm().get(0)).size();
  }

  @Override
  public boolean order(Property a, Property b) {
    return argSize(a) > argSize(b);
  }
}

class RandomSizeOrderer extends ConstraintOrderer {
  @Override
  public boolean order(Property a, Property b) {
    return Math.random() < 0.5;
  }
}

class CostOrderer extends ConstraintOrderer {
  private final Map<String, Duration> costMap;

  public CostOrderer(List<PropertyTemplate> pts) {
    costMap = new HashMap<>();
    for (PropertyTemplate pt : pts) {
      costMap.put(pt.getName(), pt.getLatency());
    }
  }

  private Duration cost(Property a) {
    return costMap.get(a.predicateForm().get(0).getName());
  }

  @Override
  public boolean order(Property a, Property b) {
    return cost(a).compareTo(cost(b)) < 0;
  }
}

class ReverseCostOrderer extends ConstraintOrderer {
  private final Map<String, Duration> costMap;

  public ReverseCostOrderer(List<PropertyTemplate> pts) {
    costMap = new HashMap<>();
    for (PropertyTemplate pt : pts) {
      costMap.put(pt.getName(), pt.getLatency());
    }
  }

  private Duration cost(Property a) {
    return costMap.get(a.predicateForm().get(0).getName());
  }

  @Override
  public boolean order(Property a, Property b) {
    return cost(a).compareTo(cost(b)) > 0;
  }
}

class FrequencyOrderer extends ConstraintOrderer {
  private final Map<String, Double> costMap;

  public FrequencyOrderer(List<PropertyTemplate> pts) {
    costMap = new HashMap<>();
    for (PropertyTemplate pt : pts) {
      costMap.put(pt.getName(), pt.getLikelihood());
    }
  }

  private double cost(Property a) {
    return costMap.get(a.predicateForm().get(0).getName());
  }

  @Override
  public boolean order(Property a, Property b) {
    return cost(a) < cost(b);
  }
}

class ReverseFrequencyOrderer extends ConstraintOrderer {
  private final Map<String, Double> costMap;

  public ReverseFrequencyOrderer(List<PropertyTemplate> pts) {
    costMap = new HashMap<>();
    for (PropertyTemplate pt : pts) {
      costMap.put(pt.getName(), pt.getLikelihood());
    }
  }

  private double cost(Property a) {
    return costMap.get(a.predicateForm().get(0).getName());
  }

  @Override
  public boolean order(Property a, Property b) {
    return cost(a) > cost(b);
  }
}
