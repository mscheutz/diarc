/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.consultant;

import edu.tufts.hrilab.fol.Term;

import java.util.List;
import java.util.stream.Collectors;

public class SpotLocationReferenceJson {
  protected List<String> properties;
  protected String variable;
  protected String graphId;

  public SpotLocationReferenceJson(SpotLocationReference reference) {
    this.variable = reference.variable.toString();
    this.properties = reference.properties.stream().map(Term::toString).collect(Collectors.toList());
    this.graphId = reference.getWaypointId();
  }

  public String getVariable() {
    return variable;
  }
  public String getGraphId() {
    return graphId;
  }
  public List<String> getProperties() {
    return properties;
  }

}
