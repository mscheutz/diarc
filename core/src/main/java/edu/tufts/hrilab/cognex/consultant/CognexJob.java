/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.cognex.consultant;

import edu.tufts.hrilab.fol.Symbol;

import java.util.ArrayList;
import java.util.List;

public class CognexJob {

  @Override
  public String toString() {
    return "CognexJob{" +
            "name='" + name + '\'' +
            ", type='" + type + '\'' +
            ", jobs=" + jobs +
            '}';
  }

  /**
   * The name of the job on the Cognex, passed into get camera data.
   */
  private String name;

  //TODO:brad: make this an ENUM instead of a string
  private String type;

  /**
   * the indices in jobs represent the job index on the Cognex
   */
  List<Symbol> jobs = new ArrayList<>();

  public CognexJob(String name, String type) {
    this.name = name;
    this.type = type;
  }

  public void addJob(Symbol s) {
    this.jobs.add(s);
  }

  public String getType() {
    return type;
  }

  public String getName() {
    return name;
  }

  public Symbol getSymbol(Integer i) {
    return jobs.get(i);
  }
}
