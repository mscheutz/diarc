/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pddl;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class Operators {
  public final static Map<String, String> DIARC_TO_PDDL;
  public final static Map<String, String> PDDL_TO_DIARC;
  static {
    Map<String, String> tmpDIARC = new HashMap<>();
    // TODO: remove all "fluent_<>" in favor of just "<>"
//    tmpDIARC.put("increase", "increase");
//    tmpDIARC.put("decrease", "decrease");
//    tmpDIARC.put("less", "<");
//    tmpDIARC.put("leq", "<=");
    tmpDIARC.put("equals", "=");
//    tmpDIARC.put("geq", ">=");
//    tmpDIARC.put("greater", ">");
    tmpDIARC.put("fluent_increase", "increase");
    tmpDIARC.put("fluent_decrease", "decrease");
    tmpDIARC.put("fluent_less", "<");
    tmpDIARC.put("fluent_leq", "<=");
    tmpDIARC.put("fluent_equals", "=");
    tmpDIARC.put("fluent_geq", ">=");
    tmpDIARC.put("fluent_greater", ">");
    tmpDIARC.put("mul", "*");
    tmpDIARC.put("div", "/");
    tmpDIARC.put("add", "+");
    tmpDIARC.put("sub", "-");
    tmpDIARC.put("assign", "assign");
    tmpDIARC.put("fluent_assign", "assign");
    DIARC_TO_PDDL = Collections.unmodifiableMap(tmpDIARC);

    // reverse map for pddl -> diarc
    Map<String, String> tmpPDDL = new HashMap<>();
    DIARC_TO_PDDL.forEach((k,v) -> tmpPDDL.put(v,k));
    PDDL_TO_DIARC = Collections.unmodifiableMap(tmpPDDL);
  }
}
