/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.planner.pddl4j;

import edu.tufts.hrilab.action.planner.Planner;
import fr.uga.pddl4j.parser.ErrorManager;
import fr.uga.pddl4j.encoding.CodedProblem;
import fr.uga.pddl4j.planners.*;
import fr.uga.pddl4j.planners.statespace.ff.FF;
import fr.uga.pddl4j.util.Plan;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;

public class Pddl4jPlanner extends Planner {
  protected static final Logger log = LoggerFactory.getLogger(Pddl4jPlanner.class);


  //Instantiate and run planner given the domain and problem
  @Override
  public String plan(File domain, File problem) {
    // parse input
    final ProblemFactory factory = ProblemFactory.getInstance();
    factory.setTraceLevel(4);
    parse(factory, domain, problem);
    final CodedProblem pb = factory.encode();

    // execute planner
    FF planner = new FF();
    Plan plan = planner.search(pb);
    if (plan != null) {
      String planString = pb.toString(plan);
      log.debug("Found plan as follows: " + planString);
      return planString;
    } else {
      log.debug("No plan found.");
      return null;
    }
  }

  //Validate domain and problem files.
  public void parse(ProblemFactory factory, File domain, File problem) {
    ErrorManager errorManager = null;

    try {
      errorManager = factory.parse(domain, problem);
    } catch (Exception e) {
      log.error("Unexpected error when parsing the PDDL planning problem description.", e);
      return;
    }

    if (!errorManager.isEmpty()) {
      log.error(errorManager.getMessages().toString());
      return;
    } else {
      log.debug("Parsing domain file and problem file done successfully");
    }
  }
}

