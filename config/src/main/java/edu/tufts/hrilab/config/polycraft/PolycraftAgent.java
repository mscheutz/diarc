/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.polycraft;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.pathfinding.PathPlannerComponent;
import edu.tufts.hrilab.polycraft.NovelRunComponent;
import edu.tufts.hrilab.polycraft.PolycraftComponent;
import edu.tufts.hrilab.polycraft.PolycraftRLComponent;
import edu.tufts.hrilab.polycraft.recovery.ExplorationComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class PolycraftAgent extends DiarcConfiguration {

  boolean useRL = false;

  StringBuilder additionPolycraftArgs = new StringBuilder();
  StringBuilder additionRLArgs = new StringBuilder();

  protected void parseArgs(CommandLine cmdLine) {

    if (cmdLine.hasOption("gameport")) {
      additionPolycraftArgs.append(" -gameport ").append(cmdLine.getOptionValue("gameport"));
    }
    if (cmdLine.hasOption("vision")) {
      int port = Integer.parseInt(cmdLine.getOptionValue("vision", "6011"));
      additionPolycraftArgs.append(" -vision ").append(port).append(" ");
    }
    if (cmdLine.hasOption("visionloc")) {
      int port = Integer.parseInt(cmdLine.getOptionValue("visionloc", "6011"));
      additionPolycraftArgs.append(" -visionloc ").append(port).append(" ");
    }
    if (cmdLine.hasOption("graph")) {
      int port = Integer.parseInt(cmdLine.getOptionValue("graph", "6012"));
      additionPolycraftArgs.append(" -graph ").append(port).append(" ");
    }
    if (cmdLine.hasOption("rl")) {
      useRL = true;
      int port = Integer.parseInt(cmdLine.getOptionValue("rl", "6013"));
      additionRLArgs.append(" -port ").append(port).append(" ");
    }
    if (cmdLine.hasOption("hint")) {
      additionPolycraftArgs.append(" -hint ");
    }
  }

  /**
   * Override this method to define command line options available in sub-class.
   * This should be paired with a parseArgs implementation.
   *
   * @return
   */
  protected List<Option> additionalUsageInfo() {

    List<Option> options = new ArrayList<>();
    options.add(Option.builder("gameport").hasArg().argName("port").desc("Set the polycraft/novelgridworlds port. (Default=9000)").build());
    options.add(Option.builder("vision").hasArg().argName("port").optionalArg(true).desc("Look for visual novelties. Optional port arg.").build());
    options.add(Option.builder("visionloc").hasArg().argName("port").optionalArg(true).desc("Look for visual novelties with localization. Optional port arg.").build());
    options.add(Option.builder("graph").hasArg().argName("port").optionalArg(true).desc("Look for graph novelties. Optional port arg.").build());
    options.add(Option.builder("rl").hasArg().argName("port").optionalArg(true).desc("Connect to RL agent.").build());
    options.add(Option.builder("hint").hasArg().argName("hint").optionalArg(true).desc("Look for hints. Optional port arg.").build());
    return options;
  }

  @Override
  public void runConfiguration() {

    createInstance(NovelRunComponent.class);
    createInstance(PolycraftComponent.class, additionPolycraftArgs.toString().trim());
    createInstance(PathPlannerComponent.class);
    createInstance(ExplorationComponent.class);
//    createInstance(edu.tufts.hrilab.action.mcts.AgentModel.class, "-agent rival");
//    createInstance(com.action.mcts.AgentModel.class, "-agent rival2");
    createInstance(GoalManagerComponent.class,
            "-beliefuniversal domains/polycraftuniversal.pl " +
                    "-dbfile core.asl domains/polycraft/polycraft.asl domains/polycraft/polycraft_nav_tp.asl domains/polycraft/polycraft2.asl domains/polycraft/recovery.asl " +
                    "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");

    if (useRL) {
      createInstance(PolycraftRLComponent.class, additionRLArgs.toString().trim());
    }
    log.info("Started.");
  }
}
