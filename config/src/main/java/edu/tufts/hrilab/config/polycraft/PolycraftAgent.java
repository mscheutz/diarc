/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.polycraft;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.polycraft.NovelRunComponent;
import edu.tufts.hrilab.polycraft.PolycraftComponent;
import edu.tufts.hrilab.polycraft.recovery.ExplorationComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class PolycraftAgent {
  // for logging
  protected static Logger log = LoggerFactory.getLogger(PolycraftAgent.class);

  private static CommandLine parseRuntimeArgs(String[] args) {
    CommandLineParser parser = new DefaultParser();
    CommandLine cmd = null;
    Options cliOptions = new Options();
    cliOptions.addOption(Option.builder("gameport").hasArg().argName("port").desc("Set the polycraft/novelgridworlds port. (Default=9000)").build());
    cliOptions.addOption(Option.builder("vision").hasArg().argName("port").optionalArg(true).desc("Look for visual novelties. Optional port arg.").build());
    cliOptions.addOption(Option.builder("visionloc").hasArg().argName("port").optionalArg(true).desc("Look for visual novelties with localization. Optional port arg.").build());
    cliOptions.addOption(Option.builder("graph").hasArg().argName("port").optionalArg(true).desc("Look for graph novelties. Optional port arg.").build());
    cliOptions.addOption(Option.builder("rl").hasArg().argName("port").optionalArg(true).desc("Connect to RL agent.").build());
    cliOptions.addOption(Option.builder("hint").hasArg().argName("hint").optionalArg(true).desc("Look for hints. Optional port arg.").build());

    try {
      cmd = parser.parse(cliOptions, args);
    } catch (ParseException e) {
      log.error("Could not parse args.", e);
      // automatically generate the help statement
      HelpFormatter formatter = new HelpFormatter();
      formatter.printHelp(PolycraftAgent.class.toString(), cliOptions);
      System.exit(-1);
    }

    return cmd;
  }

  // start the configuration
  public static void main(String[] args) {
    // parse runtime args
    boolean rl = false;

    CommandLine cmd = parseRuntimeArgs(args);

    StringBuilder additionPolycraftArgs = new StringBuilder();
    StringBuilder additionRLArgs = new StringBuilder();
    if (cmd.hasOption("gameport")) {
      additionPolycraftArgs.append(" -gameport ").append(cmd.getOptionValue("gameport"));
    }
    if (cmd.hasOption("vision")) {
      int port = Integer.parseInt(cmd.getOptionValue("vision", "6011"));
      additionPolycraftArgs.append(" -vision ").append(port).append(" ");
    }
    if (cmd.hasOption("visionloc")) {
      int port = Integer.parseInt(cmd.getOptionValue("visionloc", "6011"));
      additionPolycraftArgs.append(" -visionloc ").append(port).append(" ");
    }
    if (cmd.hasOption("graph")) {
      int port = Integer.parseInt(cmd.getOptionValue("graph", "6012"));
      additionPolycraftArgs.append(" -graph ").append(port).append(" ");
    }
    if (cmd.hasOption("rl")) {
      rl = true;
      int port = Integer.parseInt(cmd.getOptionValue("rl", "6013"));
      additionRLArgs.append(" -port ").append(port).append(" ");
    }
    if (cmd.hasOption("hint")) {
      additionPolycraftArgs.append(" -hint ");
    }

    DiarcComponent.createInstance(NovelRunComponent.class, "");
    DiarcComponent.createInstance(PolycraftComponent.class, additionPolycraftArgs.toString().trim());
    DiarcComponent.createInstance(com.pathfinding.PathPlannerComponent.class, "");
    DiarcComponent.createInstance(ExplorationComponent.class, "");
//    DiarcComponent.createInstance(edu.tufts.hrilab.action.mcts.AgentModel.class, "-agent rival");
//    DiarcComponent.createInstance(com.action.mcts.AgentModel.class, "-agent rival2");
    DiarcComponent.createInstance(edu.tufts.hrilab.action.GoalManagerImpl.class,
            "-beliefuniversal domains/polycraftuniversal.pl " +
                    "-dbfile core.asl domains/polycraft/polycraft.asl domains/polycraft/polycraft_nav_tp.asl domains/polycraft/polycraft2.asl domains/polycraft/recovery.asl " +
//                    "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector");
    "-selector com.action.selector.MCTSActionSelector");

    if (rl) {
      DiarcComponent.createInstance(edu.tufts.hrilab.polycraft.PolycraftRLComponent.class, additionRLArgs.toString().trim());
    }
    log.info("Started.");
  }
}
