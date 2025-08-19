package edu.tufts.hrilab.spot;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.python.PythonWrapper;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class SpotComponent extends DiarcComponent {
  /**
   * ip@port of hub container that python spokes should connect to
   */
  private String hubContainer;
  /**
   * Path to spot config YAML file
   */
  private String spotConfigFile;

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("hubContainer").hasArg().argName("ip@port").desc("Set internal python modules to run as spoke with this hub").build());
    options.add(Option.builder("spotConfigFile").hasArg().argName("file").desc("Path to yaml spot config").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("hubContainer")) {
      hubContainer = cmdLine.getOptionValue("hubContainer");
    }
    if (cmdLine.hasOption("spotConfigFile")) {
      spotConfigFile = cmdLine.getOptionValue("spotConfigFile");
    }
  }

  @Override
  public void init() {
    String myGroups = String.join(", ", getMyGroups());

    PythonWrapper wrapper = new PythonWrapper("diarc.spot.spot_trade_wrapper", true, hubContainer); // Declare the wrapper as a TRADE spoke
    wrapper.start("--spot-config",  spotConfigFile, "--groups", myGroups); // Start the wrapper
  }

}
