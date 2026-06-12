package edu.tufts.hrilab.spot;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.util.resource.Resources;
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
  private String spotMapFile = null;
  private String refsConfigFile = null;
  private String refsConfigDir = "config/edu/tufts/hrilab/spot";

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("hub").longOpt("tradeHubContainer").hasArg().argName("ip@port").desc("Set internal python modules to run as spoke with this hub").build());
    options.add(Option.builder("spotConfigFile").hasArg().argName("file").desc("Path to yaml spot config").build());
    options.add(Option.builder("spotMap").hasArg().argName("file").desc("Path to map file").build());
    options.add(Option.builder("refs").hasArg().argName("file").desc("Path to location refs file").build());
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
    if (cmdLine.hasOption("spotMap")) {
      spotMapFile = cmdLine.getOptionValue("spotMap");
    }
    if (cmdLine.hasOption("refs")) {
      refsConfigFile = cmdLine.getOptionValue("refs");
    }
  }

  @Override
  public void init() {
    PythonWrapper wrapper = new PythonWrapper("diarc.spot.spot_trade_wrapper", true, hubContainer); // Declare the wrapper as a TRADE spoke
    List<String> args = new ArrayList<>();

    args.add("--spot-config");
    args.add(spotConfigFile);

    args.add("--groups");
    args.addAll(getMyGroups());

    if (spotMapFile != null) {
      args.add("--spot-map");
      args.add(spotMapFile);
    }

    if (refsConfigFile != null && !refsConfigFile.isEmpty()) {
      String filename = Resources.createFilepath(refsConfigDir, refsConfigFile);
      args.add("--refs");
      args.add(filename);
    }

    wrapper.start(args.toArray(new String[args.size()])); // Start the wrapper
  }

}
