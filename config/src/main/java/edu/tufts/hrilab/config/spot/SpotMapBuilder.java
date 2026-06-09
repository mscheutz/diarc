package edu.tufts.hrilab.config.spot;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.spot.SpotComponent;
import edu.tufts.hrilab.teleop.TeleopGui;
import edu.tufts.hrilab.trade.gui.TRADEServiceGui;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import javax.swing.*;
import java.util.ArrayList;
import java.util.List;

public class SpotMapBuilder extends DiarcConfiguration {
  private String spotConfigFile = "/home/evan/Documents/spot/spot_phi.yaml";
  private String tradeHubContainer = "127.0.0.1@10001";

  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("spotConfig")) {
      spotConfigFile = cmdLine.getOptionValue("spotConfig");
    }
    if (cmdLine.hasOption("tradeHubContainer")) {
      tradeHubContainer = cmdLine.getOptionValue("tradeHubContainer");
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
    options.add(Option.builder("spotConfig").hasArg().argName("file").desc("Spot SDK YAML config file.").build());
    options.add(Option.builder("hub").longOpt("tradeHubContainer").hasArg().argName("ip@port").desc("IP and port of TRADE hub container.").build());
    return options;
  }

  @Override
  public void runConfiguration() {
    createInstance(SpotComponent.class, "-spotConfigFile " + spotConfigFile
                    + " -hubContainer " + tradeHubContainer
    );

    createInstance(TRADEServiceGui.class, "-class edu.tufts.hrilab.spot.SpotInterface");

    SwingUtilities.invokeLater(() -> new TeleopGui().start(new TeleopGui.TRADEVelocityPublisher()));
  }

}
