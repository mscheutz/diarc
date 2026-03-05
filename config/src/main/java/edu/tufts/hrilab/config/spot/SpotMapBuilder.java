package edu.tufts.hrilab.config.spot;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.spot.SpotComponent;
import edu.tufts.hrilab.teleop.TeleopGui;
import edu.tufts.hrilab.trade.gui.TRADEServiceGui;

import javax.swing.*;

public class SpotMapBuilder extends DiarcConfiguration {
  private SpotComponent spot;

  @Override
  public void runConfiguration() {
    String spotConfigFile = "/home/evan/Documents/spot/spot_phi.yaml";
//    String tradeHubContainer = "127.0.0.1@10001";
    String tradeHubContainer = "192.168.0.46@10001";
    spot = createInstance(SpotComponent.class, "-spotConfigFile " + spotConfigFile
//            + " -groups agent:robotone "
            + " -hubContainer " + tradeHubContainer
//            + " -spotMap " + "/home/evan/Documents/spot/autowalk/sarah.walk"
            + " -refs " + "tmpSpotRefs.json");


    createInstance(TRADEServiceGui.class, "-class edu.tufts.hrilab.spot.SpotInterface");

    SwingUtilities.invokeLater(() -> new TeleopGui().start(new TeleopGui.TRADEVelocityPublisher()));
  }

}
