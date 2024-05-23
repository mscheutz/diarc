package edu.tufts.hrilab.config.gui;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.gui.DemoApplication;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.boot.SpringApplication;

import edu.tufts.hrilab.map.MapComponent;

// how I launch gui

public class GuiConfig extends DiarcConfiguration {
  protected static Logger log = LoggerFactory.getLogger(GuiConfig.class);

  // start the configuration
  @Override
  public void runConfiguration() {
  }

  public static void main(String[] args) {
    GuiConfig demoConfig = new GuiConfig();
    SpringApplication.run(DemoApplication.class, args);
    demoConfig.runConfiguration();
  }
}
