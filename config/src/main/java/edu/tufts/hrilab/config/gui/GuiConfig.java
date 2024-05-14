package edu.tufts.hrilab.config.gui;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.gui.DemoApplication;
import edu.tufts.hrilab.gui.ExampleService;
import edu.tufts.hrilab.map.MapComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.boot.SpringApplication;

// how I launch gui

public class GuiConfig extends DiarcConfiguration {
  protected static Logger log = LoggerFactory.getLogger(GuiConfig.class);

  // start the configuration
  @Override
  public void runConfiguration() {
    MapComponent map = DiarcComponent.createInstance(MapComponent.class, "-map_folder /home/hrilab/code/diarc/maps/elevator_lab_test/ -start_floor 1");
    log.info("Map Component initialized");

    // Initialize and register the ExampleService with DIARC
    // Passing an empty string array as the command-line arguments
    createInstance(ExampleService.class);
    log.info("ExampleService initialized");
  }

  public static void main(String[] args) {
    GuiConfig demoConfig = new GuiConfig();
    SpringApplication.run(DemoApplication.class, args);
    demoConfig.runConfiguration();
  }
}
