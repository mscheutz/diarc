/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.trade.gui;

import javax.swing.JFrame;

import edu.tufts.hrilab.diarc.DiarcComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

/**
 * Generic GUI for any class that offers TRADE services. This uses Java reflection to
 * expose all of the methods available as a TRADE service. Note
 * that all parameter arguments require instantiating the parameter's Object
 * type from a String and the only way to do this is to hand-write each needed
 * type. Add any needed type to the growing if-else statement in
 * MethodCallHelper.callMethod() below.
 *
 * @author Evan Krause
 */
public class TRADEServiceGui extends DiarcComponent {
  private Class testComponentClass;
  private JFrame frame;
  final private Logger log = LoggerFactory.getLogger(this.getClass());
    
  public TRADEServiceGui() {
    super();
  }
  @Override
  protected void init() {
    if (testComponentClass == null) {
      log.error("Target class not set. Aborting!");
      return;
    }

    // create gui panel with all component related content
    TRADEServicePanel panel = new TRADEServicePanel(testComponentClass, getMyGroups());

    // set up JFrame
    frame = new JFrame("Component Interface: " + testComponentClass.getName());
    frame.setVisible(true);

    frame.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);
    frame.add(panel);
    frame.pack(); // to automatically resize the frame
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("class")) {
      String className = cmdLine.getOptionValue("class");
      try {
        testComponentClass = Class.forName(className);
      } catch (ClassNotFoundException e) {
        log.error("Class not found: " + className, e);
      }
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("class").numberOfArgs(1).argName("classpath").desc("GUI to make TRADE service calls to a particular class.").build());
    return options;
  }
}
