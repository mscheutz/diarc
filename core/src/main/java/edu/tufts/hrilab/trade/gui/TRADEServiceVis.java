/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.trade.gui;

import java.awt.BorderLayout;
import java.util.List;

import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import javax.swing.*;

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
public class TRADEServiceVis extends JFrame {

  private static final Logger log = LoggerFactory.getLogger(TRADEServiceVis.class);

  public TRADEServiceVis(Class clazz, List<String> clazzGroups) {
    super();
    TRADEServicePanel panel = new TRADEServicePanel(clazz, clazzGroups);
    this.setLayout(new BorderLayout());
    this.add(panel, BorderLayout.CENTER);
    this.validate();
  }
}
