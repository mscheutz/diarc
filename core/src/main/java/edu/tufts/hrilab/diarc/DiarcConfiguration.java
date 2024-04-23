/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarc;

import edu.tufts.hrilab.action.GoalManagerImpl;

import java.util.ArrayList;
import java.util.List;

abstract public class DiarcConfiguration {
  protected List<DiarcComponent> diarcComponents = new ArrayList<>();

  abstract public void runConfiguration();

  public void shutdownConfiguration() {
    // find and shutdown GoalManagers first so that goals can be cleanly cancelled while other components are still up
    List<DiarcComponent> gms = diarcComponents.stream().filter(comp -> comp instanceof GoalManagerImpl).toList();
    gms.forEach(gm -> gm.shutdown());
    diarcComponents.removeAll(gms);

    // shutdown all other components
    diarcComponents.forEach(diarcComponent -> diarcComponent.shutdown());
    diarcComponents.clear();
  }

  /**
   * Convenience method when no additional component args are needed. Automatically register with TRADE.
   *
   * @param clazz class to be instantiated (must extend DiarcComponent)
   * @param <T>
   * @return
   */
  public final <T extends DiarcComponent> T createInstance(Class<T> clazz) {
    T component = DiarcComponent.createInstance(clazz, "", true);
    diarcComponents.add(component);
    return component;
  }

  /**
   * Convenience method to automatically register with TRADE.
   *
   * @param clazz class to be instantiated (must extend DiarcComponent)
   * @param args  command line arguments
   * @param <T>
   * @return
   */
  public final <T extends DiarcComponent> T createInstance(Class<T> clazz, String[] args) {
    T component = DiarcComponent.createInstance(clazz, args, true);
    diarcComponents.add(component);
    return component;
  }

  /**
   * Convenience method to pass all args as a single String, and automatically registers with TRADE.
   *
   * @param clazz class to be instantiated (must extend DiarcComponent)
   * @param args  command line arguments
   * @param <T>
   * @return
   */
  public final <T extends DiarcComponent> T createInstance(Class<T> clazz, String args) {
    T component = DiarcComponent.createInstance(clazz, args.split(" "), true);
    diarcComponents.add(component);
    return component;
  }

  /**
   * Convenience method to pass all args as a single String, and optionally registers with TRADE.
   *
   * @param clazz             class to be instantiated (must extend DiarcComponent)
   * @param args              command line arguments
   * @param registerWithTrade true/false whether to register with TRADE
   * @param <T>
   * @return
   */
  public final <T extends DiarcComponent> T createInstance(Class<T> clazz, String args, boolean registerWithTrade) {
    T component = DiarcComponent.createInstance(clazz, args.split(" "), registerWithTrade);
    diarcComponents.add(component);
    return component;
  }

}
