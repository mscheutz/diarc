/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarc;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

abstract public class DiarcComponent {
  /**
   * Logger for createInstance factory method.
   */
  private static Logger staticLogger = LoggerFactory.getLogger(DiarcComponent.class);
  /**
   * DIARC Component logger.
   */
  protected Logger log;
  /**
   * Executor to run the optional executionLoop.
   */
  private ScheduledExecutorService loopExecutor;
  /**
   * Flag used to start execution loop thread. False by default.
   */
  protected boolean shouldRunExecutionLoop = false;
  /**
   * Execution loop time (in milliseconds).
   */
  protected long executionLoopCycleTime = 100; // ms
  /**
   * TRADE groups. Set at runtime via passed in args.
   */
  List<String> groups = new ArrayList<>();
  /**
   * DIARC component's available services. Should only be used to pass as
   * callbacks to other notification services in the system.
   */
  Collection<TRADEServiceInfo> services;
  /**
   * Command line options.
   */
  Options cliOptions;

  protected DiarcComponent() {
    log = LoggerFactory.getLogger(this.getClass());
    Runtime.getRuntime().addShutdownHook(new Thread(() -> shutdown()));
  }

  /**
   * Perform any component initialization. This should not be used for setting local fields to default values
   * as was the case in DIARC. This method is called after the constructor, and after parseArgs.
   *
   * Setting default values should be done in field declaration or in the constructor.
   */
  protected void init() {
  }

  /**
   * Override this method to parse command line args in the sub-class. Called directly after construction to pass
   * runtime values that will override default values. This should parse all the options that additionalUsageInfo provides.
   *
   * @param cmdLine
   */
  protected void parseArgs(CommandLine cmdLine) {
  }

  /**
   * Override this method to define command line options available in sub-class.
   * This should be paired with a parseArgs implementation.
   *
   * @return
   */
  protected List<Option> additionalUsageInfo() {
    return new ArrayList<>();
  }

  /**
   * This is where any code that needs to be called repeatedly in a dedicated thread should live. It is not called
   * by default and you must set shouldRunExecutionLoop to true in your constructor.
   */
  protected void executionLoop() {
  }

  /**
   * Method called to start the execution loop, if shouldRunExecutionLoop set to true.
   */
  void startExecutionLoop() {
    loopExecutor = Executors.newScheduledThreadPool(1);
    loopExecutor.scheduleAtFixedRate(() -> {
      try {
        executionLoop();
      } catch (Throwable t) {  // Catch Throwable rather than Exception (a subclass).
        log.error("Exception caught in executionLoop.", t);
      }
    }, 0, executionLoopCycleTime, TimeUnit.MILLISECONDS);
  }

  /**
   * Any component that needs particular shutdown logic needs to override this method and
   */
  protected void shutdownComponent() {
    // purposely empty
  }

  /**
   * Main entrypoint for component shutdown logic. This is automatically triggered by the Java
   * shutdown hook (i.e., ctrl-C).
   */
  final public void shutdown() {
    log.debug("[shutdownComponent] shutdown started.");

    // call component specific shutdown logic (if any)
    shutdownComponent();

    shouldRunExecutionLoop = false;
    if (loopExecutor != null) {
      loopExecutor.shutdown();
    }
    try {
      TRADE.deregister(this);
    } catch (TRADEException e) {
      log.debug("[shutdownComponent] Exception while trying to deregister with TRADE.", e);
    }
    log.debug("[shutdownComponent] shutdown ended.");
  }

  /**
   * Collect command line options from sub-classes and combine them with DIARC options.
   */
  void collectAdditionalUsageInfo() {
    cliOptions = new Options();
    cliOptions.addOption(Option.builder("groups").longOpt("groups").hasArgs().desc("TRADE groups for this DIARC component").build());
    cliOptions.addOption(Option.builder("help").longOpt("help").desc("Print command line options").build());
    additionalUsageInfo().stream().forEach(option -> cliOptions.addOption(option));
  }

  /**
   * Parse DIARC command line options and then pass args onto sub-classes for consumption.zs
   *
   * @param args
   */
  void parseDiarcArgs(String[] args) {
    CommandLineParser parser = new DefaultParser();
    CommandLine cmd;
    try {
      cmd = parser.parse(cliOptions, args);
    } catch (ParseException e) {
      log.error("Could not parse args.", e);
      // automatically generate the help statement
      HelpFormatter formatter = new HelpFormatter();
      formatter.printHelp(this.getClass().toString(), cliOptions);
      System.exit(-1);
      return;
    }

    if (cmd.hasOption("groups")) {
      groups.addAll(Arrays.asList(cmd.getOptionValues("groups")));
    }
    if (cmd.hasOption("help")) {
      HelpFormatter formatter = new HelpFormatter();
      formatter.printHelp(this.getClass().toString(), cliOptions);
      System.exit(0);
    }

    // parse sub-class(es)
    parseArgs(cmd);
  }

  /**
   * Return DIARC component groups. This is a shallow copy.
   *
   * @return
   */
  public List<String> getMyGroups() {
    return new ArrayList<>(groups);
  }

  /**
   * Return DIARC component groups as a TRADEServiceConstraint instance, which can be further modified.
   *
   * @return
   */
  public TRADEServiceConstraints getMyGroupConstraints() {
    return new TRADEServiceConstraints().inGroups(groups.toArray(new String[0]));
  }

  /**
   * Get all services offered by this component.
   *
   * @return
   */
  public Collection<TRADEServiceInfo> getMyServices() {
    return services;
  }

  /**
   * Get a TRADEServiceInfo instance for one of the services provides by this DIARC Component.
   *
   * @param serviceName
   * @param argTypes
   * @return
   */
  public TRADEServiceInfo getMyService(String serviceName, Class<?>... argTypes) {

    for(TRADEServiceInfo tsi: services){
      if(tsi.serviceName.equals(serviceName) && Arrays.equals(tsi.getServiceParameterTypes(), argTypes)) {
        return tsi;
      }
    }
    log.warn("Matching service could not be found for: "+serviceName+" with args: "+ Arrays.toString(argTypes) +" returning null." );
    return null;
  }

  /**
   * Factory method to create a DIARC component instance, parseArgs, init, and finally register as
   * with TRADE.
   *
   * @param clazz             class to be instantiated (must extend DiarcComponent)
   * @param args              command line arguments
   * @param registerWithTrade true/false whether to register with TRADE
   * @param <T>
   * @return
   */
  static public final <T extends DiarcComponent> T createInstance(Class<T> clazz, String[] args, boolean registerWithTrade) {
    // find clazz default constructor
    Constructor<T> clazzConstructor;
    try {
      Class[] cArgs = new Class[]{};
      clazzConstructor = clazz.getDeclaredConstructor(cArgs);
    } catch (NoSuchMethodException e) {
      staticLogger.error("No default constructor found for class: " + clazz, e);
      return null;
    }

    // instantiate clazz instance
    T instance;
    try {
      instance = clazzConstructor.newInstance();
    } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
      staticLogger.error("Could not instantiate class: " + clazz, e);
      return null;
    }

    // call initialization methods
    instance.collectAdditionalUsageInfo();
    instance.parseDiarcArgs(args);
    instance.init();

    // optionally register with TRADE
    if (registerWithTrade) {
      try {
        instance.services = TRADE.registerAllServices(instance, instance.groups);
      } catch (TRADEException e) {
        staticLogger.debug("Could not register with TRADE.",e);
        instance.services = new HashSet<>();
      }
    } else {
      instance.services = new HashSet<>();
    }

    // optionally start execution loop thread
    if (instance.shouldRunExecutionLoop) {
      instance.startExecutionLoop();
    }
    return instance;
  }

  /**
   * Convenience method to automatically register with TRADE.
   *
   * @param clazz class to be instantiated (must extend DiarcComponent)
   * @param args  command line arguments
   * @param <T>
   * @return
   */
  static public final <T extends DiarcComponent> T createInstance(Class<T> clazz, String[] args) {
    return createInstance(clazz, args, true);
  }

  /**
   * Convenience method to pass all args as a single String, and automatically registers with TRADE.
   *
   * @param clazz class to be instantiated (must extend DiarcComponent)
   * @param args  command line arguments
   * @param <T>
   * @return
   */
  static public final <T extends DiarcComponent> T createInstance(Class<T> clazz, String args) {
    return createInstance(clazz, args.split(" "), true);
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
  static public final <T extends DiarcComponent> T createInstance(Class<T> clazz, String args, boolean registerWithTrade) {
    return createInstance(clazz, args.split(" "), registerWithTrade);
  }

  /**
   * Main method to allow starting of a single DIARC component from the command line.
   *
   * @param args
   */
  static public final void main(String[] args) {
    String component = System.getProperty("component");
    if (component == null && args.length >= 1) {
      component = args[0];
      args = Arrays.copyOfRange(args, 1, args.length);
    }
    staticLogger.info("Starting Component: " + ((component == null) ? "null" : component));

    Class componentClazz;
    try {
      componentClazz = Class.forName(component);
    } catch (ClassNotFoundException e) {
      staticLogger.error("Could not find class: " + component, e);
      return;
    }

    createInstance(componentClazz, args);
  }
}
