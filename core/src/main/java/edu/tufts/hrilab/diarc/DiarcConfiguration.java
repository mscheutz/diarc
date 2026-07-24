/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarc;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.gui.GuiManager;
import org.apache.commons.cli.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.boot.SpringApplication;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.StringJoiner;

abstract public class DiarcConfiguration {
  /**
   * Logger for createInstance factory method.
   */
  private static Logger staticLogger = LoggerFactory.getLogger(DiarcComponent.class);

  protected Logger log;

  private boolean launchGuiBackend = false;

  protected List<DiarcComponent> diarcComponents = new ArrayList<>();

  private boolean singleComponentPerContainer = false;

  public void setSingleComponentPerContainer(boolean singleComponentPerContainer) {
    log.warn("singleComponentPerContainer is currently disabled.");
    //this.singleComponentPerContainer = singleComponentPerContainer;
  }

  /**
   * Command line options.
   */
  protected Options cliOptions;

  abstract public void runConfiguration();

  public DiarcConfiguration() {
    log = LoggerFactory.getLogger(this.getClass());
    Runtime.getRuntime().addShutdownHook(new Thread(() -> shutdownConfiguration()));
  }

  public DiarcConfiguration(String... args) {
    this();

    if (args.length > 0) {
      collectAdditionalUsageInfo();

      // put input args into the expected form for component arg parsing
      StringJoiner joiner = new StringJoiner(" ");
      Arrays.stream(args).forEach(arg -> joiner.add(arg.trim()));
      String[] cliArgs = joiner.toString().split(" ");

      parseDiarcArgs(cliArgs);
    }
  }

  public void shutdownConfiguration() {
    log.info("Shutting down...");

    // find and shutdown GoalManagers first so that goals can be cleanly cancelled while other components are still up
    List<DiarcComponent> gms = diarcComponents.stream().filter(comp -> comp instanceof GoalManagerComponent).toList();
    gms.forEach(gm -> gm.shutdown());
    diarcComponents.removeAll(gms);

    // shutdown all other components
    diarcComponents.forEach(diarcComponent -> diarcComponent.shutdown());
    diarcComponents.clear();
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
   * Collect command line options from sub-classes and combine them with DIARC options.
   */
  void collectAdditionalUsageInfo() {
    cliOptions = new Options();
    cliOptions.addOption(Option.builder("g").longOpt("gui").desc("Enable web gui backend").build());
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

    if (cmd.hasOption("gui")) {
      launchGuiBackend = true;
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
   * Convenience method to automatically register with TRADE.
   *
   * @param clazz class to be instantiated (must extend DiarcComponent)
   * @param args  command line arguments
   * @param <T>
   * @return
   */
  public final <T extends DiarcComponent> T createInstance(Class<T> clazz, String... args) {
    return createInstance(clazz, true, args);
  }

  /**
   * Create a new DiarcComponent instance, and optionally registers with TRADE.
   *
   * @param clazz             class to be instantiated (must extend DiarcComponent)
   * @param registerWithTrade true/false whether to register with TRADE
   * @param args              command line arguments
   * @param <T>
   * @return
   */
  public final <T extends DiarcComponent> T createInstance(Class<T> clazz, boolean registerWithTrade, String... args) {
    if (singleComponentPerContainer) {
      if (!registerWithTrade) {
        log.warn("All components in a single container must be registered with TRADE. Ignoring registration request for: " + clazz.getName() + " with args: " + Arrays.toString(args) + " and starting in a new container.");
      }
      return createInstanceInNewContainer(clazz, args);
    } else {
      T component = DiarcComponent.createInstance(clazz, registerWithTrade, args);
      diarcComponents.add(component);
      return component;
    }
  }

  private <T extends DiarcComponent> T createInstanceInNewContainer(Class<T> clazz, String... args) {
    // path to the current Java runtime executable
    String javaHome = System.getProperty("java.home");
    String javaBin = javaHome + File.separator + "bin" + File.separator + "java";

    // current classpath to ensure the sub-process can find the target class
    String classpath = System.getProperty("java.class.path");

    // command arguments list
    List<String> command = new ArrayList<>();
    command.add(javaBin);
    command.add("-cp");
    command.add(classpath);

    // set system properties
    command.add("-Dlogback.configurationFile=" + System.getProperty("logback.configurationFile"));
    command.add("-Dlogging.config=" + System.getProperty("logging.config"));
    command.add("-DtradeLogging.config.path=" + System.getProperty("tradeLogging.config.path"));
    command.add("-Dtrade.properties.path=" + System.getProperty("trade.properties.path"));
    command.add("-Ddiarc.planner.ff=" + System.getProperty("diarc.planner.ff"));
    command.add("-Djava.library.path=" + System.getProperty("java.library.path"));

    command.add("-Dcomponent=" + clazz.getName());
    command.add(clazz.getName());

    if (args != null) {
      StringJoiner joiner = new StringJoiner(" ");
      Arrays.stream(args).forEach(arg -> joiner.add(arg.trim()));
      command.add(joiner.toString());
    }

    // 4. Build and start the process
    ProcessBuilder builder = new ProcessBuilder(command);

    // Redirect streams so you can see the sub-process output in the main console
    builder.inheritIO();

    try {
      Process process = builder.start();
    } catch (IOException e) {
      log.error("Error starting new container: {}", clazz.getName(), e);
    }

    return null;
  }

  /**
   * Main method to allow starting of a single DIARC configuration from the command line.
   *
   * @param args
   */
  static public final void main(String[] args) {
    String component = System.getProperty("component");
    if (component == null && args.length >= 1) {
      component = args[0];
      args = Arrays.copyOfRange(args, 1, args.length);
    }
    staticLogger.info("Starting Configuration: " + ((component == null) ? "null" : component));

    Class clazz;
    try {
      clazz = Class.forName(component);
    } catch (ClassNotFoundException e) {
      staticLogger.error("Could not find class: " + component, e);
      return;
    }

    // find clazz default constructor
    Constructor<DiarcConfiguration> clazzConstructor = null;
    try {
      Class[] cArgs = new Class[]{};
      clazzConstructor = clazz.getDeclaredConstructor(cArgs);
    } catch (NoSuchMethodException e) {
      staticLogger.error("No default constructor found for class: " + clazz, e);
      System.exit(-1);
    }

    // instantiate clazz instance
    DiarcConfiguration instance = null;
    try {
      instance = clazzConstructor.newInstance();
    } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
      staticLogger.error("Could not instantiate class: " + clazz, e);
      System.exit(-1);
    }

    // call arg parsing methods
    instance.collectAdditionalUsageInfo();
    instance.parseDiarcArgs(args);

    // finally, run the diarc configuration
    instance.runConfiguration();

    // optionally launch gui backend
    if (instance.launchGuiBackend) {
      SpringApplication.run(GuiManager.class);
    }
  }
}
