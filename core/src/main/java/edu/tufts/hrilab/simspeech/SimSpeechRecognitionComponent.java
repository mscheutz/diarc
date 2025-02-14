/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.simspeech;

import edu.tufts.hrilab.diarc.DiarcComponent;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.*;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.simspeech.gui.SimSpeechRecognitionComponentVis;
import edu.tufts.hrilab.slug.common.Utterance;

import edu.tufts.hrilab.util.resource.Resources;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import ai.thinkingrobots.trade.*;

/**
 * <code>SimSpeechRecognitionComponent</code> provides a keyboard interface
 * to replace a speech recognition component. Takes utterances directly from a
 * text file and sends them to an NLP component.
 */
public class SimSpeechRecognitionComponent extends DiarcComponent {

  protected BufferedReader sbr;
  private Reader reader;
  private List<String> sConfigs = new ArrayList<>();
  private String resourceConfigPath = "config/edu/tufts/hrilab/simspeech";
  public boolean useUnk = false;
  public boolean useCommand = true;
  public boolean toLower = false;
  public boolean useGui = true;
  public Color textColor = Color.BLACK;
  private String input = null; // input from GUI visualizer, typically
  private String output = null; // output saved for getText
  private final Object syncObj = new Object();
  private boolean autoInput = false; //if should automatically process utterances from file
  private int repetitions = 1;  //# times to repeat each utterance during read-from-file mode
  private SimSpeechRecognitionComponentVis gui;
  private Symbol speaker = Factory.createSymbol("brad:agent");
  private Symbol addressee = Factory.createSymbol("self:agent");
  private boolean terminalInput;

  final protected Logger log = LoggerFactory.getLogger(this.getClass());

  /**
   * Constructs the SimSpeechRecognitionComponent
   */
  public SimSpeechRecognitionComponent() {
    super();

    reader = new Reader();
    log.trace("Starting Reader");
    reader.start();
  }

  @Override
  protected void init() {
    try {
      if (autoInput) {
        if (sConfigs.size() != 1) {
          log.error("autoInput not currently compatible with multiple config files. Only using first one.");
        }
        String file = Resources.createFilepath(resourceConfigPath, sConfigs.get(0));
        sbr = new BufferedReader(new InputStreamReader(getClass().getResourceAsStream(file)));
      } else if (terminalInput) {
        sbr = new BufferedReader(new InputStreamReader(System.in));
      }
    } catch (Exception e) {
      log.error("Error getting input stream. Shutting down!", e);
    }
    if (useGui) {
      gui = new SimSpeechRecognitionComponentVis(this);
    }
  }

  /**
   * Get the color specified for text (mostly useful for the visualizer).
   *
   * @return the color text should be displayed in
   */
  public Color getTextColor() {
    return textColor;
  }

  /**
   * Get the configuration file for input buttons (mostly useful for the
   * visualizer).
   *
   * @return the config file to use
   */
  public List<String> getConfigFiles() {
    List<String> files = new ArrayList<>();
    sConfigs.forEach(config -> files.add(Resources.createFilepath(resourceConfigPath, config)));
    return files;
  }

  /**
   * Get the flags relevant to visualization (mostly useful for the visualizer).
   * Examples include whether to display the command box/button and the "unk"
   * button.
   *
   * @return an array of boolean flags
   */
  public boolean[] getVisFlags() {
    return new boolean[]{useCommand, useUnk, toLower};
  }

  /**
   * Set input text (mostly useful for the visualizer).
   *
   * @param in the new speech input
   */
  public void setText(String in) {
    synchronized (syncObj) {
      input = in;
    }
  }

  public void setSpeaker(Symbol speaker) {
    this.speaker = speaker;
    log.info("speaker set to: " + this.speaker);
  }

  public void setAddressee(Symbol addressee) {
    this.addressee = addressee;
    log.info("addressee set to: " + this.addressee);
  }

  // ********************************************************************
  // *** SimSpeech methods
  // ********************************************************************

  /**
   * Get user input from terminal.
   */
  public String getUserInput() {
    String returnVal = null;
    try {
      if (sbr.ready()) {
        returnVal = sbr.readLine();
      }
    } catch (IOException ioe) {
      log.error("error getting input", ioe);
    }
    return returnVal;

  }

  /**
   * The <code>Reader</code> is the main loop for the component when in non-gui
   * mode.
   */
  private class Reader extends Thread {

    boolean shouldRead;

    public Reader() {
      shouldRead = true;
    }

    @Override
    public void run() {
      String prompt = "> ";
      while (shouldRead) {
        String in = null;
        if (sbr != null) {
          in = getUserInput();
          log.trace(String.format("input: %s", in));
        }
        synchronized (syncObj) {
          if (in == null) { // no terminal/file input
            // grab what might have come in from the gui and reset that
            in = input;
            input = null;
          }
          if (in != null) {
            // new input, make it available to getText
            output = in;
            prompt = "> ";
          } else {
            prompt = null;
          }
        }
        if (in != null) {
          log.debug("Adding utterance: " + in);
          Utterance.Builder utterance = new Utterance.Builder()
                  .setWords(Arrays.asList(in.split(" ")))
                  .setSpeaker(speaker)
                  .setAddressee(addressee)
                  .setIsInputUtterance(true);

          try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class))
                    .call(void.class,utterance.build());
          } catch (TRADEException e) {
            log.error("reportRecognizedSpeech failed.", e);
          }
        }
        Thread.yield();
      }
      log.trace("Exiting Reader thread...");
    }

    public void halt() {
      shouldRead = false;
    }
  }

  /**
   * Provide additional information for usage...
   */
  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("cfg").longOpt("config").hasArgs().argName("file(s)").desc("load buttons from file(s)").build());
    options.add(Option.builder("nocommand").desc("do not include command entry field").build());
    options.add(Option.builder("unk").desc("include unk button").build());
    options.add(Option.builder("textred").desc("red button text").build());
    options.add(Option.builder("textblue").desc("blue button text").build());
    options.add(Option.builder("auto").longOpt("autoInput").desc("automatically run utterances from file").build());
    options.add(Option.builder("reps").longOpt("repetitions").hasArg().argName("#").desc("# of times to repeat each utterance").build());
    options.add(Option.builder("speaker").hasArg().argName("name").desc("set speaker name").build());
    options.add(Option.builder("addressee").hasArg().argName("name").desc("set default addressee name").build());
    options.add(Option.builder("terminal").desc("get input from terminal").build());
    options.add(Option.builder("nogui").desc("run the simspeech component without a gui").build());
    return options;
  }

  /**
   * Parse additional command-line arguments
   */
  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("cfg")) {
      sConfigs = Arrays.stream(cmdLine.getOptionValues("cfg")).toList();
    }
    if (cmdLine.hasOption("nocommand")) {
      useCommand = false;
    }
    if (cmdLine.hasOption("unk")) {
      useUnk = true;
    }
    if (cmdLine.hasOption("textred")) {
      textColor = Color.RED;
    }
    if (cmdLine.hasOption("textblue")) {
      textColor = Color.BLUE;
    }
    if (cmdLine.hasOption("auto")) {
      autoInput = true;
    }
    if (cmdLine.hasOption("reps")) {
      repetitions = Integer.parseInt(cmdLine.getOptionValue("reps"));
    }
    if (cmdLine.hasOption("speaker")) {
      speaker = Factory.createSymbol(cmdLine.getOptionValue("speaker"));
    }
    if (cmdLine.hasOption("addressee")) {
      addressee = Factory.createSymbol(cmdLine.getOptionValue("addressee"));
    }
    if (cmdLine.hasOption("terminal")) {
      terminalInput = true;
    }
    if (cmdLine.hasOption("nogui")) {
      useGui = false;
    }
  }

  @Override
  public void shutdownComponent() {

    reader.halt();
    try {
      reader.join();
    } catch (InterruptedException e) {
      log.error("[shutdown]");
    }
    if (useGui) {
      gui.shutdown();
    }
  }
}
