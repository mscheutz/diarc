package edu.tufts.hrilab.example;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.diarc.DiarcComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class CallHelloComponent extends DiarcComponent {

  private TRADEServiceInfo printHelloService;
  private String myName = "world";

  public CallHelloComponent() {
    this.shouldRunExecutionLoop = true;
    this.executionLoopCycleTime = 2000;
  }

  @Override
  protected void executionLoop() {
    if (printHelloService == null) {
      try {
        printHelloService = TRADE.getAvailableService(new TRADEServiceConstraints().name("printHello").argTypes(String.class));
      } catch (TRADEException e) {
        log.error("Could not find service printHello", e);
        return;
      }
    }

    try {
      printHelloService.call(void.class, myName);
    } catch (TRADEException e) {
      log.error("Error calling printHello service", e);
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("name").hasArg().desc("Optional name for hello message").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("name")) {
      myName = cmdLine.getOptionValue("name");
    }
  }

}
