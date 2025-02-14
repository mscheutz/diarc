/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.asr;

import edu.tufts.hrilab.diarc.DiarcComponent;

import edu.tufts.hrilab.fol.Symbol;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;

public class ASRWebsocketComponent extends DiarcComponent{
  private ASRWebsocketServer server;
  private int port = 8989;
  private String speakerStr = "CommX";
  private String addresseeStr = "self";
  private Symbol speaker;
  private Symbol addressee;
  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("port").hasArg().argName("port").desc("Port to bind the websocket server to (default: " + port + " )").build());
    options.add(Option.builder("speaker").hasArg().argName("name").desc("Set speaker name (default: " + speakerStr + ")").build());
    options.add(Option.builder("addressee").hasArg().argName("name").desc("Set addressee name (default: " + addresseeStr + ")").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("port")) {
      port = Integer.parseInt(cmdLine.getOptionValue("port"));
    }
    log.debug("Websocket server using " + port);

    if (cmdLine.hasOption("speaker")) {
      speakerStr = cmdLine.getOptionValue("speaker");
    }
    speaker = new Symbol(speakerStr);
    log.debug("speaker = " + speakerStr);

    if (cmdLine.hasOption("addressee")) {
      addresseeStr = cmdLine.getOptionValue("addressee");
    }
    addressee = new Symbol(addresseeStr);
    log.debug("listener = " + addresseeStr);
  }

  @Override
  protected void init() {
    try {
      server = new ASRWebsocketServer(port);
    } catch (UnknownHostException ex) {
      log.error("Error creating websocket server", ex);
      return;
    }
    server.setSpeaker(speaker);
    server.setAddressee(addressee);
    server.start();
  }
}