package edu.tufts.hrilab.example;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;

public class PrintHelloComponent extends DiarcComponent {

  @TRADEService
  public void printHello(String caller) {
    log.info("Hello {}", caller);
  }
}