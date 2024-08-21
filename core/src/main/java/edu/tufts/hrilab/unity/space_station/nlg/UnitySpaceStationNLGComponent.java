package edu.tufts.hrilab.unity.space_station.nlg;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class UnitySpaceStationNLGComponent extends DiarcComponent {
  static private Logger log = LoggerFactory.getLogger(UnitySpaceStationNLGComponent.class);

  public UnitySpaceStationNLGComponent() {
    super();
  }

  @TRADEService
  public String spaceStationModifyNLG (String utt) {
    boolean modified = false;
    String[] parts;

    log.debug("spaceStationModifyNLG original: \"" + utt + "\"");

    if (utt.startsWith("I can not repair") && utt.contains("at nothing nothing")) {
      utt = utt.replace("at nothing nothing", "that tube");
      modified = true;
    }

    //I can not repair Tube alpha left One because alpha is not left one nothing damages
    if (utt.startsWith("I can not repair") && utt.contains("nothing damages")) {
      parts = utt.toLowerCase().split(" ");
      utt = String.format("I can not repair tube %s %s %s because it is not damaged", parts[5], parts[6], parts[7]);
      modified = true;
    }

    //I can not repair because I am In transit to nothing
    if (utt.startsWith("I can not repair") && utt.contains("In transit to nothing")) {
      utt = "I can not repair because I am not at a tube";
      modified = true;
    }

    //I can not repair because I am not In nothing
    if (utt.startsWith("I can not repair") && utt.contains("not In nothing")) {
      utt = "I can not repair because I am not at a tube";
      modified = true;
    }

    //I can not repair alpha left four because preempted goals
    if (utt.startsWith("I can not repair") && utt.contains("preempted goals")) {
      utt = utt.replace("preempted goals", "something went wrong");
      modified = true;
    }

    if (modified) {
      log.debug("spaceStationModifyNLG modified: \"" + utt + "\"");
    }
    return utt;
  }
}