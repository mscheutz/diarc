package edu.tufts.hrilab.unity.space_station.nlg;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class UnitySpaceStationNLGComponent extends DiarcComponent {
  static private Logger log = LoggerFactory.getLogger(UnitySpaceStationNLGComponent.class);

  public UnitySpaceStationNLGComponent() {
    super();
  }

  @TRADEService
  public String spaceStationModifyNLG (String utt) {
    log.info("MODIFY: " + utt);
    return utt;
  }
}