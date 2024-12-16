/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.slug.parsing.patternMatching;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.interfaces.NLUInterface;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class SpaceStationPatternMatcher extends DiarcComponent implements NLUInterface {
  /**
   * @param u
   * @return
   */
  @TRADEService
  @Action
  public Utterance parseUtterance(Utterance u) {
    // try parsing it
    try {
      return extractInstruction(u);
    } catch (Exception e) {
      log.error("PATTERN MATCHER: problem with " + u.getWordsAsString(), e);
    }
    log.info("PROBLEM: no match found for {}", u);
    return null;
  }
    /* These are the proper commands:
      “Robot 1, go to Alpha”                       INSTRUCT(player1,robot1,goToArea(robot1,beta),{})
      “Robot 1, check for broken tubes in Alpha”   INSTRUCT(player1,robot1,check(robot1,alpha),{})
      “Robot 2, go to Beta Right 10”               INSTRUCT(player1,robot1,goToTube(robot2,beta,leftFour),{})
      “Robot 2, repair Beta Right 10”              INSTRUCT(player1,robot1,repairTube(robot2,beta,rightTen),{})
    */

  // this does all the work
  // TODO: we could return partial utterance to the extent that we've extracted them for further processing and troubleshooting
  private Utterance extractInstruction(Utterance u) {
    // Remove disfluencies and politeness forms
    log.info("Instruction: " + u.getWordsAsString());

    String sentence = u.getWordsAsString().replaceAll("\\b(?:a|hey|please|um|uh|ah|oh|or|i|red|that|'m|the)\\b", "").trim();
    String cleanedSentence = sentence.replaceAll(",", "").replaceAll("1", "one").replaceAll("2", "two").trim();

    // Define patterns to match the robot addressee with homophones ("robots" will be matched to "robot 1")
    Pattern robotone = Pattern.compile("\\b(?:robots|robots one|rubber one|robot one|row bot one|rowboat one|rubber won|robot won|row bot won|rowboat won)\\b", Pattern.CASE_INSENSITIVE);

    Pattern robottwo = Pattern.compile("\\b(?:robots two|robots to|robot two|robot to|rubber two|rubber to|robot too|row bot two|row bot too|row bot to|rowboat two|rowboat too|rowboat to)\\b", Pattern.CASE_INSENSITIVE);

    // Create matchers for the cleaned sentence
    Matcher robotonematcher = robotone.matcher(cleanedSentence);
    Matcher robottwomatcher = robottwo.matcher(cleanedSentence);

    String cleanedRemaining;

    // check for the robot addressee -- must always be part of a command
    String robot;
    if (robotonematcher.find()) {
      robot = "robot1";
    } else if (robottwomatcher.find()) {
      robot = "robot2";
    } else {
      log.error("NO ROBOT ADDRESSED");
      return null;
    }

    // MS: do we need to pass u in for some default values?
    Utterance.Builder builder = new Utterance.Builder(u);
    // set the addressee
    builder.setAddressee(new Symbol(robot));
    // assume "INSTRUCT"
    builder.setUtteranceType(UtteranceType.INSTRUCT);

    // get rid of the address
    cleanedRemaining = cleanedSentence.replaceAll("\\b(?:robots|rubber one|robot one|row bot one|rowboat one|robot two|rubber two|robot too|row bot two|row bot too|rowboat two|rowboat too|robot won|row bot won|rowboat won|robot to|row bot to|rowboat to)\\b", "").trim();

    log.info("Cleaned instruction: " + cleanedRemaining);

    // check for area reference
    String area = "";
    int areaindex;
    boolean noarea = false;
    if ((areaindex = cleanedRemaining.indexOf("alpha")) > -1) {
      area = "alpha";
    } else if ((areaindex = cleanedRemaining.indexOf("beta")) > -1) {
      area = "beta";
    } else if ((areaindex = cleanedRemaining.indexOf("gamma")) > -1 || (areaindex = cleanedRemaining.indexOf("gumroad")) > -1
            || (areaindex = cleanedRemaining.indexOf("gama")) > -1) {
      area = "gamma";
    } else {
      noarea = true;
    }

    // check for tube reference
    String side = "", number = "";
    int tubeindex;
    boolean notube = false;
    if ((tubeindex = cleanedRemaining.indexOf("left")) > -1 || (tubeindex = cleanedRemaining.indexOf("les")) > -1 ||
            (tubeindex = cleanedRemaining.indexOf("lasts")) > -1) {
      side = "left";
      // get the number
      String[] items = cleanedRemaining.substring(tubeindex + 4).trim().split("\\s+");
      // capitalize first letter
      number = items[0].substring(0, 1).toUpperCase() + items[0].substring(1);
    } else if ((tubeindex = cleanedRemaining.indexOf("right")) > -1 || (tubeindex = cleanedRemaining.indexOf("gumroad")) > -1) {
      side = "right";
      // get the number
      String[] items = cleanedRemaining.substring(tubeindex + 5).trim().split("\\s+");
      // capitalize first letter
      number = items[0].substring(0, 1).toUpperCase() + items[0].substring(1);
    } else {
      notube = true;
    }

    log.info("Area: " + area + "; Tube: " + side + number);


    // check area for broken tubes
    if (cleanedRemaining.matches("(monitor|check|look in|observe)(.*)")) {
      if (noarea) {
        log.error("NO AREA specified");
        return null;
      } else {
        //return "---: Robot " + robot + ", check area " + area + " for broken tubes";
        builder.setSemantics(Factory.createPredicate("check(" + robot + "," + area + ")"));
        // return the updated utterance
        return builder.build();
      }
    }
    // now check for movement to area or tubes
    else if (cleanedRemaining.matches("(go|drive|move|come|get)(.*)")) {
      // if we don't have tube information, we need the are
      if (notube) {
        // if we don't have area info
        if (noarea) {
          log.error("NO AREA or TUBE specified for location");
          return null;
        } else {
          builder.setSemantics(Factory.createPredicate("goToArea(" + robot + "," + area + ")"));
        }
      } else {
        // we have a tube, check if we don't have an area, go to current
        if (noarea) {
          builder.setSemantics(Factory.createPredicate("goToTube(" + robot + "," + side + number + ")"));
        } else {
          builder.setSemantics(Factory.createPredicate("goToTube(" + robot + "," + area + "," + side + number + ")"));
        }
      }
    }
    // check for repair, need the exact tube info
    else if (cleanedRemaining.matches("(repair|re pair|fix)(.*)")) {
      if (notube) {
        log.error("NO TUBE specified for repair");
        return null;
      } else if (noarea) {
        if (notube) {
          builder.setSemantics(Factory.createPredicate("repairTube(" + robot + ")"));
        } else {
          builder.setSemantics(Factory.createPredicate("repairTube(" + robot + "," + side + number + ")"));
        }
      } else {
        if (notube) {
          log.error("NO TUBE specified for repair in area " + area);
          return null;
        } else {
          builder.setSemantics(Factory.createPredicate("repairTube(" + robot + "," + area + "," + side + number + ")"));
        }
      }
    }
    // this is an unknown command
    else {
      log.error("MATCHES: " + cleanedRemaining.matches("(go|drive|move|come|get)(.*)"));
      log.error("NO KNOWN COMMAND specified for robot " + robot);
      return null;
    }
    // return the updated utterance
    return builder.build();
  }

    /*
    // this is for testing only
    public static void main(String[] args) {
	String[][] instructions = 
{{"robots go to gumroad six","Robot, go to gamma right six"},{"alpha won go to gumroad six","Robot 1, come to gamma right six"},{"robot one go to gamma right six","Robot 1, come to gamma right six"},{"rubber one repair gumroad six","Robot 1, repair gamma right six"},{"robot one go to youtube gamma left eight","Robot 1 go to tube gamma left eight"},{"robot one repair tube gamma left eight","Robot 1 repair tube gamma left eight"},{"the robot one repair tube gamma left eight","Robot 1 repair tube gamma left eight"},{"yeah repair tube","Repair tube, robot 1"},{"alpha alpha one go to youtube barrel alpha","Robot, robot 1 go to tube beta left four"},{"alpha won't go to tube beta left","Robot 1 go to tube beta left four"},{"robot one repair tube beta right four","Robot 1 repair tube beta left four"},{"robot one repair tube beta left","Robot 1 repair tube beta left four"},{"alpha one repair tube beta left four","Robot 1 repair tube beta left four"},{"robot one check four broken tubes in alpha","Robot 1 check for broken tubes in alpha"},{"alpha won several broken tubes in gamma","Robot 1 check for broken tubes in gamma"},{"robot one check a beta four broken tubes","Robot 1 check beta for broken tubes"},{"robot two check gamma left four broken tubes","Robot 2 check gamma for broken tubes"},{"oh robot two stop","Robot 2 stop"},{"robot two check wing bail four broken toots","Robot 2 check wing beta for broken tubes"},{"robot two repair tube an alpha left","Robot 2 repair tube alpha left two"},{"robot robot two repair alpha left three","Robot 2 repair tube alpha left three"},{"repair get alpha less four","Robot 2 repair ga, alpha left four"},{"robot two repair bit alpha right now","Robot 2 go to tube alpha right nine"},{"the robot two repair alpha right nine","Robot 2 go to alpha right nine"},{"robot one go to","Robot 1 go to"},{"robot one go to gamma rate six","Robot 1 go to gamma right six"},{"robot two go to beta","Robot 2 go to beta"},{"robot one go to alpha right six note go to","Robot 1 go to alpha right six, oh, go to"},{"robot one go to gamma rate six","Robot 1 go to gamma right six"},{"robot one goto gamma rate six","Robot 1 go to gamma right six"},{"robot two go to alpha","Robot 2 go to alpha"},{"robot one go to gamma right six","Robot 1 go to gamma right six"},{"robot one repair tube","Robot 1 repair tube"},{"robot one go to tube gamma left eight","Robot 1 go to tube gamma left eight"},{"robot two","Robot 2"},{"go to beta left three","go to beta left three"},{"well robot two go to alpha left three","Robot 2 go to alpha left three"},{"robot two repair tube","Robot 2 repair"},{"robot one go to beta","Robot 1 go to beta"},{"robot one go to gamma rate six","Robot 1 go to gamma right six"},{"robot one repair gamma rate six","Robot 1 repair gamma right six"},{"robot one repair gamma","Robot 1 repair gamma"},{"robot one repair gamma left eight","robot 1 repair gamma left eight"},{"robot two go to wing alpha","Robot 2 go to wing alpha"},{"robot one go to beta","Robot 1 go to beta"},{"the robot one repair tube beta rate twelve","Robot 1 repair tube beta right twelve"},{"robot two go to beta right two","Robot 2 go to beta right two"},{"robot two get a beta left two","Robot 2 go to beta left two"},{"hmm robot two go to beta left three","Robot 2 go to beta left three"},{"robot two repair beta left three","Robot 2 repair beta left three"},{"robot two go to alpha left eh","Robot 2 go to alpha left eight"},{"robot two repair alpha left three","Robot 2 repair alpha left three"},{"Robot 2 repair alpha left three","Robot 2 repair alpha left three"},{"robot two go to beta left four","Robot 2 go to beta left four"},{"robot one go to l","Robot 1 go to alpha"},{"robot two go to gamma right six","Robot 2 go to gamma right six"},{"robot one go to alpha","Robot 1 go to alpha"},{"robot two go to gamma right sick","Robot 2 go to gamma right six"},{"robot two repair gamma right","Robot 2 repair gamma right six"},{"robot two go to gamma right sex","Robot 2 go to gamma right six"},{"robot two go to gamma right six","Robot 2 go to gamma right six"},{"the robot two repair gamma right six","Robot 2 repair gamma right six"},{"robot one go to beta","Robot 1 go to beta"},{"robot two go to alpha left two","Robot 2 go to alpha left three"},{"robot one repair beta left actually","Robot 1 repair beta left, actually"},{"robot one repair beta left three","Robot 1 repair beta left three"},{"robot two go to alpha","Robot 2 go to alpha"},{"robot two repair","Robot 2 repair"},{"robot two go to beta","Robot 2 go to beta"},{"robot one go to beta left three","Robot 1 go to beta left three"},{"a robot one repair gamma right six","Robot 1 repair gamma right six"},{"robot two repair beta left three","Robot 2 repair beta left three"},{"i robot one repair gamma left eight","Robot 1 repair gamma left eight"},{"robot one repair alpha left three","Robot 1 repair alpha left three"},{"a robot two repair alpha right nine","Ahh, robot 2 repair tube alpha right nine"},{"robot two repair alpha right nine","Robot 2 repair alpha right nine"},{"i robot one repair alpha left three","Robot 1 repair alpha left three"},{"robot one good at gamma","Robot 1 go to gamma"},{"robot wine repair gamma rate six","Robot 1 repair gamma right six"},{"robot one repair gamma right six","Robot 1 repair gamma right six"},{"robot two go to beta","Robot 2 go to beta"},{"robot one repair gamma right sec","Robot 1 repair gamma right six"},{"robot two repair","Robot 2, repair"},{"robot two repair beta les three","Robot 2, repair beta left three"},{"robot two go to beta","Robot 2 go to beta"},{"robot two repair beta left three","Robot 2 repair beta left three"},{"robot two go to l","Robot two go to alpha"},{"robot two go to alpha","Robot two go to alpha"},{"robot one and go to gamma","Robot one go to gamma"},{"robot one repair gamma left three","Robot 1 repair gamma left three"},{"robot one repair alpha left three","Robot 1 repair alpha left three"},{"robot two go to beta","Okay, robot 2 go to beta"},{"robot two repair tube left","Robot 2 repair tube left three"},{"the robot two repair left three","Robot 2 repair tube left three"},{"the robot two repair tube left three","Robot 2 repair tube left three"},{"robot two go to alpha","Okay robot 2 go to alpha"},{"wrote robot two repair left","Robot 2 repair left three"},{"robot two repair left three","Robot 2 repair left three"},{"robot two repair alpha left three","Robot 2 repair alpha left three"},{"robot one go to gamma","robot one go to gamma"},{"robot one repair gamma right six","Robot 1 repair gamma right six"},{"robot repair gamma left a","Robot 1 repair gamma left eight"},{"robot one repair gamma left","Robot 1 repair gamma left four"},{"robot one repair damage left forward","Robot 1 repair gamma left four"},{"robot one repair gamma left four","Robot 1 repair gamma left four"},{"the robot one gamma right sex","Robot 1, gamma right six"},{"robot repair gamma right six","Robot 1 repair gamma right six"},{"a robot two repair beta right two","Uh robot 2 repair beta right two"},{"the robot two repair beta left three","Robot 2 repair beta left three"},{"robot one go to gamma","Robot 1 go to gamma"},{"robot two go to alpha left three","Robot 2 go to alpha left three"},{"robot one repair","Robot 1 repair gamma left six"},{"robot one repair gamma left eight","Robot 1 repair gamma left eight"},{"robot one repair fatal left four","Robot 1 repair beta left four"},{"the robot one repair beta left four","Robot 1 repair beta left four"},{"robot two go to gamma","Robot 2 go to gamma"},{"left repair left four","repair left four"},{"robot two repair gama left four","Robot 2 repair gamma left four"},{"the robot two repair gama left four","Robot 2 repair gamma left four"},{"gama left four fudge","I said gamma left four ah fudge"},{"the robot two repair gama left boar","Robot 2 repair gamma left four"},{"robot one go to two gamma right six","Robot 1 go to tube gamma right six"},{"robot one re pair tube gamma right six","Robot 1 repair tube gamma right six"},{"robot two go to to beta right six","Robot 2 go to tube beta right six"},{"robot two go to to beta left three","Robot 2 go to tube beta left three"},{"robot two repair beta left three","Repair beta left three"},{"robot one go to tube alpha left three","Robot 1 go to tube alpha left three"},{"robot one repair tube alpha left three","Robot 1 repair tube alpha left three"},{"the robot two go to gamma laugh eight","Robot 2 go to tube gamma left eight"},{"robot two go to beta right twelve","Robot 2 go to beta right twelve"},{"robot one go to alpha are right now","Robot 1 go to alpha R right nine"},{"robot one go to alpha nine","Robot 1 go to alpha R right nine"},{"mobile won't go to alpha right nine","Robot 1 go to alpha R right nine"},{"robot one repair alpha bright nine","Robot 1 repair alpha right nine"},{"robot two go to gamma left four","Robot 2 go to gamma left four"},{"we're about to go to gamma left four","Robot 2 go to gamma left four"},{"robot two repair gamma lasts four","Robot 2 repair gamma left four"},{"robot won't go to beta","Robot 1 go to beta"},{"robot one go to beta left ten","Robot 1 go to beta left ten"},{"robot two go to tube gamma left two","Robot 2 go to tube gamma left two"},{"robot two go to tube beta left four","Robot 2 come to tube beta left four"},{"robot two picks beta left four","Robot 2 fix tube beta left four"},{"robot two fix alpha left four i mean fix beta left four","Robot 2 fix alpha left I mean fix beta left four"},{"robot two fix beta left four","Robot 2 fix beta left four"},{"robot two fix beta left ten","Robot 2 fix tube beta left ten"},{"robot one go to gamma right right six","Robot 1 go to gamma right six"},{"about one six two","Robot 1 fix tube"},{"robot two go to gamma le","Robot 2 go to gamma"},{"robot two go to beta left three","Robot 2 go to beta left three"},{"robot two fix too","Robot 1 go to alpha left three"},{"robot one go to alpha left three","Robot 1 go to gamma"},{"robot one go to alpha right now","Robot 1 go to alpha"},{"one go to gamma left four","go to gamma left four"},{"robot one go to gamma left four","robot one go to gamma left four"},{"robot one fixed too","Robot 1 Fix tube"},{"robot two go to gamma left eight","Robot 2 come to gamma left eight"},{"the robot two repair gamma left eight","Robot 2 repair gamma left eight"},{"roboto go to alpha left three","Robot 2 come to alpha left three"},{"robot two go to gamarra leftward","Robot 2 come to gamma left four"},{"the robot two go to alpha right two","Robot 2 come to alpha right two"},{"robot one or go to two beta left","Robot 1 go to beta left"},{"reward one rebel beta left four","Robot one repair beta left four"},{"robot wonder bear beta too","Robot 1 repair beta 2 right"},{"joe baldwin rebel beta elford","Robot 1 repair beta L4"},{"the board one go to bitter elford","Robot 1 repair beta L4"},{"alpha alpha one go to bedale photo","Robot 1 come to beta left 4"},{"alpha one grip go to beta elford","Robot 1 come to beta left 4"},{"the board one repair beta edit four","Robot 1 repair beta L4"},{"robot the one repair beta enforce","Robot 1 repair beta L4"},{"alpha won't go to beta","Robot 1 go to beta"},{"robots to go to alpha fire","Robot 2 Go to alpha"},{"robot one repair gamma right six","Robot 1 Repair Gamma Right Six"},{"robot two repair beta left","Robot 2 Repair beta left three"},{"robot two repair beta left three","Robot 2 Repair beta left three"},{"robot one repair gamarra right sex","Robot 2 repair gamma right six"},{"repair bay to left three you are there","But you're at beta left three you are there"},{"i'm robot one repair gamma right six","Um Robot 1 repair gamma right six"},{"robot two repair beta les three","Robot 2 Repair beta left three"},{"robot one repair or beta gamma right six","Robot 1 Repair Beta Gamma Right Six"},{"the robot one that repair gamma right six","Robot 1 Repair Gamma Right Six"},{"robot two go to alpha","Robot 2 go to alpha"},{"robot two repair alpha left three","Robot 2 repair alpha left three"},{"robot one go to beta","Robot 1 go to beta"},{"robot one repair beta right twelve","Robot 1 repair beta right twelve"},{"robot two repair beta left four","Robot 2 repair beta left four"},{"red robot one repair beta right twelve","Robot 1 repair beta right twelve"},{"the robot two repair beta left four","Robot 2 repair beta left four"},{"robot one repair beta left ten","Robot 1 repair beta left ten"},{"robot two go to alpha","Robot 2 go to alpha"},{"robot one go to gamma","Robot 1 go to gamma"},{"robot one repair gamma right two","Robot 1 repair gamma right two"},{"i robot two repair alpha right nine","Robot 2 repair alpha right nine"},{"i robot one fix gamma right six","Robot 1 fix gamma right six"},{"robot two fix beta left three","Robot 2 fix beta left three"},{"robot two repair gamma left eight","Robot 2 repair gamma left eight"},{"robot one repair alpha left three","Robot 1 repair alpha left three"},{"robot one repair alpha left three","Robot 1 repair alpha left three"},{"robot one repair alpha right nine","Robot 1 repair alpha right nine"},{"robot two repair","Robot 2 repair beta left, beta right four"},{"robot two repair beta right twelve","Robot 2 repair beta right twelve"},{"robot one repair gamma left four","Robot 1 repair gamma left four"},{"robot one repair left robot one repair beta left four","Robot repair alpha, repair one beta left four"},{"robot two repair beta left ten","Robot 2 repair beta left ten"}};
	
        for (String instruction[] : instructions) {
	    String parsed = extractInstruction(instruction[0]);
	    if (parsed.equals("")) {
		log.info("Original: " + instruction[0] + "  Intended: " + instruction[1]);
	    }
        }
    }
    */
}
