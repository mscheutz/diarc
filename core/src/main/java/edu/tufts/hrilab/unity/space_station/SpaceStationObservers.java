package edu.tufts.hrilab.unity.space_station;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Condition;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.fol.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SpaceStationObservers {
  private static Logger log = LoggerFactory.getLogger(SpaceStationObservers.class);

  public void registerWithTRADE(List<String> groups) {
    try {
      TRADE.registerAllServices(this, groups);
    } catch (TRADEException e) {
      log.error("Error registering TRADE Services", e);
    }
  }

  @TRADEService
  @Action
  @Condition(
          condition={"amIn(?actor,!area)"},
          type= ConditionType.PRE
  )
  @Observes({"propertyOf(tube(!area,!side,!number),damaged(!amount))"})
  public List<Map<Variable, Symbol>> observeTubesDamaged(Symbol actor, Term state) throws TRADEException {
    // get info about requested state
    Term stateTube = (Term) state.get(0);
    Symbol stateArea = stateTube.get(0);
    Symbol stateSide = stateTube.get(1);
    Symbol stateNum = stateTube.get(2);
    Symbol stateAmount = ((Term)state.get(1)).get(0);

    // clear old beliefs
    TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief").argTypes(Term.class))
            .call(void.class, Factory.createPredicate("propertyOf(tube(A,S,N),damaged(AMOUNT))"));

    // get current area, and only get results from that area
    Symbol currentArea = getCurrentArea(actor);
    if (currentArea == null) {
      return null;
    }
    List<Symbol> damagedTubes = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubesDamaged").argTypes(Symbol.class))
            .call(List.class, currentArea);

    List<Map<Variable,Symbol>> results = getRelevantTubes(state, damagedTubes);

    // add damage amount to each damaged tube result
    TRADEServiceInfo tubeHealthService = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubeHealth").argTypes(Symbol.class));
    for (Map<Variable,Symbol> tube : results) {
      Symbol area = stateArea.isVariable() ? tube.get(stateArea) : stateArea;
      Symbol side = stateSide.isVariable() ? tube.get(stateSide) : stateSide;
      Symbol num = stateNum.isVariable() ? tube.get(stateNum) : stateNum;
      Symbol tubeName = TubeNameHelpers.toFullTube(area, side, num);
      float health = tubeHealthService.call(Float.class, tubeName);
      float damage = 100 - health;

      // assume amount is always a free-variable
      tube.put((Variable) stateAmount, Factory.createSymbol(Float.toString(damage)));
    }

    return results;
  }

  @TRADEService
  @Action
  @Condition(
          condition={"amIn(?actor,!area)"},
          type= ConditionType.PRE
  )
  @Observes({"propertyOf(tube(!area,!side,!number),broken)"})
  public List<Map<Variable, Symbol>> observeTubesBroken(Symbol actor, Term state) throws TRADEException {

    // clear old beliefs
    // NOTE: technically broken tube facts don't need to be cleared because once a tube is broken, bc it stays broken forever,
    //       but this will keep things consistent with damaged and off tubes, and more intuitive for users
    TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief").argTypes(Term.class))
            .call(void.class, Factory.createPredicate("propertyOf(tube(A,S,N),broken)"));

    // get current area, and only get results from that area
    Symbol currentArea = getCurrentArea(actor);
    if (currentArea == null) {
      return null;
    }
    List<Symbol> brokenTubes = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubesBroken").argTypes(Symbol.class))
            .call(List.class, currentArea);

    return getRelevantTubes(state, brokenTubes);
  }

  @TRADEService
  @Action
  @Condition(
          condition={"amIn(?actor,!area)"},
          type= ConditionType.PRE
  )
  @Observes({"propertyOf(tube(!area,!side,!number),off)"})
  public List<Map<Variable, Symbol>> observeTubesOff(Symbol actor, Term state) throws TRADEException {

    // clear old beliefs
    TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief").argTypes(Term.class))
            .call(void.class, Factory.createPredicate("propertyOf(tube(A,S,N),off)"));

    // get current area, and only get results from that area
    Symbol currentArea = getCurrentArea(actor);
    if (currentArea == null) {
      return null;
    }
    List<Symbol> tubesOff = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubesOff").argTypes(Symbol.class))
            .call(List.class, currentArea);

    return getRelevantTubes(state, tubesOff);
  }


  /**
   * Filter relevant tubes based on queried state.
   *
   * @param state
   * @param tubes
   * @return
   */
  private List<Map<Variable, Symbol>> getRelevantTubes(Term state, List<Symbol> tubes) {
    List<Map<Variable,Symbol>> results = new ArrayList<>();
    if (!tubes.isEmpty()) {
      Term stateTube = (Term) state.get(0);
      Symbol stateArea = stateTube.get(0);
      Symbol stateSide = stateTube.get(1);
      Symbol stateNum = stateTube.get(2);


      for (Symbol tube : tubes) {
        List<Symbol> tubeParts = TubeNameHelpers.fromFullTube(tube);
        if (!stateArea.isVariable() && !stateArea.equals(tubeParts.get(0))) {
          // only care about tubes in state's area, ignore the rest
          continue;
        }

        if (!stateSide.isVariable() && !stateSide.equals(tubeParts.get(1))) {
          // only care about tubes in state's area and side, ignore the rest
          continue;
        }

        if (!stateNum.isVariable() && !stateNum.equals(tubeParts.get(2))) {
          // only care about tubes in state's area and side and number, ignore the rest
          continue;
        }

        // we care about this tube -- populate map with free variable values
        Map<Variable,Symbol> solution = new HashMap<>();
        if (stateArea.isVariable()) {
          solution.put((Variable) stateArea, tubeParts.get(0));
        }
        if (stateSide.isVariable()) {
          solution.put((Variable) stateSide, tubeParts.get(1));
        }
        if (stateNum.isVariable()) {
          solution.put((Variable) stateNum, tubeParts.get(2));
        }

        results.add(solution);
      }
    }

    return results;
  }

  /**
   * Helper method to get actor's current area.
   * @param actor
   * @return
   * @throws TRADEException
   */
  private Symbol getCurrentArea(Symbol actor) throws TRADEException {

    Variable areaVar = Factory.createVariable("A");
    Predicate query = Factory.createPredicate("amIn", actor, areaVar);

    List<Map<Variable, Symbol>> bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Term.class))
            .call(List.class, query);

    Symbol result = null;
    if (!bindings.isEmpty()) {
      result = bindings.get(0).get(areaVar);
    }

    return result;
  }
}
