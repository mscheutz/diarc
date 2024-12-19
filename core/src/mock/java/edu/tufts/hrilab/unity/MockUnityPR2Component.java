/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.unity.space_station.TubeNameHelpers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MockUnityPR2Component extends DiarcComponent implements UnityPR2Interface {

  @Override
  public Symbol getPR2Area() {
    return null;
  }

  @Override
  public boolean startPR2Repair(Symbol target) {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("repairMockTube").argTypes(Symbol.class))
              .call(Boolean.class, target);
    } catch (TRADEException e) {
      log.error("Error calling repairMockTube.", e);
      return false;
    }

    return true;
  }

  @Override
  public boolean stopPR2Repair(Symbol target) {
    return true;
  }


  //////////////////////////////////////////////////////////
  // Space Station Observers
  //////////////////////////////////////////////////////////

  @TRADEService
  @Observes({"propertyOf(TUBE,damaged(AMOUNT))"})
  public List<Map<Variable, Symbol>> observeTubesDamaged2(Term state) throws TRADEException {
    List<Map<Variable, Symbol>> tubesDamaged = observeTubesDamaged(Factory.createPredicate("propertyOf(AREA,SIDE,NUMBER,damaged(AMOUNT))"));
    Variable area = Factory.createVariable("AREA");
    Variable side = Factory.createVariable("SIDE");
    Variable num = Factory.createVariable("NUMBER");
    Variable amount = Factory.createVariable("AMOUNT");

    List<Map<Variable,Symbol>> results = new ArrayList<>();
    Symbol stateTubeName = state.get(0);
    Symbol stateAmount = ((Term)state.get(1)).get(0);
    for (Map<Variable, Symbol> tube : tubesDamaged) {
      Symbol fullTubeName = TubeNameHelpers.toFullTube(tube.get(area), tube.get(side), tube.get(num));

      Map<Variable,Symbol> solution = new HashMap<>();
      solution.put((Variable) stateTubeName, fullTubeName);
      solution.put((Variable) stateAmount, tube.get(amount));
      results.add(solution);
    }

    return results;
  }

  @TRADEService
  @Observes({"propertyOf(TUBE,broken)"})
  public List<Map<Variable, Symbol>> observeTubesBroken2(Term state) throws TRADEException {
    List<Map<Variable, Symbol>> tubesBroken = observeTubesBroken(Factory.createPredicate("propertyOf(AREA,SIDE,NUMBER,broken)"));
    Variable area = Factory.createVariable("AREA");
    Variable side = Factory.createVariable("SIDE");
    Variable num = Factory.createVariable("NUMBER");

    List<Map<Variable,Symbol>> results = new ArrayList<>();
    Symbol stateTubeName = state.get(0);
    for (Map<Variable, Symbol> tube : tubesBroken) {
      Map<Variable,Symbol> solution = new HashMap<>();
      Symbol fullTubeName = TubeNameHelpers.toFullTube(tube.get(area), tube.get(side), tube.get(num));
      solution.put((Variable) stateTubeName, fullTubeName);
      results.add(solution);
    }

    return results;
  }


  @TRADEService
  @Observes({"propertyOf(TUBE,off)"})
  public List<Map<Variable, Symbol>> observeTubesOff2(Term state) throws TRADEException {
    List<Map<Variable, Symbol>> tubesBroken = observeTubesOff(Factory.createPredicate("propertyOf(AREA,SIDE,NUMBER,off)"));
    Variable area = Factory.createVariable("AREA");
    Variable side = Factory.createVariable("SIDE");
    Variable num = Factory.createVariable("NUMBER");

    List<Map<Variable,Symbol>> results = new ArrayList<>();
    Symbol stateTubeName = state.get(0);
    for (Map<Variable, Symbol> tube : tubesBroken) {
      Map<Variable,Symbol> solution = new HashMap<>();
      Symbol fullTubeName = TubeNameHelpers.toFullTube(tube.get(area), tube.get(side), tube.get(num));
      solution.put((Variable) stateTubeName, fullTubeName);
      results.add(solution);
    }

    return results;
  }

  @TRADEService
  @Observes({"propertyOf(AREA,SIDE,NUMBER,damaged(AMOUNT))"})
  public List<Map<Variable, Symbol>> observeTubesDamaged(Term state) throws TRADEException {
    List<Symbol> damagedTubes = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubesDamaged").argTypes())
            .call(List.class);

    List<Map<Variable,Symbol>> results = getRelevantTubes(state, damagedTubes);

    // add damage amount to each damaged tube result
    Symbol stateArea = state.get(0);
    Symbol stateSide = state.get(1);
    Symbol stateNum = state.get(2);
    Symbol stateAmount = ((Term)state.get(3)).get(0);
    TRADEServiceInfo tubeHealthService = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubeHealth").argTypes(Symbol.class));
    for (Map<Variable,Symbol> tube : results) {
      Symbol tubeName = TubeNameHelpers.toFullTube(tube.get(stateArea), tube.get(stateSide), tube.get(stateNum));
      float health = tubeHealthService.call(Float.class, tubeName);
      float damage = 100 - health;

      // assume amount is always a free-variable
      tube.put((Variable) stateAmount, Factory.createSymbol(Float.toString(damage)));
    }

    return results;
  }

  @TRADEService
  @Observes({"propertyOf(AREA,SIDE,NUMBER,broken)"})
  public List<Map<Variable, Symbol>> observeTubesBroken(Term state) throws TRADEException {
    List<Symbol> brokenTubes = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubesBroken").argTypes())
            .call(List.class);

    return getRelevantTubes(state, brokenTubes);
  }

  @TRADEService
  @Observes({"propertyOf(AREA,SIDE,NUMBER,off)"})
  public List<Map<Variable, Symbol>> observeTubesOff(Term state) throws TRADEException {
    List<Symbol> tubesOff = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSpaceStationTubesOff").argTypes())
            .call(List.class);

    return getRelevantTubes(state, tubesOff);
  }

  private List<Map<Variable, Symbol>> getRelevantTubes(Term state, List<Symbol> tubes) {
    List<Map<Variable,Symbol>> results = new ArrayList<>();
    if (!tubes.isEmpty()) {
      Symbol stateArea = state.get(0);
      Symbol stateSide = state.get(1);
      Symbol stateNum = state.get(2);


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
}
