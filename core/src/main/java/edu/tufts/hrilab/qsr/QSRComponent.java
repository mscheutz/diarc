package edu.tufts.hrilab.qsr;

import ai.thinkingrobots.trade.*;

import edu.tufts.hrilab.consultant.pose.PoseConsultant;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;

import javax.vecmath.Point3d;
import java.util.*;

public class QSRComponent extends DiarcComponent {
//    private final SPARQ sparq;

  PoseConsultant consultant;

  public QSRComponent() {
    super();
    consultant = new PoseConsultant(PoseReference.class, "location", initializeProperties());
  }

  private List<String> initializeProperties() {
    List<String> properties = new ArrayList<>();

    // Add all the ePRAm properties to the consultant
    Arrays.stream(ePRAm.Direction.values()).forEach(p -> properties.add(p.name()));
    Arrays.stream(ePRAm.Distance.values()).forEach(p -> properties.add(p.name()));
    return properties;
  }

  /**
   * Calculates the set of QSR relationships from an agent to known entities in the world.
   * Used to generate relational predicates from consultants
   * Consultant currently hard-coded as vision.
   * QSR currently hard-coded as ePRAm.
   *
   * @param agent The DIARC agent that we want to calculate relations for
   * @return A set of predicates describing the QSRs from the agent to the known entities in the world.
   */
  @TRADEService
  public Set<Predicate> observeQSR(Symbol agent) {

    Set<Predicate> state = new HashSet<>();

    Map<Symbol, Double> symbols;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getActivatedEntities").inGroups("location"));
      symbols = tsi.call(Map.class);
      tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class, List.class));
      for (Symbol symbol : symbols.keySet()) {
        Point3d agentLoc = tsi.call(Point3d.class, agent, Point3d.class);
        Point3d targetLoc = tsi.call(Point3d.class, symbol, Point3d.class);
        for (String relation : ePRAm.calculateRelations(agentLoc, targetLoc)) {
          state.add(Factory.createPredicate(relation, agent, symbol));
        }
      }
    } catch (TRADEException e) {
      throw new RuntimeException(e);
    }
    return state;
  }
}
