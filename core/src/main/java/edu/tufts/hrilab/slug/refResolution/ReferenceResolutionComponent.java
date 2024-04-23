/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.common.Utterance;

import java.util.*;
import java.util.stream.Collectors;

import edu.tufts.hrilab.vision.util.PredicateHelper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import scala.collection.JavaConverters;

import static java.util.stream.Collectors.toMap;

import ai.thinkingrobots.trade.*;

public class ReferenceResolutionComponent extends DiarcComponent {
  final protected Logger log = LoggerFactory.getLogger(this.getClass());

  //Brad: from hyper resolver
  private GHSelfModel gh;
  private Map<Symbol, EntityScore> relevanceMap = new HashMap<>(); //the actual data structure that Growler uses
  private ParseReport previousUtterance;
  private Resolver resolver;


  //Brad: a bit of a hack holds the most recently learned pose or location, since the ref isn't generated at the time
  //      of the parse report the RE in the teaching utterance isn't included in the saliencey calculation. This approximates that.

  public ReferenceResolutionComponent() {
    super();
    gh = new GHSelfModel();
    previousUtterance = new ParseReport(null, new ArrayList<>());
  }


  protected void init() {
    //Instantiate new instance of resolution classes
    resolver = new Resolver(this.getMyGroups());

    //subscribe to join notifications from consultants
    try {
      TRADE.requestNotification(this, "joined", new TRADEServiceConstraints().name("getKBName"),null, "newConsultantCallback");
    } catch (TRADEException e) {
      log.error("[init] error registering for TRADE notification for new consultant join");
    }
  }

  @Override
  protected void shutdownComponent() {
    try {
      TRADE.cancelNotification(this, "joined",null);
    } catch (TRADEException e) {
      log.error("[init] error canceling TRADE notification for new consultant join");
    }

    TRADEServiceInfo updatePropertiesHandledService = getMyService("updateResolverPropertiesHandled");
    TRADE.getAvailableServices(new TRADEServiceConstraints().name("unregisterForNewPropertyNotification")).forEach(x -> {
      try {
        x.call(void.class, updatePropertiesHandledService);
      } catch (TRADEException e) {
        log.error("[shutdownComponent] error unregistering for new property notifications from consultants when shutting down",e);
      }
    });
  }

  //todo: not great to implement callbacks here instead of in the Resolver, but I needed "getMyService" which is implemented on DiarcComponent
  /**
   * Calls down to the resolver to update information about consultants after a new consultant is added.
   * Additionally, registers for notifications from consultants for when new properties are added to the consultant.
   *
   * @param tsc TRADEServiceConstraints provided by caller.
   */
  @TRADEService
  public void newConsultantCallback(TRADEServiceInfo tsi) {
    resolver.updateConsultCache(null);

    TRADEServiceInfo updatePropertiesHandledService = getMyService("updateResolverPropertiesHandled");
    TRADE.getAvailableServices(new TRADEServiceConstraints().name("registerForNewPropertyNotification")).forEach( x -> {
      try {
        x.call(void.class, updatePropertiesHandledService);
      } catch (TRADEException e) {
        log.error("[newConsultantCallback] error registering for new properties callback after a new consultant joined",e);
      }
            }
    );
  }

  /**
   * Callback for a new property being added to a consultant.
   *
   */
  @TRADEService
  public void updateResolverPropertiesHandled() {
    resolver.updatePropertyCache();
  }

  /**
   * Attempts to resolve references in the Utterance.
   *
   * @return returns Utterance with resolved reference (can still contain unresolved references)
   */
  @TRADEService
  @Action
  public Utterance resolveReferences(Utterance utterance) {
    log.info("Resolving references in " + utterance);
//    resolver.registerWithTRADE(); // registering here causes deadlock in some trade configurations (e.g., 3 containers across 2 machines)
    //TODO:brad: I don't think we always want to do this, but we need to for the multi robot case
    resolver.updateConsultCache(utterance.getListeners().get(0).toString());
    Growler growler = new Growler(resolver, gh, JavaConverters.mapAsScalaMap(relevanceMap));

    //this check is here so that only free vars that actually exist in the semantics are resolved,
    //sort of a temporary work around for cases like the Nao's where we don't actually want to do RR
    if (!utterance.getSupplementalSemantics().isEmpty() &&
            ((Term) utterance.getSemantics()).getOrderedLeaves().stream().anyMatch(Symbol::isVariable)) {

      //split into sets of topographically related free vars
      Map<List<Variable>, List<Term>> entitiyVarSets =
              PredicateHelper.splitIntoEntities(utterance.getSupplementalSemantics());
      log.debug("entity var sets: " + entitiyVarSets);

      List<List<Hypothesis>> bindingHypotheses = new ArrayList<>();

      //resolve things incrementally by free var set
      for (Map.Entry<List<Variable>, List<Term>> entry : entitiyVarSets.entrySet()) {

        //Determine GHTier of vars to be resolved
        Map<Variable, Symbol> relevantVars = new HashMap<>();
        for (Variable v : entry.getKey()) {
          relevantVars.put(v, utterance.getTierAssignments().get(v));
        }
        log.debug("relevant vars: " + relevantVars);

        //Update GH Model
        //Everything that's in focus or activated is shifted into the Discourse Context,
        //i.e., familiar buffer, and the focus and activated buffers are flushed.
        gh.updateFAM();

        //flush the old relevance map.
        relevanceMap.clear();

        //Use the parse report from the previous utterance to update the "relevance" information for each entity.
        //Currently we're only using one utterance worth of linguistic information.
        //Ultimately we need to factor in the overall dialogue recency, right?
        updateRelevanceMapWithLinguisticInformation(previousUtterance);

        //update focus based on learning salience;
//        if(lastLearned != null) {
//          gh.updateFOC(lastLearned);
//        }
        //Update what's in the focus of attention based on the parse report
        gh.updateFOC(previousUtterance);
        //Update what's Activated based on the parse report and based on what consultants report to be salient
        gh.updateACT(previousUtterance, getSalientRefs(resolver.getAllActivatedEntities()));
        log.debug("Updated GH model");
        log.trace("relevance map: " + relevanceMap);
        //Now we perform reference resolution!

        List<Hypothesis> currentHypothesis = growler.resolve_clause(entry.getValue(), relevantVars);

        bindingHypotheses.add(currentHypothesis);

        log.debug("current Hypotheses: " + currentHypothesis);
        log.debug("Binding Hypotheses: " + bindingHypotheses);

        //Store linguistic information about this utterance for use in future resolution.
        previousUtterance = new ParseReport(utterance, currentHypothesis);
      }

      //create a list of all possible bindings given the hypotheses we have
      Hypothesis def = new Hypothesis(new scala.collection.immutable.HashMap<>(), 1.0);
      List<Hypothesis> finalHyp = new ArrayList<>();
      generatePermutations(bindingHypotheses, finalHyp, 0, def);
      bind(utterance, finalHyp);
      if (utterance != null &&
              !utterance.getBindings().isEmpty() &&
              utterance.getBindings().get(0).isEmpty() &&
              !utterance.getSupplementalSemantics().isEmpty()) {
        log.warn("Found no bindings found for utterance: " + utterance);
      } else {
        log.info("Bound Utterances: " + utterance);
        log.debug("bindings: {}",utterance.getBindings());
      }

    } else {
      previousUtterance = new ParseReport(utterance, new ArrayList<>());
      log.debug("nothing to resolve in utterance: " + utterance);
    }

    return utterance;
  }

  //Helper methods used in addUtterance:

  //incorporates paresReport information into relevance map, which gets used by Growler
  private void updateRelevanceMapWithLinguisticInformation(ParseReport p) {
    for (Map.Entry<Symbol, ParseReportEntry> e : p.entrySet()) {
      //Right now ignoring "syntactic prominence" as focus || subject ends up being identical to main clause.
      //brad: changed this to the OR to see what happens
      log.debug("[urmwli] ref: " + e.getKey() + " pre: " + e.getValue());
      relevanceMap.put(e.getKey(), new EntityScore(e.getValue().mainClause || e.getValue().focus, 0, e.getValue().recency, 0));
    }
  }

  //incorporates salience information from consultants into relevance map
  private List<Symbol> getSalientRefs(Map<Symbol, Double> activatedEntities) {
    log.debug("[getSalientRefs] Activated Entities: " + activatedEntities);
    return activatedEntities.entrySet().stream().map(e -> {
              log.debug("[getSalientRefs] Here's an activated entity! " + e);
              Symbol s = e.getKey();
              if (relevanceMap.containsKey(s)) { //If it's already in the map, just update its "bonus".
                EntityScore es = relevanceMap.get(s);
                relevanceMap.put(s, new EntityScore(es.inMainClause(), es.synProm(), es.recency(), java.lang.Math.max(es.bonus(), e.getValue())));
              } else { // Otherwise add a new entry with its bonus.
                //TODO:brad:this was probably a bug in the original hyperresolver, or at least never used...
                relevanceMap.put(s, new EntityScore(false, 0, 0, e.getValue()));
              }
              log.debug("[getSalientRefs] Got Salient Refs ({}}) -- rM ({})",s,relevanceMap);
              return s;
            }
    ).collect(Collectors.toList());
  }

  private void generatePermutations(List<List<Hypothesis>> Lists, List<Hypothesis> result, int depth, Hypothesis
          current) {
    if (depth == Lists.size()) {
      result.add(current);
      return;
    }
    for (int i = 0; i < Lists.get(depth).size(); ++i) {
      generatePermutations(Lists, result, depth + 1, current.merge(Lists.get(depth).get(i)));
    }
  }

  /**
   * Converts hypotheses list to the form used in Utterance.getBindings() and then adds those bindings to the Utterance.
   * @param u
   * @param hypotheses
   */
  private void bind(Utterance u, List<Hypothesis> hypotheses) {
    if (hypotheses.isEmpty()) {
      //TODO:brad: figure out what to do here
    } else {
      for (Hypothesis h : hypotheses) {
        //fixme
        Map<Variable, Symbol> filtered = JavaConverters.mapAsJavaMap(h.assignments()).entrySet().stream()
            .filter(a -> !a.getValue().getName().contains("?")).collect(toMap(Map.Entry::getKey, Map.Entry::getValue));
        u.addBinding(filtered);
      }

      if (log.isDebugEnabled()) {
        //get all free vars from supplemental semantics
        List<Variable> vars = new ArrayList<>();
        for (Term s : u.getSupplementalSemantics()) {
          vars.addAll(s.getOrderedVars());
        }

        log.debug("supplemental semantics: " + u.getSupplementalSemantics());
        log.debug("vars: " + vars);
        log.debug("bound semantics: " + u.getBoundSemantics());
        log.debug("bindings: " + u.getBindings());
      }
    }
  }

  /**
   * generate referring expression (list of descriptor bindings) for a given ref_id
   *
   * @param ref reference to get bindings for
   * @return map of descriptor bindings for use in NLG
   */
  @TRADEService
  public Map<Symbol, List<Term>> generateRE(Symbol ref) {
    log.debug("generating RE - component level");
    return resolver.generateRE(ref);
  }

  @TRADEService
  public List<Term> getProperties(Symbol ref) {
    log.debug("[getProperties] ref: "+ref);
    return resolver.getProperties(ref);
  }

  @TRADEService
  public <E> E getEntityForReference(Symbol ref, Class<E> entityJavaType){
    return resolver.getEntityForReference(ref,entityJavaType);
  }

  @TRADEService
  public <E> E getEntityForReference(Symbol ref, Class<E> entityJavaType, List<Term> constraints){
    return resolver.getEntityForReference(ref,entityJavaType,constraints);
  }

  @TRADEService
  public void updateFOC(Symbol r) {
    gh.focBuffer.add(r);
  }

  @TRADEService
  @Action
  public boolean assertProperties(Symbol ref, List<Term> properties) {
    return resolver.assertProperties(ref, properties);
  }

  /**
   * Posits a hypothetical reference that has all the properties in properties.
   *
   * @param properties list of properties for hypothetical reference
   * @return the posited reference
   */
  @TRADEService
  @Action
  public Symbol positReference(List<Term> properties) {
    return resolver.positReference(properties, null);
  }

  /**
   * Posits a hypothetical reference that has all of the properties in properties. This is a convince method so that it can easily be called during action learning.
   *
   * @param propertiesList wrapper predicate whose args are a list of properties used to posit the reference
   * @return true if there is a consultant that can handle all of the members of properties
   */
  @TRADEService
  @Action
  public Symbol positReference(Symbol actor,Term propertiesList) {
    List<Term> props = new ArrayList<>();
    for(Symbol prop: propertiesList.getArgs()){
      if(prop.isTerm()){
        props.add((Term) prop);
      }else{
        log.warn("[positReference] and valid property: "+prop);
      }
    }
    return resolver.positReference(props, actor);
  }


  @TRADEService
  @Action
  @Observes({"property_of(X,Y)"})
  public List<Map<Variable, Symbol>> observeProperties(Symbol actor, Term query) {
    List<Map<Variable,Symbol>> bindings = new ArrayList<>();
    if(query.getArgs().size() ==2) {

      Symbol refID = query.get(0);
      List<Term> properties =resolver.getProperties(refID);

      Symbol queryProperty = query.get(1);
      if(queryProperty.isVariable()) {
        for (Term p : properties) {
          Map<Variable, Symbol> binding = new HashMap<>();
          binding.put((Variable) queryProperty, Factory.createSymbol(p.getName()));
          bindings.add(binding);
        }
      }
      else if(queryProperty.isTerm()){
        log.warn("[observeProperties] invalid query format, second arg can't be a Term: "+query);
      } else{
        for (Term p : properties) {
          if(queryProperty.getName().equals(p.getName())) {
            bindings.add(new HashMap<>());
            return bindings;
          }
        }
      }
    }else{
      log.warn("[observeProperties] invalid observation predicate: "+query);
    }

    return bindings;
  }

/////////////////////////////////////////////////////////////////////////////
///////////////////// internal classes //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

  //This was a tuple in the scala impl, it's just a way of packaging the information about how a ref was used in its utterance
  private class ParseReportEntry {
    boolean subjects;
    boolean focus;
    boolean mainClause;
    int recency;

    ParseReportEntry(boolean subjects, boolean focus, boolean mainClause, int recency) {
      this.subjects = subjects;
      this.focus = focus;
      this.mainClause = mainClause;
      this.recency = recency;
    }
  }

  //for a given utterance this class represents how each ref was used in the utterance in which it appeared, in terms of syntactic prominence (I think that is the term used in the papers)
  private class ParseReport extends HashMap<Symbol, ParseReportEntry> {

    //Is this reference the subject under any candidate interpretation?
    //i.e., is there a variable v potentially bound to this ref, that is the first argument of the utterance's clause?
    //TODO:brad: upon the integration of frame semantics this can be done differently
    private boolean isSubject(Variable ref, Map<Variable, List<Symbol>> allRefs, Utterance u) {
      // give edge case for if the predicate hasn't been wrapped a second time (i.e. by pragmatics)
      if (u.getSemantics().isTerm()) {
        if (((Term) u.getSemantics()).getOrderedLeaves().size() <= 1) {
          return allRefs.keySet().stream().filter(x -> x.getName().equals(ref.getName())).collect(Collectors.toSet()).stream().anyMatch(
                  //TODO:brad: should this be the name or the object
                  v -> ((Term) u.getSemantics()).getOrderedLeaves().get(0).getName().equals(v.getName())
          );
        }
        return allRefs.keySet().stream().filter(x -> x.getName().equals(ref.getName())).collect(Collectors.toSet()).stream().anyMatch(
                //TODO:brad: should this be the name or the object
                v -> ((Term) u.getSemantics()).getOrderedLeaves().get(1).getName().equals(v.getName())
        );
      } else {
        return false;
      }
    }

    //Is this reference the syntactic focus under any candidate interpretation?
    //i.e., is there a variable v potentially bound to this ref, that is in any *other* argument of the utterance's clause?
    private boolean isFocus(Variable ref, Map<Variable, List<Symbol>> allRefs, Utterance u) {
      if (u.getSemantics().isTerm()) {
        return allRefs.keySet().stream().filter(x -> x.getName().equals(ref.getName())).collect(Collectors.toSet()).stream().anyMatch(
                v -> ((Term) u.getSemantics()).getOrderedLeaves().stream().anyMatch(
                        //TODO:brad: should this be the name or the object
                        x -> x.getName().equals(v.getName())
                )
        );
      }
      return false;
    }

    //Is this reference in the main clause under any candidate interpretation?
    //i.e., is there a variable v potentially bound to this ref, that is in any argument of the utterance's clause?
    //Note that right now this is equivalent to isSubject || isFocus.
    private boolean isMainClause(Variable ref, Map<Variable, List<Symbol>> allRefs, Utterance u) {
      if (u.getSemantics().isTerm()) {
        return allRefs.keySet().stream().filter(x -> x.getName().equals(ref.getName())).collect(Collectors.toSet()).stream().anyMatch(
                v -> ((Term) u.getSemantics()).getOrderedLeaves().stream().anyMatch(
                        //TODO:brad: should this be the name or the object
                        x -> x.getName().equals(v.getName())
                )
        );
      }
      return false;
    }

    //What is the average recency of this reference across interpretations?
    //TODO:brad: wait, are we calculating the average reference id, and using that as a measure of recency? I'm not sure I understand how that makes sense.
    private int getRecency(Variable ref, Map<Variable, List<Symbol>> allRefs) {

      //variablePattern = "" "([^0-9]*)([0-9]*)" "".r
      List<Symbol> usages = allRefs.get(ref);
      int sum = 0;
      for (Symbol useage : usages) {
        sum += Integer.valueOf(useage.getName().split("_")[1]);
      }

      //not really sure why we need to do this, but preserving previous functionality, for now...
      if (sum == 0) sum = 1;

      //TODO:brad do we really want this to be an int?
      return sum / usages.size();
    }

    ParseReport(Utterance u, List<Hypothesis> hyps) {
      super();

      log.debug("Building parse report!");
      Map<Variable, List<Symbol>> allRefs = new HashMap<>();
      for (Hypothesis h : hyps) {
        for (Map.Entry<Variable, Symbol> assignment : JavaConverters.mapAsJavaMap(h.assignments()).entrySet()) {
          if (allRefs.containsKey(assignment.getKey())) {
            allRefs.get(assignment.getKey()).add(assignment.getValue());
          } else {
            List<Symbol> bindingList = new ArrayList<>();
            bindingList.add(assignment.getValue());
            allRefs.put(assignment.getKey(), bindingList);
          }
        }
      }

      for (Map.Entry<Variable, List<Symbol>> ref : allRefs.entrySet()) {
        for (Symbol refID : ref.getValue()) {
          log.debug("Adding an entry for ref " + refID + " subject?: " + isSubject(ref.getKey(), allRefs, u) + " focus?: " + isFocus(ref.getKey(), allRefs, u) + " mainClause?: " + isMainClause(ref.getKey(), allRefs, u) + " recency: " + getRecency(ref.getKey(), allRefs));
          ParseReportEntry pre = new ParseReportEntry(isSubject(ref.getKey(), allRefs, u), isFocus(ref.getKey(), allRefs, u), isMainClause(ref.getKey(), allRefs, u), getRecency(ref.getKey(), allRefs));
          log.debug("ParseReportEntry {}", pre);
          this.put(refID, pre);
        }
      }
    }
  }

  //A model of the memory used in the Givneness Hierarchy, it stores refs in buffers based on their cognitive status, recency, and salience from consultants
  static class GHSelfModel {
    private static Logger log = LoggerFactory.getLogger(GHSelfModel.class);

    // EAK: it's unclear if these need to be ordered -- converted from list to get rid of duplicates
    LinkedHashSet<Symbol> focBuffer = new LinkedHashSet<>();
    LinkedHashSet<Symbol> actBuffer = new LinkedHashSet<>();
    LinkedHashSet<Symbol> famBuffer = new LinkedHashSet<>();

    //Familiar buffer
    void updateFAM() {
      //TODO:does it matter that this used to be prepend?
      famBuffer.addAll(focBuffer);
      famBuffer.addAll(actBuffer);
      focBuffer.clear();
      actBuffer.clear();
      log.trace("FAM updated: " + famBuffer);
    }

    //Focus of Attention
    void updateFOC(ParseReport pr) {
      Map<Symbol, ParseReportEntry> relevantEntries = pr.entrySet().stream().filter(v -> v.getValue().mainClause).collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
      focBuffer.addAll(relevantEntries.keySet().stream().filter(k -> !focBuffer.contains(k) && !k.getType().contains("dialog")).collect(Collectors.toList()));
      log.debug("FOC updated: " + focBuffer);
    }

    void updateFOC(Symbol r){
      focBuffer.add(r);
    }

    //Activated
    void updateACT(ParseReport pr, List<Symbol> srs) {
      actBuffer.addAll(pr.keySet().stream().filter(x -> !actBuffer.contains(x) && !x.getType().contains("dialog")).collect(Collectors.toList()));
      actBuffer.addAll(srs.stream().filter(x -> !actBuffer.contains(x)).collect(Collectors.toList()));
      actBuffer.removeAll(focBuffer);
      log.trace("ACT updated: " + focBuffer);
    }
  }
}
