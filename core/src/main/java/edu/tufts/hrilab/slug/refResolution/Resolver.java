/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.util.Utilities;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

public class Resolver {

  private final ConstraintOrderer orderer = new SizeOrderer();
  final Logger log = LoggerFactory.getLogger(getClass());
  private final double TAU = 0.1;
  final double GENTAU = 0.8;

  private final TRADEServiceConstraints additionalConstraints = new TRADEServiceConstraints();

  List<ConsultantInfo> consultants;
  protected Map<ConsultantInfo, List<Property>> propertyCache;

  public Resolver(List<String> groups) {
    if (!groups.isEmpty()) {
      additionalConstraints.inGroups(groups.toArray(new String[0]));
    }
    updateConsultCache(null);
  }

  static class ConsultantInfo {
    final String kbName;
    final TRADEServiceInfo tsi;
    final TRADEServiceConstraints tsc;

    ConsultantInfo(String kbName, TRADEServiceInfo tsi, TRADEServiceConstraints tsc) {
      this.kbName = kbName;
      this.tsi = tsi;
      this.tsc = tsc;
    }

    @Override
    public String toString() {
      return kbName + "Info";
    }
  }

  public void updateConsultCache(Symbol actor) {
    if (actor == null) {
      Collection<TRADEServiceInfo> services = TRADE.getAvailableServices(
          additionalConstraints.name("getKBName").argTypes());
      consultants = new ArrayList<>();
      for (TRADEServiceInfo c : services) {
        String name = null;
        try {
          name = c.call(String.class);
        } catch (TRADEException e) {
          log.error("[updateConsultCache] exception calling getKBName", e);
        }
        String[] groupArr = c.getGroups().toArray(new String[0]);
        TRADEServiceConstraints tsc = new TRADEServiceConstraints().inGroups(groupArr);
        consultants.add(new ConsultantInfo(name, c, tsc));
      }
    } else {
      Set<String> agents;
      try {
        agents = TRADE.getAvailableService(
            new TRADEServiceConstraints().name("getAllDiarcAgentsForActor")
                .argTypes(Symbol.class, Boolean.class))
            .call(Set.class, actor, false);
      } catch (Exception e) {
        log.error("[updateConsultCache] exception calling getAllDiarcAgentsForActor", e);
        agents = new HashSet<>();
      }

      Set<String> untypedActors = new HashSet<>();
      Set<String> typedActors = new HashSet<>();
      for (String a : agents) {
        Symbol agent = Factory.createSymbol(a);
        untypedActors.add("agent:" + agent.getName());
        typedActors.add("agent:" + (agent.hasType() ? agent : Factory.createSymbol(agent.getName(), "agent")));
      }

      Collection<TRADEServiceInfo> services = TRADE.getAvailableServices(
          additionalConstraints.name("getKBName").argTypes());
      consultants = new ArrayList<>();
      for (TRADEServiceInfo c : services) {
        List<String> cGroups = new ArrayList<>(c.getGroups());
        boolean include = cGroups.isEmpty()
            || cGroups.stream().noneMatch(g -> g.startsWith("agent:"))
            || cGroups.stream().anyMatch(g -> untypedActors.contains(g) || typedActors.contains(g));
        if (include) {
          String name = null;
          try {
            name = c.call(String.class);
          } catch (TRADEException e) {
            log.error("[updateConsultCache] exception calling getKBName", e);
          }
          String[] groupArr = cGroups.toArray(new String[0]);
          TRADEServiceConstraints tsc = new TRADEServiceConstraints().inGroups(groupArr);
          consultants.add(new ConsultantInfo(name, c, tsc));
        }
      }
    }
    updatePropertyCache();
  }

  public void updatePropertyCache() {
    propertyCache = new HashMap<>();
    for (ConsultantInfo c : consultants) {
      try {
        List<Term> handled = TRADE.getAvailableService(
            c.tsc.name("getPropertiesHandled").argTypes()).call(List.class);
        List<Property> props = handled.stream().map(Property::new).collect(Collectors.toList());
        propertyCache.put(c, props);
      } catch (Exception e) {
        log.error("[updatePropertyCache] exception for consultant " + c.kbName, e);
        propertyCache.put(c, new ArrayList<>());
      }
    }
  }

  public <E> E getEntityForReference(Symbol ref, Class<E> entityJavaType) {
    List<ConsultantInfo> matching = consultants.stream()
        .filter(c -> ref.getName().contains(c.kbName))
        .collect(Collectors.toList());
    if (matching.size() != 1) {
      log.warn("[getEntityForReference] Multiple consultants with the same kbName, using the first one " + matching);
    }
    try {
      return TRADE.getAvailableService(
          matching.get(0).tsc.name("convertToType").argTypes(Symbol.class, Class.class))
          .call(entityJavaType, ref, entityJavaType);
    } catch (TRADEException e) {
      log.error("[getEntityForReference] exception calling convertToType", e);
      return null;
    }
  }

  public boolean assertProperties(Symbol ref, List<Term> properties) {
    List<ConsultantInfo> matching = consultants.stream()
        .filter(c -> ref.getName().contains(c.kbName))
        .collect(Collectors.toList());
    if (matching.size() != 1) {
      log.warn("[assertProperties] Multiple consultants with the same kbName, using the first one " + matching);
    }
    boolean success = false;
    try {
      success = TRADE.getAvailableService(
          matching.get(0).tsc.name("assertProperties").argTypes(Symbol.class, List.class))
          .call(Boolean.class, ref, properties);
    } catch (Exception e) {
      log.error("exception calling assertProperties on: " + matching.get(0).kbName, e);
    }
    return success;
  }

  public Symbol positReference(List<Term> properties, Symbol actor) {
    updateConsultCache(actor);
    List<ConsultantInfo> matchingConsultants = new ArrayList<>();
    for (ConsultantInfo c : consultants) {
      try {
        List<Term> handled = TRADE.getAvailableService(
            c.tsc.name("getPropertiesHandled").argTypes()).call(List.class);
        boolean matches = properties.stream().anyMatch(q ->
            handled.stream().anyMatch(p ->
                p.getName().equals(q.getName())
                    && p.getArgs().size() == q.getArgs().size()
                    && p.getArgs().get(0).getType().equals(q.getArgs().get(0).getType())));
        if (matches) matchingConsultants.add(c);
      } catch (Exception e) {
        log.error("[positReference] exception getting propertiesHandled for " + c.kbName, e);
      }
    }

    if (matchingConsultants.isEmpty()) {
      log.warn("[positReference] no consultant found that can handle: " + properties);
      return null;
    } else if (matchingConsultants.size() != 1) {
      log.warn("[positReference] Multiple consultants with the same kbName, using the first one " + matchingConsultants);
    }

    List<Variable> vars = Utilities.getUnboundVariables(properties);
    try {
      Map<Variable, Symbol> m = TRADE.getAvailableService(
          matchingConsultants.get(0).tsc.name("createReferences").argTypes(List.class))
          .call(Map.class, vars);
      TRADE.getAvailableService(
          matchingConsultants.get(0).tsc.name("assertProperties").argTypes(Map.class, Double.class, List.class))
          .call(Boolean.class, m, 1.0, properties);
      if (m.size() > 1) {
        log.warn("[positReference] created more than 1 reference. Returning first reference. Properties: " + properties);
      }
      Symbol refID = m.get(vars.get(0));
      log.debug("[positReference] posited new reference: " + refID);
      return refID;
    } catch (TRADEException e) {
      log.error("[createReference] exception calling createReference", e);
      return null;
    }
  }

  public List<Hypothesis> resolve(List<Property> query, List<Variable> co, List<Hypothesis> initialHyps, boolean posit) {
    log.debug("[resolve]");
    List<Variable> T = getVariableMapping(query);
    log.debug("[resolve] T: {}", T);

    List<Property> boundOrderedQuery = query.stream()
        .map(p -> p.boundForm(T))
        .sorted(orderer.toComparator())
        .collect(Collectors.toList());

    log.debug("[resolve] BOQ: {}", boundOrderedQuery);

    SolutionsResult result = getSolutions(boundOrderedQuery, co, initialHyps);
    List<Hypothesis> solutions = result.solutions;
    List<Variable> remainingVars = result.remainingVars;

    List<Variable> abandonedVariables = new ArrayList<>(co);
    abandonedVariables.removeAll(remainingVars);

    log.debug("[resolve] Valid Solutions: ");
    solutions.forEach(x -> log.debug("\t SOL: {}", x));

    if (posit && !abandonedVariables.isEmpty()) {
      log.debug("[resolve] Positing before returning solutions");
      return completeSolutions(solutions, abandonedVariables, boundOrderedQuery);
    } else {
      log.debug("[resolve] Returning solutions; no positing necessary");
      return solutions;
    }
  }

  public List<Hypothesis> completeSolutions(List<Hypothesis> s, List<Variable> av, List<Property> q) {
    // Remove 'unfamiliar' from q
    List<Property> q2 = q.stream()
        .filter(t -> !t.nonDSPredicateForm().getName().equals("unfamiliar"))
        .collect(Collectors.toList());

    log.trace("IN COMPLETE SOLUTIONS");
    log.trace("compSol: " + s + ", " + av + ", " + q2);

    Hypothesis bestS = s.isEmpty() ? new Hypothesis(new HashMap<>(), 1.0) : s.get(0);

    List<ConsultantInfo> cons = new ArrayList<>();
    for (Variable v : av) {
      Optional<Property> matchingProp = q2.stream()
          .filter(p -> p.predicateForm().getVars().stream().anyMatch(x -> x.getName().equals(v.getName())))
          .findFirst();
      if (matchingProp.isPresent()) {
        Property p = matchingProp.get();
        for (ConsultantInfo c : consultants) {
          try {
            List<Term> handled = TRADE.getAvailableService(
                c.tsc.name("getPropertiesHandled").argTypes()).call(List.class);
            boolean matches = handled.stream().map(Property::new).anyMatch(cp -> cp.matches(p));
            if (matches) {
              cons.add(c);
              break;
            }
          } catch (Exception e) {
            log.error("[completeSolutions] exception getting propertiesHandled", e);
          }
        }
      }
    }

    Map<Variable, Symbol> newAssignments = new HashMap<>(bestS.assignments());
    int limit = Math.min(av.size(), cons.size());
    for (int i = 0; i < limit; i++) {
      Variable v = av.get(i);
      ConsultantInfo c = cons.get(i);
      List<Variable> vars = new ArrayList<>();
      vars.add(new Variable(v.getName(), c.kbName));
      try {
        Map<Variable, Symbol> created = TRADE.getAvailableService(
            c.tsc.name("createReferences").argTypes(List.class))
            .call(Map.class, vars);
        newAssignments.putAll(created);
      } catch (Exception e) {
        log.error("[completeSolutions] exception calling createReferences", e);
      }
    }

    List<Term> nonDSTerms = q2.stream().map(Property::nonDSPredicateForm).collect(Collectors.toList());
    for (ConsultantInfo c : consultants) {
      try {
        TRADE.getAvailableService(
            c.tsc.name("assertProperties").argTypes(Map.class, Double.class, List.class))
            .call(Boolean.class, new HashMap<>(newAssignments), 1.0, nonDSTerms);
      } catch (TRADEException e) {
        log.error("[completeSolutions] exception calling assertProperties", e);
      }
    }

    return List.of(new Hypothesis(newAssignments, bestS.likelihood()));
  }

  private SolutionsResult getSolutions(List<Property> bOQ, List<Variable> co, List<Hypothesis> initialHyps) {
    List<Hypothesis> solutions = bfs(bOQ, initialHyps);
    if (solutions.isEmpty() && !co.isEmpty()) {
      List<Property> rr = removeReferences(bOQ, co.get(0));
      List<Variable> coTail = co.subList(1, co.size());
      if (!coTail.isEmpty() && !rr.isEmpty()) {
        return getSolutions(rr, coTail, initialHyps);
      } else {
        return new SolutionsResult(solutions, coTail);
      }
    }
    log.debug("[getSolutions] returning ({},{}) ", solutions, co, "");
    return new SolutionsResult(solutions, co);
  }

  private List<Property> removeReferences(List<Property> query, Variable variable) {
    log.debug("removing {} from {}", variable, query, "");
    return query.stream()
        .filter(p -> p.predicateForm().getVars().stream().noneMatch(v -> v.equals(variable)))
        .collect(Collectors.toList());
  }

  private double scoreT(List<Variable> t, List<Property> S) {
    double ret = 1.0;
    for (Property s : S) {
      ret += Pts(s, s.boundForm(t));
    }
    double retFinal = ret;
    t.forEach(v -> log.debug("Variable " + v + " Type " + v.getType() + " Properties " + S + " Score " + retFinal));
    return ret;
  }

  private double Pts(Property s, Property st) {
    int g = gamma(st);
    double ret = g == 0 ? 0.0 : 1.0 / gamma(s);
    Variable sArg = (Variable) ((Term) s.predicateForm().get(0)).get(0);
    Variable stArg = (Variable) ((Term) st.predicateForm().get(0)).get(0);
    log.debug("Pts for " + sArg.getType()
        + " " + s + " / " + stArg.getType() + " " + st + ": " + ret);
    return ret;
  }

  private int gamma(Property s) {
    return (int) propertyCache.values().stream().flatMap(List::stream).filter(p -> p.matches(s)).count();
  }

  public List<Variable> getVariableMapping(List<Property> S) {
    List<Variable> SV = S.stream()
        .flatMap(p -> p.predicateForm().getOrderedVars().stream())
        .distinct()
        .collect(Collectors.toList());

    SV.forEach(x -> log.debug("[getVariableMapping] {} -> {}", SV, x, ""));

    if (SV.isEmpty()) {
      return new ArrayList<>();
    }

    Variable firstVar = SV.get(0);
    List<List<Variable>> T = consultants.stream()
        .map(c -> {
          List<Variable> entry = new ArrayList<>();
          entry.add(new Variable(firstVar.getName(), firstVar.getType().isEmpty() ? c.kbName : firstVar.getType()));
          return entry;
        })
        .collect(Collectors.toList());

    for (int i = 1; i < SV.size(); i++) {
      Variable V = SV.get(i);
      List<List<Variable>> newT = new ArrayList<>();
      for (List<Variable> mi : T) {
        for (ConsultantInfo c : consultants) {
          List<Variable> newEntry = new ArrayList<>(mi);
          newEntry.add(new Variable(V.getName(), c.kbName));
          newT.add(newEntry);
        }
      }
      T = newT;
    }

    if (T.isEmpty()) {
      log.error("Variable mapping failed. Likely because no consultant found supporting supplied kbName");
      return new ArrayList<>();
    }

    List<Property> sFinal = S;
    return T.stream()
        .max(Comparator.comparingDouble(t -> scoreT(t, sFinal)))
        .orElse(new ArrayList<>());
  }

  protected List<Symbol> initialDomain(String v) throws Exception {
    Optional<ConsultantInfo> match = consultants.stream()
        .filter(c -> c.kbName.equals(v))
        .findFirst();
    if (!match.isPresent()) throw new Exception("No consultant keyed with name " + v);
    return TRADE.getAvailableService(
        match.get().tsc.name("getInitialDomain").argTypes(List.class))
        .call(List.class, new ArrayList<>());
  }

  public double assess(Hypothesis h, Property prop) {
    log.trace("In assess with " + h + " -- " + prop);
    double propLikelihood;
    Optional<ConsultantInfo> consultantOpt = getConsultant(h, prop);
    if (consultantOpt.isPresent()) {
      ConsultantInfo consultantInfo = consultantOpt.get();
      log.debug("Chose consultant: " + consultantInfo.kbName);
      try {
        propLikelihood = TRADE.getAvailableService(
            consultantInfo.tsc.name("process").argTypes(Term.class, Map.class))
            .call(Double.class, prop.nonDSPredicateForm(), new HashMap<>(h.assignments()));
      } catch (Exception e) {
        log.error("[Assess] Exception calling process", e);
        propLikelihood = 0.0;
      }
    } else {
      log.debug("Couldn't find a consultant for hypothesis and property " + h + " " + prop);
      propLikelihood = 0.0;
    }
    return h.likelihood() * propLikelihood;
  }

  public Map<Symbol, Double> getAllActivatedEntities() {
    Map<Symbol, Double> result = new HashMap<>();
    for (ConsultantInfo c : consultants) {
      try {
        Map<Symbol, Double> entities = TRADE.getAvailableService(
            c.tsc.name("getActivatedEntities").argTypes()).call(Map.class);
        result.putAll(entities);
      } catch (Exception e) {
        log.error("[getAllActivatedEntities] consultant: " + c.kbName, e);
      }
    }
    log.debug("[gAAE] Result: " + result);
    return result;
  }

  public Map<Symbol, Double> getActivatedEntities(List<String> groups) {
    Optional<ConsultantInfo> match = consultants.stream()
        .filter(c -> c.tsi.getGroups().containsAll(groups))
        .findFirst();
    if (!match.isPresent()) throw new RuntimeException("No consultant keyed with groups " + groups);
    try {
      return TRADE.getAvailableService(
          match.get().tsc.name("getActivatedEntities").argTypes()).call(Map.class);
    } catch (TRADEException e) {
      log.error("[getActivatedEntities] consultant: " + match.get().kbName, e);
      return new HashMap<>();
    }
  }

  private Optional<ConsultantInfo> getConsultant(Hypothesis h, Property p) {
    List<Variable> vars = new ArrayList<>(h.assignments().keySet());
    Property bf = p.boundForm(vars);
    log.debug("FINDING CONSULTANT FOR: " + bf + " / " + bf.predicateForm().toString());
    log.debug("hypothesis ids: " + h.assignments().values().stream().map(Symbol::getType).collect(Collectors.toList()));

    List<ConsultantInfo> filteredConsultants = consultants.stream()
        .filter(c -> h.assignments().values().stream().anyMatch(a -> a.getType().equals(c.kbName)))
        .collect(Collectors.toList());

    log.debug("filtered consultants: {}", filteredConsultants);

    return filteredConsultants.stream()
        .filter(c -> propertyCache.getOrDefault(c, new ArrayList<>()).stream().anyMatch(cp -> cp.matches(bf)))
        .findFirst();
  }

  public Hypothesis b2h(Map<Symbol, Symbol> b) {
    Map<Variable, Symbol> h = new HashMap<>();
    for (Map.Entry<Symbol, Symbol> entry : b.entrySet()) {
      h.put(new Variable(entry.getKey().getName()), entry.getValue());
    }
    return new Hypothesis(h, 1.0);
  }

  private List<Map<Symbol, Symbol>> allComboBindings(List<Symbol> args) {
    log.debug("ACB CALLED WITH: " + args);
    List<List<Map<Symbol, Symbol>>> candLists = new ArrayList<>();

    for (Symbol sym : args) {
      if (sym instanceof Variable) {
        Variable v = (Variable) sym;
        Optional<ConsultantInfo> consultant = consultants.stream()
            .filter(c -> c.kbName.equalsIgnoreCase(v.getType()))
            .findFirst();
        if (consultant.isPresent()) {
          try {
            List<Symbol> domain = TRADE.getAvailableService(
                consultant.get().tsc.name("getInitialDomain").argTypes(List.class))
                .call(List.class, new ArrayList<>());
            List<Map<Symbol, Symbol>> entries = new ArrayList<>();
            for (Symbol id : domain) {
              Map<Symbol, Symbol> m = new LinkedHashMap<>();
              m.put(v, id);
              entries.add(m);
            }
            candLists.add(entries);
          } catch (Exception e) {
            log.error("[allComboBindings] exception", e);
          }
        }
      }
    }

    log.debug("candLists: " + candLists);
    if (candLists.isEmpty()) return new ArrayList<>();

    List<Map<Symbol, Symbol>> accumulated = candLists.get(0);
    for (int i = 1; i < candLists.size(); i++) {
      List<Map<Symbol, Symbol>> newList = new ArrayList<>();
      for (Map<Symbol, Symbol> a : accumulated) {
        for (Map<Symbol, Symbol> b : candLists.get(i)) {
          Map<Symbol, Symbol> merged = new LinkedHashMap<>(a);
          merged.putAll(b);
          newList.add(merged);
        }
      }
      accumulated = newList;
    }

    log.debug("comboBindings: " + accumulated);
    return accumulated;
  }

  public List<Term> getProperties(Symbol ref) {
    String consultantName = ref.getType();
    log.debug("CONSULTANTNAME: " + consultantName);
    Optional<ConsultantInfo> consultantOpt = consultants.stream()
        .filter(c -> c.kbName.equalsIgnoreCase(consultantName))
        .findFirst();
    if (!consultantOpt.isPresent()) return new ArrayList<>();
    try {
      return TRADE.getAvailableService(
          consultantOpt.get().tsc.name("getAssertedProperties").argTypes(Symbol.class))
          .call(List.class, ref);
    } catch (TRADEException e) {
      log.error("[getProperties] call of getAssertedProperties for ref: " + ref + " from: " + consultantOpt.get().kbName, e);
      return new ArrayList<>();
    }
  }

  public LinkedHashMap<Symbol, List<Term>> generateRE(Symbol ref) {
    log.debug("generating RE -resolver");
    updateConsultCache(null);

    LinkedHashMap<Symbol, List<Property>> found = new LinkedHashMap<>();
    Queue<Symbol> refQueue = new LinkedList<>();
    refQueue.add(ref);

    while (!refQueue.isEmpty()) {
      log.debug("Referent Queue: " + refQueue);
      Symbol r = refQueue.poll();
      if (!found.containsKey(r)) {
        Optional<STMResult> stmResultOpt = stmGenerateRETuple(r, new ArrayList<>(found.keySet()));
        if (!stmResultOpt.isPresent()) {
          return new LinkedHashMap<>();
        }
        STMResult stmResult = stmResultOpt.get();
        Optional<Map.Entry<Symbol, List<Property>>> newTuple = generateRETuple(
            r, new ArrayList<>(found.keySet()), stmResult);
        if (newTuple.isPresent()) {
          Map.Entry<Symbol, List<Property>> d = newTuple.get();
          found.put(d.getKey(), d.getValue());
          refQueue.addAll(refsIn(d.getValue()));
        }
      }
    }

    log.debug("Returning REG: " + found);

    LinkedHashMap<Symbol, List<Term>> ret = new LinkedHashMap<>();
    for (Map.Entry<Symbol, List<Property>> pair : found.entrySet()) {
      ret.put(pair.getKey(), pair.getValue().stream().map(Property::nonDSPredicateForm).collect(Collectors.toList()));
    }
    return ret;
  }

  private Optional<STMResult> stmGenerateRETuple(Symbol ref, List<Symbol> found) {
    String consultantName = ref.getType();
    log.debug("CONSULTANTNAME: " + consultantName);
    List<ConsultantInfo> filteredConsultants = consultants.stream()
        .filter(c -> c.kbName.equalsIgnoreCase(consultantName))
        .collect(Collectors.toList());

    List<Property> description = new ArrayList<>();
    List<Symbol> domain = new ArrayList<>();
    for (ConsultantInfo c : filteredConsultants) {
      try {
        List<Symbol> d = TRADE.getAvailableService(
            c.tsc.name("getInitialDomain").argTypes(List.class))
            .call(List.class, new ArrayList<>());
        domain.addAll(d);
      } catch (Exception e) {
        log.error("[stmGenerateRETuple] exception getting domain", e);
      }
    }
    log.debug("domain: " + domain);
    List<Symbol> distractors = domain.stream().filter(s -> !s.equals(ref)).collect(Collectors.toList());
    log.debug("distractors: " + distractors);

    List<Property> propsList = new ArrayList<>();
    for (ConsultantInfo c : filteredConsultants) {
      try {
        List<Term> asserted = TRADE.getAvailableService(
            c.tsc.name("getAssertedProperties").argTypes(Symbol.class))
            .call(List.class, ref);
        asserted.stream().map(Property::new).forEach(propsList::add);
      } catch (Exception e) {
        log.error("[stmGenerateRETuple] exception getting asserted properties", e);
      }
    }

    Deque<Map.Entry<Property, Map<Symbol, Symbol>>> props = new ArrayDeque<>();
    for (Property p : propsList) {
      props.push(new AbstractMap.SimpleEntry<>(p, new LinkedHashMap<>()));
    }

    log.debug("PROPS: " + props);

    while (!props.isEmpty() && !distractors.isEmpty()) {
      log.debug("Distractors: " + distractors + "; P: " + props);
      Map.Entry<Property, Map<Symbol, Symbol>> entry = props.pop();
      Property prop = entry.getKey();
      Map<Symbol, Symbol> bindings = entry.getValue();
      log.debug("TRYING: " + prop);

      List<Symbol> args = ((Term) prop.predicateForm().get(0)).getArgs();
      log.debug("BINDINGS: " + bindings + " /// FOUND: " + found);

      List<Symbol> bindingValues = new ArrayList<>(bindings.values());
      bindingValues.retainAll(found);
      if (!bindingValues.isEmpty()) {
        log.debug("Refers to something already described, dropping");
      } else {
        log.debug("LINE6 -- " + prop + " -- " + bindings);
        log.debug("args: " + args);

        Optional<Symbol> missingOpt = args.stream().filter(s -> !bindings.containsKey(s)).findFirst();
        if (!missingOpt.isPresent()) {
          throw new RuntimeException("No variables left unfilled?");
        }
        Symbol missing = missingOpt.get();
        log.debug("MISSING: " + missing);

        final List<Symbol> distractorsCopy = new ArrayList<>(distractors);
        List<Symbol> elimDist = distractorsCopy.stream()
            .filter(d -> {
              Map<Symbol, Symbol> bd = new LinkedHashMap<>(bindings);
              bd.put(ref, d);
              return assess(b2h(bd), prop) < GENTAU;
            })
            .collect(Collectors.toList());

        log.debug("DISTRACTORS ELIMINATED: " + elimDist);

        if (!elimDist.isEmpty() || distractors.isEmpty()) {
          List<Variable> boundVarList = bindings.entrySet().stream()
              .map(e -> new Variable(e.getKey().getName(), e.getValue().getName()))
              .collect(Collectors.toList());
          description.add(prop.boundForm(boundVarList));
          distractors = distractors.stream().filter(d -> !elimDist.contains(d)).collect(Collectors.toList());
          log.debug("DESCRIPTION: " + description);
          log.debug("DISTRACTORS: " + distractors);
        }
      }
    }
    return Optional.of(new STMResult(ref, description, distractors));
  }

  private List<Symbol> refsIn(List<Property> props) {
    return props.stream().flatMap(p -> refsInS(p.predicateForm().get(0)).stream()).collect(Collectors.toList());
  }

  private List<Symbol> refsInS(Symbol s) {
    if (s instanceof Variable) {
      Variable v = (Variable) s;
      log.debug("refsInS: considering: " + v.getName() + " - " + v.getType());
      if (v.getType().contains("_")) {
        String type = v.getType();
        return List.of(Factory.createSymbol(type + ":" + type.split("_")[0]));
      } else {
        return new ArrayList<>();
      }
    } else if (s instanceof Term) {
      return ((Term) s).getArgs().stream().flatMap(a -> refsInS(a).stream()).collect(Collectors.toList());
    } else {
      return new ArrayList<>();
    }
  }

  private Optional<Map.Entry<Symbol, List<Property>>> generateRETuple(
      Symbol ref, List<Symbol> found, STMResult stmResult) {
    String consultantName = ref.getType();
    log.debug("CONSULTANTNAME: " + consultantName);

    Optional<ConsultantInfo> consultantOpt = consultants.stream()
        .filter(c -> c.kbName.equalsIgnoreCase(consultantName))
        .findFirst();
    if (!consultantOpt.isPresent()) return Optional.empty();
    ConsultantInfo consultant = consultantOpt.get();

    List<Property> description = stmResult.description;
    List<Symbol> distractors = new ArrayList<>(stmResult.distractors);

    List<Property> propsList;
    try {
      List<Term> handled = TRADE.getAvailableService(
          new TRADEServiceConstraints().name("getPropertiesHandled").argTypes()
              .inGroups(consultant.tsi.getGroups().toArray(new String[0])))
          .call(List.class);
      propsList = handled.stream().map(Property::new).collect(Collectors.toList());
    } catch (Exception e) {
      log.error("[generateRETuple] exception getting propertiesHandled", e);
      return Optional.empty();
    }

    Deque<Map.Entry<Property, Map<Symbol, Symbol>>> props = new ArrayDeque<>();
    for (Property p : propsList) {
      props.push(new AbstractMap.SimpleEntry<>(p, new LinkedHashMap<>()));
    }

    log.debug("PROPS: " + props);

    while (!props.isEmpty() && !distractors.isEmpty()) {
      log.debug("Distractors: " + distractors + "; P: " + props);
      Map.Entry<Property, Map<Symbol, Symbol>> entry = props.pop();
      Property prop = entry.getKey();
      Map<Symbol, Symbol> bindings = entry.getValue();
      log.debug("TRYING: " + prop);

      List<Symbol> args = ((Term) prop.predicateForm().get(0)).getArgs();
      log.debug("args: " + args);

      if (args.size() > 1 && bindings.isEmpty()) {
        log.debug("FOLDING ON ACB");
        List<Symbol> candidateVariables = args.stream()
            .filter(v -> v instanceof Variable && v.getType().equalsIgnoreCase(consultantName))
            .collect(Collectors.toList());
        log.debug("candidate variables: " + candidateVariables);

        for (Symbol cv : candidateVariables) {
          List<Symbol> otherArgs = args.stream()
              .filter(x -> !x.getName().equals(cv.getName()))
              .collect(Collectors.toList());
          List<Map<Symbol, Symbol>> combos = allComboBindings(otherArgs);
          for (Map<Symbol, Symbol> b : combos) {
            props.push(new AbstractMap.SimpleEntry<>(prop, b));
          }
        }
      } else {
        log.debug("BINDINGS: " + bindings + " /// FOUND: " + found);
        List<Symbol> bindingValues = new ArrayList<>(bindings.values());
        bindingValues.retainAll(found);
        if (!bindingValues.isEmpty()) {
          log.debug("Refers to something already described, dropping");
        } else {
          log.debug("LINE6 -- " + prop + " -- " + bindings);
          log.debug("args: " + args);
          Optional<Symbol> missingOpt = args.stream().filter(s -> !bindings.containsKey(s)).findFirst();
          if (!missingOpt.isPresent()) {
            throw new RuntimeException("No variables left unfilled?");
          }
          Symbol missing = missingOpt.get();
          log.debug("Missing: " + missing);

          Map<Symbol, Symbol> testBindings = new LinkedHashMap<>(bindings);
          testBindings.put(missing, ref);
          if (assess(b2h(testBindings), prop) >= GENTAU) {
            log.debug("IT'S A MATCH!");
            final List<Symbol> distractorsCopy = new ArrayList<>(distractors);
            List<Symbol> elimDist = distractorsCopy.stream()
                .filter(did -> {
                  Map<Symbol, Symbol> bd = new LinkedHashMap<>(bindings);
                  bd.put(missing, did);
                  return assess(b2h(bd), prop) < GENTAU;
                })
                .collect(Collectors.toList());
            log.debug("DISTRACTORS ELIMINATED: " + elimDist);
            if (!elimDist.isEmpty() || distractors.isEmpty()) {
              List<Variable> boundVarList = bindings.entrySet().stream()
                  .map(e -> new Variable(e.getKey().getName(), e.getValue().getType()))
                  .collect(Collectors.toList());
              description.add(prop.boundForm(boundVarList));
              distractors = distractors.stream().filter(d -> !elimDist.contains(d)).collect(Collectors.toList());
              log.debug("DESCRIPTION: " + description);
              log.debug("DISTRACTORS: " + distractors);
            }
          }
        }
      }
    }
    return Optional.of(new AbstractMap.SimpleEntry<>(ref, description));
  }

  //========================================================================
  // Greedy Best First Search
  //========================================================================

  private List<Hypothesis> bfs(List<Property> query, List<Hypothesis> initialHyps) {
    List<Hypothesis> solutions = new ArrayList<>();
    PriorityQueue<Map.Entry<Hypothesis, List<Property>>> hypothesisQueue =
        new PriorityQueue<>(Comparator.comparingDouble(e -> -e.getKey().likelihood()));

    if (initialHyps.isEmpty()) {
      initializeHQ(query, hypothesisQueue);
    } else {
      for (Hypothesis h : initialHyps) {
        hypothesisQueue.add(new AbstractMap.SimpleEntry<>(h, new ArrayList<>(query)));
      }
    }

    while (!hypothesisQueue.isEmpty()) {
      Map.Entry<Hypothesis, List<Property>> n = hypothesisQueue.poll();
      if (!n.getValue().isEmpty()) {
        Optional<Variable> nv = newVariable(n.getKey(), n.getValue());
        if (nv.isPresent()) {
          expandVariable(n.getKey(), n.getValue(), nv.get(), hypothesisQueue);
        } else {
          applyConstraint(n.getKey(), n.getValue(), hypothesisQueue, solutions);
        }
      } else {
        solutions.add(n.getKey());
      }
    }

    log.debug("Solutions: " + solutions);
    return solutions;
  }

  private void initializeHQ(List<Property> query,
      PriorityQueue<Map.Entry<Hypothesis, List<Property>>> hypothesisQueue) {
    if (query.isEmpty()) throw new RuntimeException("Couldn't get the head variable for empty query");
    List<Variable> vars = query.get(0).predicateForm().getOrderedVars();
    if (vars.isEmpty()) throw new RuntimeException("Couldn't get the head variable for " + query.get(0));
    Variable firstVar = vars.get(0);
    try {
      List<Symbol> domain = initialDomain(firstVar.getType());
      for (Symbol consultantAndId : domain) {
        log.trace("Initial Domain of " + firstVar + " includes " + consultantAndId);
        Map<Variable, Symbol> newBuffer = new HashMap<>();
        newBuffer.put(firstVar, consultantAndId);
        hypothesisQueue.add(new AbstractMap.SimpleEntry<>(
            new Hypothesis(newBuffer, 1.0), new ArrayList<>(query)));
      }
    } catch (Exception e) {
      throw new RuntimeException("Couldn't initialize HQ: " + e.getMessage(), e);
    }
  }

  private Optional<Variable> newVariable(Hypothesis h, List<Property> mb) {
    Set<String> assignedVarNames = h.assignments().keySet().stream()
        .map(Variable::getName).collect(Collectors.toSet());
    return mb.get(0).predicateForm().getVars().stream()
        .filter(v -> !assignedVarNames.contains(v.getName()))
        .findFirst();
  }

  private void expandVariable(Hypothesis h, List<Property> mb, Variable v,
      PriorityQueue<Map.Entry<Hypothesis, List<Property>>> hypothesisQueue) {
    try {
      List<Symbol> domain = initialDomain(v.getType());
      for (Symbol cAndId : domain) {
        log.trace("Expanding variable " + v + " with candidate " + cAndId);
        Map<Variable, Symbol> newAssignments = new HashMap<>(h.assignments());
        newAssignments.put(v, cAndId);
        hypothesisQueue.add(new AbstractMap.SimpleEntry<>(
            new Hypothesis(newAssignments, h.likelihood()), mb));
      }
    } catch (Exception e) {
      log.error("[expandVariable] exception", e);
    }
  }

  private void applyConstraint(Hypothesis h, List<Property> mb,
      PriorityQueue<Map.Entry<Hypothesis, List<Property>>> hypothesisQueue,
      List<Hypothesis> solutions) {
    double newLikelihood = assess(h, mb.get(0));
    List<Property> mbTail = mb.subList(1, mb.size());
    if (newLikelihood > TAU) {
      if (mbTail.isEmpty()) {
        solutions.add(new Hypothesis(h.assignments(), newLikelihood));
      } else {
        hypothesisQueue.add(new AbstractMap.SimpleEntry<>(
            new Hypothesis(h.assignments(), newLikelihood), mbTail));
      }
    }
  }

  //========================================================================
  // Private helper classes
  //========================================================================

  private static class SolutionsResult {
    final List<Hypothesis> solutions;
    final List<Variable> remainingVars;

    SolutionsResult(List<Hypothesis> solutions, List<Variable> remainingVars) {
      this.solutions = solutions;
      this.remainingVars = remainingVars;
    }
  }

  private static class STMResult {
    final Symbol ref;
    final List<Property> description;
    final List<Symbol> distractors;

    STMResult(Symbol ref, List<Property> description, List<Symbol> distractors) {
      this.ref = ref;
      this.description = description;
      this.distractors = distractors;
    }
  }
}
