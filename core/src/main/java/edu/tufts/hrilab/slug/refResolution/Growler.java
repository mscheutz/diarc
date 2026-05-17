/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

/**
 * Growler: Givenness and Relevance Theoretic Open-World Entity Resolution
 */
public class Growler {

  public static final double RELEVANCE_THRESHOLD = 0.2;
  public static final double PROBABILITY_THRESHOLD = 0.2;
  public static final double DEFAULT_RELEVANCE = 0.0;
  public static final double[] RELEVANCE_WEIGHTINGS = {10.0, 5.0, 2.0, 1.0};

  private final Resolver resolver;
  private final ReferenceResolutionComponent.GHSelfModel gh;
  private final Map<Symbol, EntityScore> relevanceMap;
  final Logger log = LoggerFactory.getLogger(getClass());

  public Growler(Resolver resolver, ReferenceResolutionComponent.GHSelfModel gh, Map<Symbol, EntityScore> relevanceMap) {
    this.resolver = resolver;
    this.gh = gh;
    this.relevanceMap = relevanceMap;
  }

  // ====== Static companion object methods ======

  public static double scaleRelevance(double x) {
    return (1.0 / RELEVANCE_WEIGHTINGS.length) / (1.0 + Math.exp(-0.5 * x)) * 2.0 - (1.0 / RELEVANCE_WEIGHTINGS.length);
  }

  public static double weightRelevance(EntityScore score) {
    double[] relevanceVector = {score.inMainClause() ? 1.0 : 0.0, score.synProm(), score.recency(), score.bonus()};
    double result = 0.0;
    for (int i = 0; i < relevanceVector.length; i++) {
      result += scaleRelevance(relevanceVector[i] * RELEVANCE_WEIGHTINGS[i]);
    }
    return result;
  }

  public static Hypothesis cmclToHypothesis(CrossMappingCandidateList c) {
    Map<Variable, Symbol> assignments = new HashMap<>();
    for (RelevanceTheoreticBinding b : c.bindings()) {
      assignments.put(b.variable(), b.candidate().ref());
    }
    return new Hypothesis(assignments, c.probability());
  }

  public static List<CrossMappingCandidateList> generateCrossMappingCandidateList(SingleVarCandidateList svc) {
    List<CrossMappingCandidateList> result = new ArrayList<>();
    for (RelevanceTheoreticCandidateWithProabability c : svc.getCandidates()) {
      List<RelevanceTheoreticBinding> bindings = new ArrayList<>();
      bindings.add(new RelevanceTheoreticBinding(svc.variable(), c.candidate()));
      result.add(new CrossMappingCandidateList(bindings, c.probability()));
    }
    return result;
  }

  public static List<CrossMappingCandidateList> mergeCrossMappingCandidateLists(
      List<CrossMappingCandidateList> first, List<CrossMappingCandidateList> second) {
    List<CrossMappingCandidateList> result = new ArrayList<>();
    for (CrossMappingCandidateList c1 : first) {
      for (CrossMappingCandidateList c2 : second) {
        List<RelevanceTheoreticBinding> merged = new ArrayList<>(c1.bindings());
        merged.addAll(c2.bindings());
        result.add(new CrossMappingCandidateList(merged, c1.probability() * c2.probability()));
      }
    }
    return result;
  }

  // ====== Instance methods ======

  public LinkedList<String> getMnemonicActions(Symbol ghTier) {
    switch (ghTier.getName()) {
      case "INFOCUS":
        return new LinkedList<>(List.of("INFOCUS"));
      case "ACTIVATED":
        return new LinkedList<>(List.of("ACTIVATED", "INFOCUS"));
      case "REFACTIVATED":
        return new LinkedList<>(List.of("ACTIVATED", "INFOCUS", "POSIT"));
      case "FAMILIAR":
        return new LinkedList<>(List.of("ACTIVATED", "INFOCUS", "FAMILIAR", "LTM"));
      case "UNFAMILIAR":
        return new LinkedList<>(List.of("UNFAMILIAR"));
      case "DEFINITE":
        return new LinkedList<>(List.of("ACTIVATED", "INFOCUS", "FAMILIAR", "LTMP"));
      case "INDEFINITE":
        return new LinkedList<>(List.of("POSIT"));
      default:
        log.error("invalid GhTier when getting Mnemonic Actions " + ghTier);
        return new LinkedList<>();
    }
  }

  public List<Symbol> domain(String mnemonicAction) {
    switch (mnemonicAction) {
      case "INFOCUS":
        return new ArrayList<>(gh.focBuffer);
      case "ACTIVATED":
        return new ArrayList<>(gh.actBuffer);
      case "FAMILIAR":
        return new ArrayList<>(gh.famBuffer);
      default:
        return Collections.singletonList(Factory.createSymbol("?_0"));
    }
  }

  public double getRelevance(Symbol ref) {
    EntityScore result = relevanceMap.get(ref);
    if (result != null) {
      log.trace("weight relevance: " + ref + " " + result + " " + weightRelevance(result));
      return weightRelevance(result);
    } else {
      log.trace("default relevance: " + ref + " " + relevanceMap);
      return DEFAULT_RELEVANCE;
    }
  }

  public double getProbabilityofSatisfyingUnaryPredicates(RelevanceTheoreticBinding binding, List<Term> allTerms) {
    if (binding.candidate().ref().equals(Factory.createSymbol("?_0"))) return 1.0;
    List<Term> relevantPredicates = allTerms.stream()
        .filter(term -> term.getOrderedVars().stream().anyMatch(v -> v.getName().equals(binding.variable().getName()))
            && term.getOrderedVars().size() == 1)
        .collect(Collectors.toList());
    if (relevantPredicates.isEmpty()) return 1.0;
    List<Property> props = relevantPredicates.stream().map(Property::new).collect(Collectors.toList());
    return getProbabilityOfSatisfyingProperties(List.of(binding), props);
  }

  public double getProbabilityOfSatisfyingProperties(List<RelevanceTheoreticBinding> relevantBindings, List<Property> props) {
    Map<Variable, Symbol> assignmentsMap = new HashMap<>();
    for (RelevanceTheoreticBinding r : relevantBindings) {
      assignmentsMap.put(r.variable(), r.candidate().ref());
    }
    Hypothesis hyp = new Hypothesis(assignmentsMap, 1.0);

    List<Variable> T;
    try {
      T = resolver.getVariableMapping(props);
    } catch (Exception e) {
      log.warn("Failed to get variable mapping. Assuming zero probability. ");
      return 0.0;
    }

    List<Property> boundProps = props.stream().map(p -> p.boundForm(T)).collect(Collectors.toList());

    double product = 1.0;
    for (Property boundProp : boundProps) {
      double assessed;
      try {
        assessed = resolver.assess(hyp, boundProp);
      } catch (Exception e) {
        log.debug("Failed to get probability from assess. Assuming zero probability. ");
        return 0.0;
      }
      product *= assessed;
    }
    return product;
  }

  public double getProbabilityOfSatisfyingPolyadicProperties(List<RelevanceTheoreticBinding> bindings, List<Term> allTerms) {
    List<RelevanceTheoreticBinding> relevantBindings = bindings.stream()
        .filter(b -> !b.candidate().ref().equals(Factory.createSymbol("?_0")))
        .collect(Collectors.toList());
    if (relevantBindings.isEmpty()) return 1.0;

    List<Term> relevantPredicates = allTerms.stream()
        .filter(term -> bindings.stream().noneMatch(
            binding -> term.getOrderedVars().contains(binding.variable()) && term.getOrderedVars().size() == 1))
        .collect(Collectors.toList());
    if (relevantPredicates.isEmpty()) return 1.0;

    List<Property> props = relevantPredicates.stream().map(Property::new).collect(Collectors.toList());
    return getProbabilityOfSatisfyingProperties(relevantBindings, props);
  }

  public void generateInitialValidCandidateList(SingleVarCandidateList svcl, List<Term> allTerms) {
    log.debug("svcl: " + svcl);
    log.debug("generateInitialValidCandidateList " + svcl.variable() + " " + svcl.getTiers() + " " + svcl.getCandidates());

    while (svcl.getCandidates().stream().noneMatch(c -> c.candidate().relevance() >= RELEVANCE_THRESHOLD)
        && !svcl.getTiers().isEmpty()
        && !svcl.getTiers().peekFirst().contains("LTM")) {

      List<RelevanceTheoreticCandidateWithProabability> cand = new ArrayList<>();
      for (Symbol ref : domain(svcl.getTiers().peekFirst())) {
        RelevanceTheoreticCandidate rtc = new RelevanceTheoreticCandidate(ref, getRelevance(ref));
        RelevanceTheoreticBinding binding = new RelevanceTheoreticBinding(svcl.variable(), rtc);
        double prob = getProbabilityofSatisfyingUnaryPredicates(binding, allTerms);
        log.trace("rtcwp: " + new RelevanceTheoreticCandidateWithProabability(rtc, prob));
        if (prob >= PROBABILITY_THRESHOLD) {
          cand.add(new RelevanceTheoreticCandidateWithProabability(rtc, prob));
        }
      }
      log.debug("mnemonic action: {}", svcl.getTiers().peekFirst());
      log.debug("cand: {}", cand);
      svcl.getCandidates().addAll(cand);
      svcl.getCandidates().sort(Comparator.comparingDouble(c -> -c.candidate().relevance()));
      svcl.getTiers().removeFirst();
    }
  }

  public List<CrossMappingCandidateList> generateFullTable(List<SingleVarCandidateList> svcls, List<Term> allTerms) {
    log.debug("generating full table. svcls:" + svcls + " allTerms: " + allTerms);
    if (svcls.isEmpty()) return new ArrayList<>();

    List<List<CrossMappingCandidateList>> mapped = svcls.stream()
        .map(Growler::generateCrossMappingCandidateList)
        .collect(Collectors.toList());

    List<CrossMappingCandidateList> accumulated = mapped.get(0);
    for (int i = 1; i < mapped.size(); i++) {
      accumulated = mergeCrossMappingCandidateLists(accumulated, mapped.get(i));
      accumulated = accumulated.stream()
          .filter(c -> c.probability() >= PROBABILITY_THRESHOLD)
          .collect(Collectors.toList());
    }

    List<CrossMappingCandidateList> result = new ArrayList<>();
    for (CrossMappingCandidateList m : accumulated) {
      double newProb = m.probability() * getProbabilityOfSatisfyingPolyadicProperties(m.bindings(), allTerms);
      result.add(new CrossMappingCandidateList(m.bindings(), newProb));
    }
    return result.stream()
        .filter(c -> c.probability() >= PROBABILITY_THRESHOLD)
        .collect(Collectors.toList());
  }

  /**
   * The GROWLER algorithm.
   *
   * @param semantics The set of semantic constraints to use during resolution
   * @param statuses  The set of status cue mappings for each variable used in those constraints.
   * @return
   */
  public List<Hypothesis> resolve_clause(List<Term> semantics, Map<Variable, Symbol> statuses) {
    log.debug("[resolve_clause] IN RESOLVE CLAUSE");

    // Create a copy so we can modify the list of semantics
    List<Term> allTerms = new ArrayList<>();
    for (Term t : semantics) {
      allTerms.add((Term) t.clone());
    }
    log.debug("[resolve_clause] ALL TERMS: {}", allTerms);

    Set<Variable> allVars = new HashSet<>(statuses.keySet());
    log.debug("[resolve_clause] ALL VARS: {}", allVars);

    Map<Variable, LinkedList<String>> allPlans = new HashMap<>();
    for (Map.Entry<Variable, Symbol> entry : statuses.entrySet()) {
      allPlans.put(entry.getKey(), getMnemonicActions(entry.getValue()));
    }
    log.debug("[resolve_clause] ALL PLANS: {}", allPlans);

    List<SingleVarCandidateList> singleVarCandSets = new ArrayList<>();
    for (Map.Entry<Variable, LinkedList<String>> entry : allPlans.entrySet()) {
      singleVarCandSets.add(new SingleVarCandidateList(entry.getKey(), entry.getValue(), new ArrayList<>()));
    }

    // Adding unfamiliar property to the variable
    for (Map.Entry<Variable, LinkedList<String>> entry : allPlans.entrySet()) {
      if (entry.getValue().contains("UNFAMILIAR")) {
        Term t = new Term("unfamiliar", entry.getKey());
        allTerms.add(0, t);
      }
    }

    singleVarCandSets.forEach(svcs -> generateInitialValidCandidateList(svcs, allTerms));
    log.debug("[resolve_clause] SVCS: {}", singleVarCandSets);

    List<CrossMappingCandidateList> fulltable = generateFullTable(singleVarCandSets, allTerms);
    log.debug("[resolve_clause] FULL TABLE: {}", fulltable);

    Set<Variable> vars_with_help_needed = allVars.stream()
        .filter(v -> fulltable.stream().noneMatch(entry ->
            entry.bindings().stream().anyMatch(m ->
                m.variable().equals(v) && m.candidate().relevance() >= RELEVANCE_THRESHOLD)))
        .collect(Collectors.toSet());
    log.debug("[resolve_clause] VWHN: {}", vars_with_help_needed);

    for (SingleVarCandidateList s : singleVarCandSets) {
      log.debug("s: {} vwhn: {} vwhn contains?: {} tiers: {} tiers exists: {}",
          s, vars_with_help_needed, vars_with_help_needed.contains(s.variable()),
          s.getTiers(), s.getTiers().stream().anyMatch(t -> !t.contains("LTM")));
    }

    while (singleVarCandSets.stream().anyMatch(s ->
        vars_with_help_needed.contains(s.variable()) && s.getTiers().stream().anyMatch(t -> !t.contains("LTM")))) {

      for (SingleVarCandidateList svcs : singleVarCandSets) {
        if (vars_with_help_needed.contains(svcs.variable()) && svcs.getTiers().stream().anyMatch(t -> !t.contains("LTM"))) {
          log.debug("[resolve_clause] CONSIDERING: {}", svcs);

          SingleVarCandidateList updateSet = new SingleVarCandidateList(svcs.variable(), svcs.getTiers(), new ArrayList<>());
          generateInitialValidCandidateList(updateSet, allTerms);

          List<SingleVarCandidateList> replacementList = new ArrayList<>();
          for (SingleVarCandidateList x : singleVarCandSets) {
            if (x.variable().getName().equals(svcs.variable().getName())) {
              replacementList.add(updateSet);
            } else {
              replacementList.add(x);
            }
          }

          List<CrossMappingCandidateList> updateTable = generateFullTable(replacementList, allTerms);
          fulltable.addAll(updateTable);

          singleVarCandSets.stream()
              .filter(x -> x.variable().equals(svcs.variable()))
              .findFirst()
              .ifPresent(x -> x.getCandidates().addAll(updateSet.getCandidates()));

          Set<Variable> finalVars_with_help_needed = vars_with_help_needed;
          vars_with_help_needed = allVars.stream()
              .filter(v -> fulltable.stream().noneMatch(entry ->
                  entry.bindings().stream().anyMatch(m ->
                      m.variable().equals(v) && m.candidate().relevance() >= RELEVANCE_THRESHOLD)))
              .collect(Collectors.toSet());
        }
      }
    }

    // Build fulltableWithRelevance
    List<Map.Entry<CrossMappingCandidateList, Double>> fulltableWithRelevance = new ArrayList<>();
    for (CrossMappingCandidateList x : fulltable) {
      double relevanceSum = x.bindings().stream().mapToDouble(b -> b.candidate().relevance()).sum();
      fulltableWithRelevance.add(new AbstractMap.SimpleEntry<>(x, relevanceSum));
    }
    fulltableWithRelevance.sort((a, b) -> Double.compare(b.getValue(), a.getValue()));

    log.debug("full table with relevance: {}", fulltableWithRelevance);

    OptionalDouble maxRelevanceOpt = fulltableWithRelevance.stream().mapToDouble(Map.Entry::getValue).max();
    double prefixThreshold = maxRelevanceOpt.isPresent() ? maxRelevanceOpt.getAsDouble() / 2.0 : 0.0;
    final double RELEVANCE_PREFIX_THRESHOLD = prefixThreshold;

    List<CrossMappingCandidateList> toret = fulltableWithRelevance.stream()
        .filter(e -> e.getValue() >= RELEVANCE_PREFIX_THRESHOLD)
        .map(Map.Entry::getKey)
        .collect(Collectors.toList());

    List<Hypothesis> hyps = toret.stream().map(Growler::cmclToHypothesis).collect(Collectors.toList());
    if (hyps.isEmpty()) {
      hyps = new ArrayList<>(List.of(new Hypothesis(new HashMap<>(), 1.0)));
    }

    Set<Variable> finalVars_with_help_needed = vars_with_help_needed;

    List<SingleVarCandidateList> ltmNoPositSVCSs = singleVarCandSets.stream()
        .filter(s -> finalVars_with_help_needed.contains(s.variable())
            && !s.getTiers().isEmpty() && "LTM".equals(s.getTiers().peekFirst()))
        .collect(Collectors.toList());

    List<SingleVarCandidateList> ltmPositSVCSs = singleVarCandSets.stream()
        .filter(s -> finalVars_with_help_needed.contains(s.variable())
            && ((!s.getTiers().isEmpty() && "LTMP".equals(s.getTiers().peekFirst())) || s.getTiers().isEmpty()))
        .collect(Collectors.toList());

    List<Variable> ltmNoPositVars = ltmNoPositSVCSs.stream().map(SingleVarCandidateList::variable).collect(Collectors.toList());
    List<Variable> ltmPositVars = ltmPositSVCSs.stream().map(SingleVarCandidateList::variable).collect(Collectors.toList());

    List<Term> ltmNoPositTerms = allTerms.stream()
        .filter(term -> term.getArgs().stream().anyMatch(arg ->
            arg instanceof Variable && ltmNoPositVars.stream().anyMatch(x -> x.getName().equals(((Variable) arg).getName()))))
        .collect(Collectors.toList());

    List<Term> ltmPositTerms = allTerms.stream()
        .filter(term -> term.getArgs().stream().anyMatch(arg ->
            arg instanceof Variable && ltmPositVars.stream().anyMatch(x -> x.getName().equals(((Variable) arg).getName()))))
        .collect(Collectors.toList());

    log.debug("[resolve_clause] LTM VARS: " + ltmNoPositVars + " / " + ltmPositVars
        + ", LTM TERMS: " + ltmNoPositTerms + " / " + ltmPositTerms);

    List<Hypothesis> results = new ArrayList<>();
    for (Hypothesis hyp : hyps) {
      results.addAll(resolve_from_LTM(ltmNoPositTerms, ltmNoPositVars, hyp, false));
    }
    log.debug("initial results : {}", results);
    // Note: keeping the Scala behavior - filter is called but result not reassigned (subtle bug preserved)
    results.stream().filter(h -> h.likelihood() >= PROBABILITY_THRESHOLD).collect(Collectors.toList());
    if (results.isEmpty()) {
      results = new ArrayList<>(List.of(new Hypothesis(new HashMap<>(), 1.0)));
    }
    List<Hypothesis> tempResults = new ArrayList<>();
    for (Hypothesis hyp : results) {
      tempResults.addAll(resolve_from_LTM(ltmPositTerms, ltmPositVars, hyp, true));
    }
    results = tempResults.stream().filter(h -> h.likelihood() >= PROBABILITY_THRESHOLD).collect(Collectors.toList());

    if (results.isEmpty()) {
      results = new ArrayList<>(List.of(new Hypothesis(new HashMap<>(), 1.0)));
    }

    log.debug("[resolve_clause] intermediary results: {}", results);
    if (results.size() == 1) {
      log.debug("[resolve_clause] >>> positing results");
      Hypothesis resHyp = results.get(0);
      log.debug("resHyp: {}", resHyp);

      List<Variable> indefVars = resHyp.assignments().entrySet().stream()
          .filter(e -> e.getValue().getName().contains("?"))
          .map(Map.Entry::getKey)
          .collect(Collectors.toList());
      log.debug("[resolve_clause] >>> >>> indefVars: {}", indefVars);

      if (!indefVars.isEmpty()) {
        List<Term> indefTerms = allTerms.stream()
            .filter(term -> term.getArgs().stream().anyMatch(arg ->
                arg instanceof Variable && indefVars.stream().anyMatch(x -> x.getName().equals(((Variable) arg).getName()))))
            .collect(Collectors.toList());
        log.debug("indef terms: {}", indefTerms);

        Map<Variable, Symbol> knownAssignments = new HashMap<>();
        for (Map.Entry<Variable, Symbol> e : resHyp.assignments().entrySet()) {
          if (!e.getValue().getName().contains("?")) {
            knownAssignments.put(e.getKey(), e.getValue());
          }
        }
        Hypothesis knownHyp = new Hypothesis(knownAssignments, resHyp.likelihood());
        log.debug("known Hyps: {}", knownHyp);
        results = posit_to_LTM(indefTerms, indefVars, knownHyp);
      }
    }

    log.debug("[resolve_clause] final results: " + results);
    return results;
  }

  public List<Hypothesis> resolve_from_LTM(List<Term> terms, List<Variable> vars, Hypothesis initialHyp, boolean posit) {
    log.debug("[resolve_from_LTM]: {} {} {} {}", terms, vars, initialHyp, posit);

    Map<Variable, Symbol> indefBinds = new HashMap<>();
    Map<Variable, Symbol> knownBinds = new HashMap<>();
    for (Map.Entry<Variable, Symbol> entry : initialHyp.assignments().entrySet()) {
      if (entry.getValue().getName().contains("?")) {
        indefBinds.put(entry.getKey(), entry.getValue());
      } else {
        knownBinds.put(entry.getKey(), entry.getValue());
      }
    }
    log.debug("[resolve_from_LTM] indefBinds: {} knownBinds: {}", indefBinds, knownBinds);
    Hypothesis knownHyp = new Hypothesis(knownBinds, initialHyp.likelihood());

    List<Property> termsPP = terms.stream().map(Property::new).collect(Collectors.toList());

    if (termsPP.isEmpty()) {
      return List.of(knownHyp);
    }

    List<Hypothesis> resHyps = resolver.resolve(termsPP, vars, List.of(knownHyp), posit);
    log.debug("[resolve_from_LTM] before hypothesis construction: {}", resHyps);
    // Note: Scala code creates new hyps but returns original resHyps (subtle behavior preserved)
    resHyps.stream().map(h -> {
      Map<Variable, Symbol> merged = new HashMap<>(h.assignments());
      merged.putAll(indefBinds);
      return new Hypothesis(merged, h.likelihood());
    }).collect(Collectors.toList());
    log.debug("[resolve_from_LTM] after hypothesis construction: {}", resHyps);
    return resHyps;
  }

  public List<Hypothesis> posit_to_LTM(List<Term> terms, List<Variable> varNames, Hypothesis initialHyp) {
    log.debug("        @posit_to_LTM: " + terms + " " + varNames + " " + initialHyp);

    List<Property> termsPP = terms.stream().map(Property::new).collect(Collectors.toList());
    List<Variable> T = resolver.getVariableMapping(termsPP);
    List<Property> bTermsPP = termsPP.stream().map(p -> p.boundForm(T)).collect(Collectors.toList());

    if (termsPP.isEmpty()) {
      return new ArrayList<>();
    } else {
      return resolver.completeSolutions(List.of(initialHyp), varNames, bTermsPP);
    }
  }
}
