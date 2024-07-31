/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.pragmatics;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.belief.provers.clingo.ClingoProver;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.fol.util.PragUtil;
import edu.tufts.hrilab.slug.common.UtteranceType;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.*;

import edu.tufts.hrilab.belief.provers.*;
import edu.tufts.hrilab.belief.provers.prolog.Prolog;
import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import ai.thinkingrobots.trade.*;

public class PragmaticsComponent extends DiarcComponent implements PragmaticsInterface {
  final protected Logger log = LoggerFactory.getLogger(this.getClass());

  protected List<PragRuleProverSet> ruleSets = new ArrayList<>();
  protected List<PragRuleProverSet> genRuleSets = new ArrayList<>();

  /**
   * Relevant query terms collected from all rule sets' environmental context constraints. These
   * only change if the rule sets change.
   */
  protected Set<Term> relevantQueryTerms = new HashSet<>();
  /**
   * The set of relevantQueryTerms that have support from Belief. This is updated during each
   * call to pragmatics from an outside component to reflect the latest state of Belief.
   */
  protected Set<Term> relevantBeliefs = new HashSet<>();
  /**
   * The dialogue history from DialogueComponent. This is updated during each
   * call to pragmatics from an outside component to reflect the latest state of Dialogue.
   */
  protected List<Utterance> dialogHistory =  new ArrayList<>();

  protected List<String> pragRulesFiles = new ArrayList<>();
  protected List<String> genRulesFiles = new ArrayList<>();

  // use these default rules files if none are provided
  private String defaultPragRules = "demos.prag";
  private String defaultGenRules = "demosgen.prag";

  // default resource directory to load rules files
  protected String resourceConfigPath = "config/edu/tufts/hrilab/slug/pragmatics";

  private Prover.Engine proverType = Prover.Engine.PROLOG;
  private Prover prover;

  //set default values
  protected void init() {
    // load prag rules
    if (!pragRulesFiles.isEmpty()) {
      pragRulesFiles.forEach(file -> ruleSets.addAll(loadPragRules(Resources.createFilepath(resourceConfigPath, file))));
    } else {
      ruleSets = loadPragRules(Resources.createFilepath(resourceConfigPath, defaultPragRules));
    }

    // load gen prag rules
    if (!genRulesFiles.isEmpty()) {
      genRulesFiles.forEach(file -> genRuleSets.addAll(loadPragRules(Resources.createFilepath(resourceConfigPath, file))));
    } else {
      genRuleSets = loadPragRules(Resources.createFilepath(resourceConfigPath, defaultGenRules));
    }

    // Instantiate prover
    switch (proverType) {
      case PROLOG:
        prover = new Prolog();
        break;
      case CLINGO:
        prover = new ClingoProver();
        break;
      default:
        log.error("Cannot instantiate prover " + proverType + ". Defaulting to prolog.");
        prover = new Prolog();
    }
  }

  public PragmaticsComponent() {
    super();
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("pragrules").hasArgs().argName("file(s)").desc("load pragmatic rules from file(s)").build());
    options.add(Option.builder("genrules").hasArgs().argName("file(s)").desc("load pragmatic generation rules from file(s)").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("pragrules")) {
      pragRulesFiles.addAll(Arrays.asList(cmdLine.getOptionValues("pragrules")));
    }
    if (cmdLine.hasOption("genrules")) {
      genRulesFiles.addAll(Arrays.asList(cmdLine.getOptionValues("genrules")));
    }
  }

  protected List<PragRuleProverSet> loadPragRules(String path) {
    List<PragRuleProverSet> ruleSets = new ArrayList<>();

    try {
      InputStream in = getClass().getResourceAsStream(path);
      BufferedReader br = new BufferedReader(new InputStreamReader(in));

      String line = br.readLine();
      int stage = 0;
      PragRuleProverSet prs = new PragRuleProverSet();

      while (line != null) {
        if (!line.startsWith("#")) {
          switch (stage) {
            case 0:
              if (line.trim().startsWith("NEW-SET")) {
                prs = new PragRuleProverSet();
                stage = 1;
              }
              break;
            case 1:
              if (line.indexOf('(') >= 0) {
                prs.parseContext(line);
              } else if (line.startsWith("RULES")) {
                stage = 2;
              }
              break;
            case 2:
              log.debug(line);
              if (line.trim().startsWith("END-SET")) {
                ruleSets.add(prs);
                stage = 0;
              } else if (line.contains(":=")) {
                log.debug("newrule");
                PragRuleProver pr = new PragRuleProver(line);
                prs.add(pr);
              }
              break;
          }
        }
        line = br.readLine();
      }
    } catch (IOException ioe) {
      log.error("Problem parsing prag file.", ioe);
    }

    ////// get relevant belief terms (to periodically query)
    for (PragRuleProverSet prs : ruleSets) {
      if (!prs.getEnvContext().isEmpty()) {
        relevantQueryTerms.addAll(prs.getEnvContext());
      }
    }
    return ruleSets;
  }

  @Override
  public Utterance applyPragmaticMeaning(Utterance utterance) {
    log.debug("[applyPragmaticMeaning] utterance received: " + utterance);
    updateRelevantBeliefsAndDialogueHistory();
    if (utterance.isInputUtterance()) {
      applyInputPragmaticMeaning(utterance);
    } else {
      applyOutputPragmaticMeaning(utterance);
    }
    log.info("[applyPragmaticMeaning] direct meaning: {} indirect meaning: {}",
            utterance.getBoundSemanticsPredicate(),utterance.getBoundIndirectSemanticsPredicate());
    return utterance;
  }

  public void applyInputPragmaticMeaning(Utterance utterance) {

    List<Term> indirectMeaning = utterance.getIndirectSemantics();
    Symbol semantics = utterance.getSemantics();

    if (!initialPragmaticsFound(utterance)) {
      Pair<Set<Term>, Set<Term>> meanings = getPragmaticMeanings(utterance, ruleSets);
      //get on and off record meanings, send to rr if connected.
      Set<Term> directMeaning = meanings.getLeft();
      log.debug("Direct Meaning: " + directMeaning);
      indirectMeaning = new ArrayList<>(meanings.getRight());
      log.debug("Indirect Meaning: " + indirectMeaning);

      //TODO:brad: this is the only place where the dialogue consultant is used. If this is changed it can be removed
      if (directMeaning.size() > 0) {
        if (!utterance.getBindings().isEmpty()) {
          Map<Variable, Symbol> varbindings = new HashMap<>();
          Map<Symbol, Symbol> refbindings = new HashMap<>();
          for (Variable v : utterance.getBindings().get(0).keySet()) {
            Symbol s = utterance.getBindings().get(0).get(v);
            if (s != null && s.getName().contains("dialog_")) {
              try {
                List<Term> refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("getProperties").argTypes(Symbol.class)).call(List.class, s);
                if (!refs.isEmpty()) {
                  Term re = refs.get(0);
                  if (re.get(0).equals(v)) {
                    //property(VAR0) form
                    varbindings.put(v, Factory.createSymbol(re.getName()));
                    refbindings.put(s, Factory.createSymbol(re.getName()));
                  } else {
                    varbindings.put(v, re);
                    refbindings.put(s, re);
                  }
                }
              } catch (TRADEException e) {
                log.error("Error calling getProperties:", e);
              }
            } else if (s != null) {
              varbindings.put(v, s);
            }
          }

          Term t = new ArrayList<>(directMeaning).get(0).copyWithNewBindings(varbindings);
          for (Symbol ref : refbindings.keySet()) {
            t = Factory.createPredicate(t.toUntypedString().replace(ref.getName(), refbindings.get(ref).toString()));
          }
          utterance.setSemantics(t);
        } else {
          utterance.setSemantics(new ArrayList<>(directMeaning).get(0));
        }
      }
      utterance.setIndirectSemantics(indirectMeaning);
      log.debug("[applyPragmaticMeaning] semantics: " + utterance.getSemantics());
      if (!directMeaning.isEmpty()) {
        semantics = directMeaning.iterator().next();
      } else {
        semantics = utterance.getSemantics();
      }
    }

    List<PragRuleProverSet> newSets = loadPragRules(Resources.createFilepath(resourceConfigPath, "demospiecemeal.prag"));
    //necessary for the belief contexts of the prag rules in the recursive file
    updateRelevantBeliefs();

    Pair<Set<Term>, Set<Term>> secondmeanings = getRecursedMeanings(utterance, newSets);
    Set<Term> newDirectMeaning = secondmeanings.getLeft();

    if (newDirectMeaning.size() > 0) {
      semantics = new ArrayList<>(newDirectMeaning).get(0);
    }
    if (!utterance.getBindings().isEmpty()) {
      for (Variable var : utterance.getBindings().get(0).keySet()) {
        semantics = Factory.createPredicate(semantics.toString().replace(utterance.getBindings().get(0).get(var).toString(), var.toString()));
      }
    }
    utterance.setSemantics(semantics);
    utterance.setIndirectSemantics(indirectMeaning);
  }

  private void applyOutputPragmaticMeaning(Utterance utterance) {
    Pair<Set<Term>, Set<Term>> meanings = getRecursedMeanings(utterance, genRuleSets);
    Set<Term> directMeaning = meanings.getLeft();

    if (directMeaning.size() > 0) {
      utterance.setSemantics(new ArrayList<>(directMeaning).get(0));
    }
  }

  private boolean initialPragmaticsFound(Utterance u) {
    //We only want to check the nonrecursive set of meanings if we know we haven't done so already.
    //This checks to see if we've already done an initial pass at pragmatics.
    //If we have, then skip the nonrecursive rules and go straight to demospiecemeal for recursive pragmatics.

    Symbol semantics = u.getSemantics();

    if (u.getSemantics().isVariable()) {
      semantics = u.getBoundSemantics().get(0);
    }

    if (semantics.getName().equals("want") && u.getType().equals(UtteranceType.INSTRUCT)) {
      return true;
    } else if (semantics.getName().equals("wantBel") && u.getType().equals(UtteranceType.STATEMENT)) {
      return true;
    } else if (semantics.getName().equals("itk") && u.getType().equals(UtteranceType.QUESTION)) {
      return true;
    } else if (semantics.getName().equals("greeting") && u.getType().equals(UtteranceType.GREETING)) {
      return true;
    } else {
      return false;
    }
  }

  protected void updateRelevantBeliefs() {

    Set<Term> supportedRelBels = new HashSet<>();
    log.trace("relevant query terms: " + relevantQueryTerms);
    List<Term> terms = new ArrayList<>();
    List<Term> queries = new ArrayList<>();
    for (Term s : relevantQueryTerms) {
      if (s.isTerm()) {
        Term t = s;
        terms.add(t);
        Term queryTerm = PragUtil.formPrologQueryTerm(t);

        log.trace("queryTerm: " + queryTerm);
        queries.add(queryTerm);
      }
    }

    List<List<Map<Variable, Symbol>>> nBindings = new ArrayList<>();
    try {
      nBindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(List.class)).call(List.class, queries);
    } catch (TRADEException e) {
      log.warn("Could not query beliefs.", e);
    }
    if (!nBindings.isEmpty()) {
      for (int i = 0; i < terms.size(); i++) {
        Term t = terms.get(i);
        List<Map<Variable, Symbol>> bindings = nBindings.get(i);
        for (Map<Variable, Symbol> binding : bindings) {
          Term boundTerm = PragUtil.getBoundTerm(binding, t);
          supportedRelBels.add(boundTerm);
        }
      }
    }
    log.trace("supportedRelBels: " + supportedRelBels);
    relevantBeliefs = supportedRelBels;
  }

  protected Pair<Set<Term>, Set<Term>> getMeaningsInSet(PragRuleProverSet prs, Utterance utt,
                                                        Set<Term> relevantBeliefs, List<Utterance> dialogHistory) {

    List<Map<Variable, Symbol>> dialogApply = new ArrayList<>();
    List<Map<Variable, Symbol>> beliefApply = prs.checkEnvironmentalApplicability(relevantBeliefs);
    if (!beliefApply.isEmpty()) {
      dialogApply = prs.checkDialogueApplicability(dialogHistory);
    }
    if (!dialogApply.isEmpty()) {
      Map<Variable, Symbol> shouldApply = new HashMap<>();
      if (beliefApply.size() == 1) {
        shouldApply.putAll(beliefApply.get(0));
      }
      if (dialogApply.size() == 1) {
        shouldApply.putAll(dialogApply.get(0));
      }
      prover.setTheory("");

      //load rules into prover for each
      for (PragRuleProver pr : prs) {
        if (pr.getLeftType() != utt.getType()) {
          continue;
        }

        log.trace("leftSem = " + pr.leftSem + " | utt = " + utt.getSemantics());
        for (Symbol prLHS : pr.leftSem) {
          if (prLHS.isVariable()) {
            Map<Variable, Symbol> bindingsOption = new HashMap<>();
            Symbol semantics = utt.getSemantics();
            List<Term> boundSemantics = utt.getBoundSemantics();
            log.debug("[getMeaningsInSet] bound semantics: " + boundSemantics);
            if (!boundSemantics.isEmpty()) {
              semantics = boundSemantics.get(0);
            }
            bindingsOption.put((Variable) prLHS, semantics);
            bindingsOption = PragUtil.getBindingsSymHelper(bindingsOption, pr.getAddressee(), utt.getAddressee());
            bindingsOption = PragUtil.getBindingsSymHelper(bindingsOption, pr.getSpeaker(), utt.getSpeaker());
            PragRuleProver boundpr = pr.getBoundVersion(bindingsOption);
            Set<Term> directMeaning = boundpr.getSemantics();
            Set<Term> indirectMeaning = boundpr.getOffRecordSem();

            if (directMeaning != null && indirectMeaning != null) {
              log.debug("match found! " + directMeaning);
              log.debug("bindings: " + bindingsOption);
              return Pair.of(bindMeanings(directMeaning, shouldApply), bindMeanings(indirectMeaning, shouldApply));
            }
          } else {
            if (utt.getSemantics().isTerm()) {
              // get bound semantics to assert to prover
              Map<Variable, Symbol> suppBindings = generateBindingsFromSupplementalSemantics(utt);
              for (Term boundSemantics : utt.getBoundSemantics()) {
                if (!boundSemantics.getVars().isEmpty()) {
                  // if bound semantics has un-bound variables, use temp bindings generated from the supp semantics
                  boundSemantics = PragUtil.getBoundTerm(suppBindings, boundSemantics);
                }
                prover.assertBelief(boundSemantics);
              }
            } else if (utt.getSemantics().equals(prLHS)) {
              Map<Variable, Symbol> bindingsOption = new HashMap<>();
              bindingsOption = PragUtil.getBindingsSymHelper(bindingsOption, pr.getAddressee(), utt.getAddressee());
              if (bindingsOption == null) {
                // does not match the current prag rule
                continue;
              }
              bindingsOption = PragUtil.getBindingsSymHelper(bindingsOption, pr.getSpeaker(), utt.getSpeaker());
              PragRuleProver boundpr = pr.getBoundVersion(bindingsOption);
              Set<Term> directMeaning = boundpr.getSemantics();
              Set<Term> indirectMeaning = boundpr.getOffRecordSem();
              if (directMeaning != null && indirectMeaning != null) {
                //log.debug("match found! "+directMeaning);
                log.debug("bindings: " + bindingsOption);
                return Pair.of(bindMeanings(directMeaning, shouldApply), bindMeanings(indirectMeaning, shouldApply));
              }
            }

            List<Map<Variable, Symbol>> bindings = new ArrayList<>();
            try {
              bindings = prover.queryBelief((Term) prLHS);
            } catch (Exception e) {
              log.error("Error querying prover with: " +  prLHS, e);
            }

            if (!bindings.isEmpty()) {
              //for each context binding, check against each prag rule binding. if there's no overlap in keyset, merge.
              //we're only iterating through beliefApply rather than dialogApply
              //because there shouldn't be a situation in which there's more than one binding returned from a dialog context
              List<Map<Variable, Symbol>> combinedBindings = new ArrayList<>();
              for (Map<Variable, Symbol> binding : bindings) {
                for (Map<Variable, Symbol> beliefBinding : beliefApply) {
                  Map<Variable, Symbol> newbinding = new HashMap<>();
                  boolean conflict = false;
                  for (Variable v : beliefBinding.keySet()) {
                    //check if there's any conflict in the bindings-- if there is, we don't want to consider that combo
                    if (binding.containsKey(v) && !binding.get(v).equals(beliefBinding.get(v))) {
                      conflict = true;
                    }
                  }
                  for (Variable v : dialogApply.get(0).keySet()) {
                    if (binding.containsKey(v) && !binding.get(v).equals(dialogApply.get(0).get(v))) {
                      conflict = true;
                    }
                  }
                  if (!conflict) {
                    newbinding.putAll(beliefBinding);
                    newbinding.putAll(binding);
                    combinedBindings.add(newbinding);
                  }
                }
              }

              //get bindings for the property context
              bindings = prs.checkPropertyApplicability(combinedBindings, utt, prs.getPropertyContext());

              log.trace("query: " + prLHS + " | bindings: " + bindings);
              if (!(bindings.isEmpty())) {
                //TODO: fix bindings issue such that we get accurate bindings regardless of prag rule ordering
                Map<Variable, Symbol> bindingsOption = bindings.get(0);
                bindingsOption = PragUtil.getBindingsSymHelper(bindingsOption, pr.getAddressee(), utt.getAddressee());
                if (bindingsOption == null) {
                  // does not match the current prag rule
                  continue;
                }
                bindingsOption = PragUtil.getBindingsSymHelper(bindingsOption, pr.getSpeaker(), utt.getSpeaker());
                for (Variable key : bindingsOption.keySet()) {
                  //replace 'safe' variable name with the correct original variable name
                  for (Term supp : utt.getSupplementalSemantics()) {
                    if (!supp.getArgs().isEmpty()) {
                      String phrase = bindingsOption.get(key).toString();
                      phrase = phrase.replace(supp.getName(), supp.get(0).toString());
                      if (bindingsOption.get(key).isPredicate()) {
                        bindingsOption.replace(key, Factory.createPredicate(phrase));
                      } else {
                        bindingsOption.replace(key, Factory.createSymbol(phrase));
                      }
                    }
                  }
                }

                PragRuleProver boundpr = pr.getBoundVersion(bindingsOption);
                Set<Term> directMeaning = boundpr.getSemantics();
                Set<Term> indirectMeaning = boundpr.getOffRecordSem();
                if (directMeaning != null && indirectMeaning != null) {
                  log.debug("bindings: " + bindingsOption);
                  log.debug("match found! " + directMeaning);
                  return Pair.of(bindMeanings(directMeaning, shouldApply), bindMeanings(indirectMeaning, shouldApply));
                }
              }
            }
          }
        }
      }
    }
    return Pair.of(new HashSet<>(), new HashSet<>());
  }

  /**
   * Create bindings using only supplemental semantics. This is useful if ref resolution hasn't been done
   * yet, or if reference resolution couldn't get references for all free-variables.
   * We want to make sure prolog doesn't mess up the variable name, in other words replace 'VAR0' with 'cup'
   * or whatever is stored in the supplemental semantics
   *
   * @param utterance
   * @return
   */
  private Map<Variable, Symbol> generateBindingsFromSupplementalSemantics(Utterance utterance) {
    Map<Variable, Symbol> suppBindings = new HashMap<>();
    for (Term supp : utterance.getSupplementalSemantics()) {
      if (supp.hasArgs()) {
        //if we haven't done ref resolution yet, we want to make sure prolog doesn't mess up the variable name
        //in other words replace 'VAR0' with 'cup' or whatever is stored in the supplemental semantics
        suppBindings.put((Variable) supp.get(0), Factory.createSymbol(supp.getName(), supp.get(0).getType()));
      } else {
        log.debug("supplemental semantics without args: " + supp);
      }
    }
    return suppBindings;
  }

  protected void updateRelevantBeliefsAndDialogueHistory() {
    // update relevant beliefs
    updateRelevantBeliefs();

    // update dialogue history
    try {
      dialogHistory =TRADE.getAvailableService(new TRADEServiceConstraints().name("getDialogueHistory")).call(List.class);
    } catch (TRADEException e) {
      log.warn("Could not call getDialogueHistory.", e);
    }
  }

  protected Set<Term> bindMeanings(Set<Term> terms, Map<Variable, Symbol> bindings) {
    Set<Term> boundTerms = new HashSet<>();
    Iterator value = terms.iterator();
    while (value.hasNext()) {
      boundTerms.add(((Term) value.next()).copyWithNewBindings(bindings));
    }
    return boundTerms;
  }

  protected Pair<Set<Term>, Set<Term>> getPragmaticMeanings(Utterance u) {
    //assume normal ruleset if not specified
    return getPragmaticMeanings(u, ruleSets);
  }

  protected Pair<Set<Term>, Set<Term>> getPragmaticMeanings(Utterance u, List<PragRuleProverSet> sets) {
    // iterate through possible rule sets
    for (PragRuleProverSet prs : sets) {
      Pair<Set<Term>, Set<Term>> meaning = getMeaningsInSet(prs, u, relevantBeliefs, dialogHistory);
      if (!meaning.getLeft().isEmpty()) {
        return meaning;
      }
    }
    return Pair.of(new HashSet<>(), new HashSet<>());
  }

  protected Pair<Set<Term>, Set<Term>> getRecursedMeanings(Utterance u, List<PragRuleProverSet> sets) {
    //recurse through arguments of term
    Utterance utt;
    Term recursedTerm = null;
    if (u.getSemantics().isTerm()) {
      List<Symbol> recursedArgs = new ArrayList<>();
      List<Symbol> args = ((Term) u.getSemantics()).getArgs();
      for (Symbol s : args) {
        Symbol sym = s;
        if (s.isTerm()) {

          Utterance uArg = new Utterance.Builder()
                  .setSpeaker(u.getSpeaker())
                  .setAddressee(u.getAddressee())
                  .addListeners(u.getListeners())
                  .setSemantics(s)
                  .setUtteranceType(u.getType())
                  .setIsInputUtterance(u.isInputUtterance()).build();

          //add them to the set
          if (!u.equals(uArg)) {
            Pair<Set<Term>, Set<Term>> argSems = getRecursedMeanings(uArg, sets);
            if (argSems != null && !argSems.getLeft().isEmpty()) {
              sym = new ArrayList<>(argSems.getLeft()).get(0);
            }
          }
        }
        recursedArgs.add(sym);
      }
      recursedTerm = Factory.createPredicate(u.getSemantics().getName(), recursedArgs);

      utt = new Utterance.Builder()
              .setSpeaker(u.getSpeaker())
              .setAddressee(u.getAddressee())
              .addListeners(u.getListeners())
              .setSemantics(recursedTerm)
              .setBindings(u.getBindings())
              .setUtteranceType(u.getType())
              .setIsInputUtterance(u.isInputUtterance()).build();
    } else {
      utt = u;
    }

    //need to get relevant sets via belief and dialogue history
    Pair<Set<Term>, Set<Term>> meaning = getPragmaticMeanings(utt, sets);
    if (!meaning.getLeft().isEmpty()) {
      return meaning;
    }

    if (recursedTerm != null) {
      Set<Term> set = new HashSet<>();
      set.add(recursedTerm);
      return Pair.of(set, new HashSet<>());
    }
    return Pair.of(new HashSet<>(), new HashSet<>());
  }

}
