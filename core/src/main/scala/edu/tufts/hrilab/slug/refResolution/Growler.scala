/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution

import edu.tufts.hrilab.fol.{Factory, Symbol, Term, Variable}
import ReferenceResolutionComponent.GHSelfModel
import org.slf4j.{Logger, LoggerFactory}

import scala.collection.JavaConverters._
import scala.collection.mutable.ListBuffer
import scala.collection.{Seq, mutable}
import scala.language.implicitConversions
import scala.util.Try

/** Pseudocode
  * Algorithm 9 growler (Λ, Ξ)
  * 1: Λ: set of formulae, Ξ: set of status cue mappings
  * 2: Θ = create_plan_maps(Ξ)
  * 3: C = create_candidate_maps(Λ V )
  * 4: for all v ∈ ΛV do
  * 5:  while (~E c ∈ C_v | R(c) >= ~R) ∧ (Θ_v \ LTM = ∅) do
  * 6:    grow(Λ_v, C_v, Θ_v)
  * 7:  end while
  * 8: end for
  * 9: Γ = BuildAndAssessTable(C)
  * 10: R = [v ∈ ΛV | CanAndShouldExpand(v, Γ)]
  * 11: while R = ∅ do
  * 12:   for all v ∈ R do
  * 13:     C' = (C \ C_v ) ∪ grow(Λ_v , C_v , Θ_v )
  * 14:     Γ = Γ ∪ BuildAndAssessTable(C')
  * 15:     C_v = C_v ∪ C'_v
  * 16:   end for
  * 17:   R = [v ∈ ΛV |CanAndShouldExpand(v, Γ)]
  * 18: end while
  * 19: Γ* = ASSESS-LTM(Λ, Q)
  * 20: if | Γ* | =/= 1 then
  * 21:   return relevantPrefix(Γ*)
  * 22: else
  * 23:   return assert(bind(Λ, Γ*[0]))
  * 24: end if
  *
  * Algorithm 10 grow(Λ,C,Θ)
  * 1: for all e ∈ domain(head(Θ)) do
  * 2:  P(e) = ASSESS(e, Λ)
  * 3:  if P (e) > ~P then
  * 4:    C = C ∪ <e, P (e), R(e)>
  * 5:  end if
  * 6: end for
  * 7: pop(Θ)
  * 8: return C
  */


/**
  * Growler: Givenness and Relevance Theoretic Open-World Entity Resolution
  *
  * @param gh        -- Giveness Hierarchyy buffer data structure
  * @param relevanceMap -- A data structure for storing the relevance of various entities,
  *                     based on linguistic or visual data.
  */
class Growler(resolver: Resolver, gh: GHSelfModel, relevanceMap: mutable.Map[Symbol, EntityScore]){

  import Growler._ //Companion object helper methods

  final val log:Logger = LoggerFactory.getLogger(getClass)

  def getMnemonicActions(ghTier: Symbol): ListBuffer[String]={
    ghTier.getName match {
      case "INFOCUS" => ListBuffer("INFOCUS")
      case "ACTIVATED" => ListBuffer("ACTIVATED","INFOCUS")
      case "REFACTIVATED" => ListBuffer("ACTIVATED","INFOCUS","POSIT")
      case "FAMILIAR" => ListBuffer("ACTIVATED", "INFOCUS","FAMILIAR","LTM")
      case "UNFAMILIAR" => ListBuffer("UNFAMILIAR")
      case "DEFINITE" => ListBuffer("ACTIVATED","INFOCUS","FAMILIAR","LTMP")
      case "INDEFINITE" => ListBuffer("POSIT")
      case _ =>
        log.error("invalid GhTier when getting Mnemonic Actions "+ ghTier)
        ListBuffer()
    }
  }

  /**
    * The GROWLER algorithm.
    *
    * @param semantics The set of semantic constraints to use during resolution
    * @param statuses  The set of status cue mappings for each variable used in those constraints.
    * @return
    */
  def resolve_clause(semantics: java.util.List[Term], statuses: java.util.Map[Variable, Symbol]): java.util.List[Hypothesis] = {
    log.debug("[resolve_clause] IN RESOLVE CLAUSE")
    var allVars = Set[Variable]()
    var allPlans = Map[Variable, ListBuffer[String]]()
    var allTerms = List[Term]()

    //Create a copy so we can modify the list of semantics
    allTerms = semantics.asScala.toList.map {
      _.clone
    }
    log.debug("[resolve_clause] ALL TERMS: {}",allTerms)

    // The First thing we must do is determine the set of variables we need to find bindings for.
    // PSEUDOCODE SIDEBAR: This creates Lambda^V
    allVars = statuses.keySet.asScala.toSet /* only include variable with status cues */
    log.debug("[resolve_clause] ALL VARS: {}",allVars)

    // Next, we need to create a list of actions to perform in search of each variable.
    // PSEUDOCODE SIDEBAR: This creates Theta_V, line 2.
    //    allPlans = allVars.map { s => s -> ListBuffer(statuses(s).actions: _*) }.toMap
    allPlans = statuses.asScala.toMap.map {
      case (key, value) => key -> getMnemonicActions(value)
    }
    log.debug("[resolve_clause] ALL PLANS: {}",allPlans)

    // Next, we create an initially empty set of candidates associated with each variable.
    // PSEUDOCODE SIDEBAR: This creates C_v, line 3.
    val singleVarCandSets = allPlans.map {
      case (v: Variable, tiers: ListBuffer[String]) =>
        SingleVarCandidateList(v, tiers, ListBuffer[RelevanceTheoreticCandidateWithProabability]())
    }

    //adding unfamiliar property to the variable
    allPlans.foreach { v =>
      if (v._2.contains("UNFAMILIAR")) {
        var t = new Term("unfamiliar", v._1)
        allTerms = t :: allTerms
      }
    }

    // Next, we need to populate each of these lists with an initial set of candidates which satisfy the unary predicates found in @param semantics.
    // PSEUDOCODE Sidebar: This is lines 4-8
    singleVarCandSets.foreach(svcs => generateInitialValidCandidateList(svcs, allTerms))
    log.debug("[resolve_clause] SVCS: {}",singleVarCandSets)

    //Now, GROWLER must consider polyadic predicates. To do so, it begins by building a table of
    // hypotheses Γ (fulltable) containing combinations of variable assignments
    // PSEUDOCODE Sidebar: Line 9
    var fulltable = generateFullTable(singleVarCandSets.toSeq, allTerms)
    log.debug("[resolve_clause] FULL TABLE: {}",fulltable)

    //Because this process may result in all sufficiently relevant referential candidates being eliminated,
    //GROWLER now goes through a second cycle of potential expansion-by-mnemonic-action.
    // GROWLER goes through this cycle so long as it is determined that it can and should perform such expansion
    //for some set of variables V.
    //PSEUDOCODE Sidebar: This is line 10.

    //A variable v can be expanded if:
    // 1) there does not already exist a hypothesis in Γ that binds v to a sufficiently relevant candidate referent
    //    (assessed immediately below, and the first condition of the subsequent while loop)
    // 2) Θ_v is nonempty (excepting LTM queries) (the second condition of the subsequent while loop)
    var vars_with_help_needed = allVars.filterNot(v =>
      fulltable.exists(_.bindings.exists(m =>
        m.variable.equals(v) && m.candidate.relevance >= RELEVANCE_THRESHOLD)
      ))
    log.debug("[resolve_clause] VWHN: {}",vars_with_help_needed)

    singleVarCandSets.foreach(s => {
      log.debug(
        "s: {} vwhn: {} vwhn contains?: {} tiers: {} tiers exists: {}"
        , Array(s, vars_with_help_needed,vars_with_help_needed.contains(s.variable),s.tiers,s.tiers.exists(!_.contains("LTM"))):Any
      )
    }
    )

    //While there are variables without sufficiently relevant candidates, that have room for expansion...
    // PSEUDOCODE SIDEBAR: This is line 11.
    while (singleVarCandSets.exists(s => vars_with_help_needed.contains(s.variable) && s.tiers.exists(!_.contains("LTM")))) {
      //PSUDOECODE SIDEBAR: This is line 12.
      singleVarCandSets.foreach {
        //Is this the variable that can be expanded?
        case svcs if vars_with_help_needed.contains(svcs.variable) && svcs.tiers.exists(!_.contains("LTM")) =>
          log.debug("[resolve_clause] CONSIDERING: {}",svcs)
          // First, GROWLER creates a copy of C (i.e., C' ) that
          //differs from C in that the candidate referents associated with v are replaced
          //by the new candidate referents discovered through a round of expansion effected by the grow subroutine (i.e., C'_v )
          //PSEUDOCODE SIDEBAR: These are summarized in line 13.
          // -- Step 1: Create an empty candidate list for this variable.
          val updateSet = SingleVarCandidateList(svcs.variable, svcs.tiers, ListBuffer[RelevanceTheoreticCandidateWithProabability]())
          // -- Step 2: Find the new set of candidates that would be identified if another tier were considered for that variable
          generateInitialValidCandidateList(updateSet, allTerms)
          // -- Step 3: Create a copy of C that replaces the candidates identified for the variable in question with those that are newly identified
          val replacementList = singleVarCandSets.map { case x if x.variable.getName.equals(svcs.variable.getName) => updateSet; case x => x }

          //GROWLER then creates a new hypothesis table using C , and adds all new sufficiently probable hypotheses found in this table to Γ.
          //PSEUDOCODE SIDEBAR: This is line 14.
          val updateTable = generateFullTable(replacementList.toSeq, allTerms)
          fulltable ++= updateTable
          //Finally, GROWLER adds all new sufficiently probable bindings for v (i.e., C'_v ) to C-v
          //PSEUDOCODE SIDEBAR: This is line 15.
          singleVarCandSets.find(_.variable.equals(svcs.variable)).map(_.candidates ++= updateSet.candidates)
          //PSEUDOCODE SIDEBAR: This is line 17
          vars_with_help_needed = allVars.filterNot(v =>
            fulltable.exists(_.bindings.exists(m =>
              m.variable.equals(v) && m.candidate.relevance >= RELEVANCE_THRESHOLD)
            )
          )
      }
    }

    // Now, GROWLER must deal with LTM queries that were previously set
    // aside, using the ASSESS-LTM subroutine. For any variables that still should
    // be expanded, and that can be expanded if LTM-querying is viewed as an
    // acceptable mnemonic action, ASSESS-LTM uses DIST-POWER to effect
    // such LTM queries and update the results stored in Γ*
    //PSEUDOCODE SIDEBAR: The next set of lines are line 19.

    // -- Step 1: For each  multivariable binding candidate in the fulltable, create a new table where each binding
    // hypothesis is attached to the total relevance of all candidates involved in that hypothesis.
    val fulltableWithRelevance = fulltable.map(x => (x, x.bindings.map(_.candidate.relevance).sum))
      // Then, sort this table in descending order of relevance.
      .sortBy(x => -x._2)

    log.debug("full table with relevance: {}", fulltableWithRelevance)

    // Determine the subset of hypotheses whose relevance falls above some threshold. Here, we say that we'll return hypotheses
    // that are at least half as relevant as the most relevant option.
    val RELEVANCE_PREFIX_THRESHOLD: Option[Double] = fulltableWithRelevance.headOption.map(_._2 / 2.0)
    val toret: Seq[CrossMappingCandidateList] = fulltableWithRelevance.takeWhile(_._2 >= RELEVANCE_PREFIX_THRESHOLD.getOrElse(0.0))
      //And then we can strip off the relevance scores, as we don't need them anymore.
      .map(_._1)
    //Now we need to turn these into hypotheses so we can send them to POWER.
    var hyps = toret.map(cmclToHypothesis)
    //If for some reason this is empty, reset hyps to an empty hypothesis.
    if (hyps.isEmpty) hyps = Seq(Hypothesis(Map(), 1.0))
    //Find variables that we should send to POWER without positing new entities
    val ltmNoPositSVCSs = singleVarCandSets.filter(s => vars_with_help_needed.contains(s.variable) && s.tiers.headOption.exists(_.equals("LTM")))
    //Find variables that we should send to POWER, positing new entities if necessary
    //    val ltmPositSVCSs = singleVarCandSets.filter(s => vars_with_help_needed.contains(s.variableName) && s.tiers.headOption.exists(_.equals(LTMP)))
    val ltmPositSVCSs = singleVarCandSets.filter(s => vars_with_help_needed.contains(s.variable) && (s.tiers.headOption.exists(_.equals("LTMP")) || s.tiers.isEmpty))
    //Find the variable names for both cases
    val ltmNoPositVars = ltmNoPositSVCSs.map(_.variable).toList
    val ltmPositVars = ltmPositSVCSs.map(_.variable).toList
    //Find all predicates involving those variables, in both cases
    val ltmNoPositTerms = allTerms.filter(_.getArgs.asScala.exists {
      case v: Variable => ltmNoPositVars.exists(x => x.getName.equals(v.getName))
      case _ => false
    })
    val ltmPositTerms = allTerms.filter(_.getArgs.asScala.exists {
      //case v: Variable => ltmPositVars.contains(v)
      case v: Variable => ltmPositVars.exists(x => x.getName.equals(v.getName))
      case _ => false
    })

    log.debug("[resolve_clause] LTM VARS: " + ltmNoPositVars + " / " + ltmPositVars + ", LTM TERMS: " + ltmNoPositTerms + " / " + ltmPositTerms)

    //For each hypothesis, use POWER to additionally consider entities across LTM, with that hypothesis as the starting point, in both cases.

    var results = hyps.flatMap { hyp =>
      resolve_from_LTM(ltmNoPositTerms, ltmNoPositVars, hyp, posit = false)
    }
    log.debug("initial results : {}",results)
    results.filter(_.likelihood >= PROBABILITY_THRESHOLD)
    if (results.isEmpty) results = Seq(Hypothesis(Map(), 1.0))
    results = results.flatMap{ hyp => resolve_from_LTM(ltmPositTerms, ltmPositVars, hyp, posit = true) }.filter(_.likelihood >= PROBABILITY_THRESHOLD)

    if (results.isEmpty) results = Seq(Hypothesis(Map(), 1.0))

    // Finally, we might need to hypothesize new entities. Unfortunately, since we're not actually doing
    // multiple hypothesis tracking of the world, we can only confidently posit new entities if we end up with a single
    // valid hypothesis.
    // PSEUDOCODE SIDEBAR: This is 20-23. Technically that code only computes the relevant prefix afterward if there's more
    // than one valid hypothesis, but I think this is more efficient. Also, that does positing on the most relevant hypothesis,
    // Whereas this only does positing if there's a single hypothesis.
    // Both may be valid.
    log.debug("[resolve_clause] intermediary results: {}",results)
    if (results.size == 1) { // TODO: do we really only posit if there's only 1 hyp? see above
      log.debug("[resolve_clause] >>> positing results")
      val resHyp = results.head
      log.debug("resHyp: {}", resHyp)
      //These are the vars that need positing!
      val indefVars = resHyp.assignments.filter(_._2.getName.contains("?")).keys.toSeq
      log.debug("[resolve_clause] >>> >>> indefVars: {}",indefVars)
      if (indefVars.nonEmpty) {
        //These are the terms involving those vars!
        val indefTerms = allTerms.filter(_.getArgs.asScala.exists {
          //case v: Variable => indefVars.contains(v)
          case v: Variable => indefVars.exists(x => x.getName.equals(v.getName))
          case _ => false
        })
        log.debug("indef terms: {}",indefTerms)
        //This is the most relevant (only remaining) hypothesis, filtered to only include the known bindings.
        val knownHyp = Hypothesis(resHyp.assignments.filter(!_._2.getName.contains("?")), resHyp.likelihood)
        log.debug("known Hyps: {}", knownHyp)
        results = posit_to_LTM(indefTerms, indefVars, knownHyp)
      }
    }

    log.debug("[resolve_clause] final results: " + results)
    results.asJava
  }

  /**
    * creates new representations in long term memory
    *
    * @param terms      -- the predicates involved with those new representations
    * @param varNames   -- one variable for each entity to be posited
    * @param initialHyp -- known bindings for the other entities
    * @return -- a new set of bindings that combines the initial hyp with new bindings for new entities
    */
  def posit_to_LTM(terms: List[Term], varNames: Seq[Variable], initialHyp: Hypothesis): Seq[Hypothesis] = {
    log.debug("        @posit_to_LTM: " + terms + " " + varNames + " " + initialHyp)

    //TODO:brad: update Predicate Property to extend term
    val termsPP = terms.map(new Property(_))
    val T = resolver.getVariableMapping(termsPP)
    val bTermsPP = termsPP.map(_.boundForm(T))

    if (termsPP.isEmpty) {
      Nil
    } else {
      resolver.completeSolutions(Seq(initialHyp), varNames, bTermsPP)
    }
  }

  /**
    * Uses POWER to resolve references to entities referenced in a set of predicates, given an initial partial binding.
    *
    * @param terms      -- the predicate constraints
    * @param vars   -- the variables that need resolving
    * @param initialHyp -- the initial hypothesis
    * @param posit      -- whether it's okay to create new representations for potentially previously unknown entities
    * @return -- a set of possible sufficiently probably binding hypotheses
    */
  def resolve_from_LTM(terms: List[Term], vars: Seq[Variable], initialHyp: Hypothesis, posit: Boolean): Seq[Hypothesis] = {
    log.debug("[resolve_from_LTM]: {} {} {} {}", Array(terms, vars, initialHyp, posit):Any)
    //val (indefBinds, knownBinds) = initialHyp.assignments.partition(_._2 == "?")
    val (indefBinds, knownBinds) = initialHyp.assignments.partition(_._2.getName.contains("?"))
    log.debug("[resolve_from_LTM] indefBinds: {} knownBinds: {}",indefBinds,knownBinds,"")
    val knownHyp = Hypothesis(knownBinds, initialHyp.likelihood)

    //TODO:brad: update Predicate Property to extend term
    val termsPP = terms.map(new Property(_))

    if (termsPP.isEmpty) {
      Seq(knownHyp)
    } else {
      val resHyps = {
        List(resolver.resolve(termsPP, vars, Seq(knownHyp), posit): _*)
      }
      log.debug("[resolve_from_LTM] before hypothesis construction: {}",resHyps)
      resHyps.map(h => Hypothesis(h.assignments ++ indefBinds, h.likelihood))
      log.debug("[resolve_from_LTM] after hypothesis construction: {}",resHyps)
      resHyps
    }
  }

  /**
    * Creates an initial list of candidates for a variable.
    *
    * @param svcl a data structure containing the variable/list of mnemonic actions, /empty-list-of-candidates
    */
  //TODO: SANITY CHECK THAT THIS ACTUALLY MODIFIES THE DATA STRUCTURES
  def generateInitialValidCandidateList(svcl: SingleVarCandidateList, allTerms: Seq[Term]): Unit = {
    svcl match {
      case SingleVarCandidateList(variable, mnemonicActions, cands) => //Deconstruct s into the variable, list of mnemonic actions left to take
        log.debug("svcl: " + svcl)
        log.debug("generateInitialValidCandidateList " + variable + " " + mnemonicActions + " " + cands)


        //Expand the set of initial candidates until we have at least one which is sufficiently relevant...
        // with the caveat that we can't keep expanding if there are no actions left to consider, and that we shouldn't
        // be doing any full LTM queries at this stage!
        //PSEUDOCODE SIDEBAR: This is line 5.
        while (!cands.exists(c => c.candidate.relevance >= RELEVANCE_THRESHOLD)
          && mnemonicActions.nonEmpty
          && !mnemonicActions.head.contains("LTM")) {

          //PSEUDOCODE SIDEBAR: This is the GROW function.
          //We're gonna add stuff to the set of candidates!
          //cands ++= //(Line 4)
          //Okay. The set of candidates we can consider are those in the data structure associated with the next mnemonic action to consider. (Line 1)
          val cand = domain(mnemonicActions.head).map { ref: Symbol =>
            //For each of those candidates, calculate its relevance, and the probability that it satisfies the unary predicates (Line 2)
            val rtc = RelevanceTheoreticCandidate(ref, getRelevance(ref))

            val rtcwp = RelevanceTheoreticCandidateWithProabability(rtc, getProbabilityofSatisfyingUnaryPredicates(RelevanceTheoreticBinding(variable, rtc), allTerms))
            log.trace("rtcwp: " + rtcwp)
            rtcwp
            //But only keep those that are sufficiently probable (Line 3)
          }.filter {
            _.probability >= PROBABILITY_THRESHOLD
          }
          log.debug("mnemonic action: {}", mnemonicActions.head)
          log.debug("cand: {}", cand)
          cands ++= cand
          //Sort the candidates in decreasing order of relevance
          cands.sortBy(-_.candidate.relevance)
          //And pop the mnemonic action we performed off its list. (Line 7)
          mnemonicActions.trimStart(1)
        }
    }
  }

  /**
    * What's the domain associated with mnemonic action m?
    *
    * @param m The mnemonic action
    * @return A reference to the appropriate data structure
    */
  def domain( mnemonicAction: String): Seq[Symbol] = mnemonicAction match {
    case "INFOCUS" => gh.focBuffer.asScala.toSeq
    case "ACTIVATED" => gh.actBuffer.asScala.toSeq
    case "FAMILIAR" => gh.famBuffer.asScala.toSeq
    case _ => Seq(Factory.createSymbol("?_0")) //TODO:brad: is this really what we want?
  }


  /**
    * Calculates the relevance of a reference
    *
    * @param ref the reference in question
    * @return Gets the entity score from the relevanceMap and returns the weighted relevance score of that entity,
    *         or the default Relevance if no entry was found.
    */
  def getRelevance(ref: Symbol): Double = relevanceMap.get(ref) match {
    case Some(result) => log.trace("weight relevance: " + ref + " " + relevanceMap.get(ref) + " " + weightRelevance(result)); weightRelevance(result)
    case None => log.trace("default relevance: " + ref + " " + relevanceMap); DEFAULT_RELEVANCE
  }


  /**
    * Gets the probability of reference Ref satisfying all of the predicates in the current query
    *
    * @param relevantBindings -- the variables of interest and the refs they bound to
    * @param props            -- the terms that must be satisfied
    * @return
    */
  def getProbabilityOfSatisfyingProperties(relevantBindings: Seq[RelevanceTheoreticBinding], props: Seq[Property]): Double = {
    //Create a new Hypothesis for these variable-reference bindings.
    //TODO: Ideally r should contain type info, but we're just injecting it here for now
    val hyp = Hypothesis(relevantBindings.foldRight(Map[Variable, Symbol]())((r, m) => m + (r.variable -> r.candidate.ref)), 1.0)
    //Given the properties, determine which consultants to associate with each variable.
    val T = Try {
      resolver.getVariableMapping(props)
    }.getOrElse {
      log.warn("Failed to get variable mapping. Assuming zero probability. ")
      return 0.0
    }
    //Then, bind the properties according to these consultant associations.
    val boundProps: Seq[Property] = props.map(_.boundForm(T))
    //Finally, for each of these unary predicate properties, use POWER's assess method to determine how probable
    //It is that that property holds under variable binding hypothesis hyp, and multiply the resulting probabilities
    // together to calculate the joint probability under a Naive Bayesian assumption.
    boundProps.map { boundProp: Property =>
      Try {
        resolver.assess(hyp, boundProp)
      }.getOrElse {
        log.debug("Failed to get probability from assess. Assuming zero probability. ")
        return 0.0
      }
    }.reduceLeft(_ * _)
  }

  /**
    * Gets the probability of reference Ref satisfying all of the polyadic predicates in the current query when bound to @param variable.
    *
    * @param bindings -- the variables of interest and the refs they are bound to
    * @param allTerms -- the terms that must be satisfied
    * @return
    */
  def getProbabilityOfSatisfyingPolyadicProperties(bindings: Seq[RelevanceTheoreticBinding], allTerms: Seq[Term]): Double = {
    //We don't want to deal with purely hypothetical, unposited references
    val relevantBindings = bindings.filter(_.candidate.ref != Factory.createSymbol("?_0"))
    //If we're only dealing with a purely hypothetical, unposited references, assume they satisfies everything.
    if (relevantBindings.isEmpty) return 1.0
    //Find all the predicates that wouldn't have been examined before.
    val relevantPredicates = allTerms.filterNot { term: Term =>
      bindings.exists(binding => term.getOrderedVars.contains(binding.variable) && term.getOrderedVars.size()==1)
    }
    //If there aren't any unary predicates involving this variable, return 1.0. (No possibility of evidence against it!)
    if (relevantPredicates.isEmpty) return 1.0
    //Convert the predicates to properties
    val props: Seq[Property] = relevantPredicates.map {
      new Property(_)
    }
    getProbabilityOfSatisfyingProperties(relevantBindings, props)
  }

  /**
    * Gets the probability of reference Ref satisfying all of the unary predicates in the current query when bound to @param variable.
    *
    * @param binding  -- the variable of interest and the ref it's bound to
    * @param allTerms -- the terms that must be satisfied
    * @return
    */
  def getProbabilityofSatisfyingUnaryPredicates(binding: RelevanceTheoreticBinding, allTerms: Seq[Term]): Double = {
    //If we're dealing with a purely hypothetical, unposited reference, assume it satisfies everything.
    if (binding.candidate.ref == Factory.createSymbol("?_0")) return 1.0
    //Find all the predicates that use only this variable, and filter out irrelevant Predicates
    val relevantPredicates = allTerms.filter { predicate: Term =>
      predicate.getOrderedVars.asScala.exists(v => v.getName.equals(binding.variable.getName)) && predicate.getOrderedVars.size()==1
    }
    //If there aren't any unary predicates involving this variable, return 1.0. (No possibility of evidence against it!)
    if (relevantPredicates.isEmpty) return 1.0
    //Convert the predicates to properties
    val props: Seq[Property] = relevantPredicates.map {
      //TODO:brad: update Predicate Property to extend term
      new Property(_)
    }
    getProbabilityOfSatisfyingProperties(Seq(binding), props)
  }

  /**
    * Generates the full table of hypotheses in order to consider polyadic properties
    *
    * @param svcls -- a sequence of lists of candidates associated with different variables
    * @return a cross-mapping candidate list that combines these into different multivariable binding hypotheses.
    */
  def generateFullTable(svcls: Seq[SingleVarCandidateList], allTerms: Seq[Term]): Seq[CrossMappingCandidateList] = {
    log.debug("generating full table. svcls:" + svcls + " allTerms: " + allTerms)
    svcls
      // First, we transform each single-variable candidate list into a cross-mapping candidate list that only contains information about that single variable.
      .map(generateCrossMappingCandidateList)
      //Then, merge these together into one giant table of multivariable binding hypothese,
      .reduceLeft[Seq[CrossMappingCandidateList]] {
      (cmclAccumulator, cmclNext) =>
        mergeCrossMappingCandidateLists(cmclAccumulator, cmclNext)
          // and ditch any hypotheses that we already know are insufficiently probable based on information from unary predicates alone.
          .filter(_.probability >= PROBABILITY_THRESHOLD)
    }
      //Finally, use POWER's asses-all method to get the full probability, newly considering polyadic properties...
      .map { m: CrossMappingCandidateList => m.copy(probability = m.probability * getProbabilityOfSatisfyingPolyadicProperties(m.bindings, allTerms)) }
      //And filter out insufficiently relevant  hypotheses.
      .filter(_.probability >= PROBABILITY_THRESHOLD)
  }

}

object Growler {
  val RELEVANCE_THRESHOLD = 0.2 //How relevant does a candidate need to be to accept without moving on?
  val PROBABILITY_THRESHOLD = 0.2 //How probable does a candidate need to be to accept at all?
  val DEFAULT_RELEVANCE = 0.0 // If we don't know otherwise, the relevance of a given reference is zero. Change this to 1 to assume everything is relevant.

  implicit def bool2int(b: Boolean): Int = if (b) 1 else 0

  /**
    * Converts a set of bindings with associated probabilities into a Hypothesis, the data structure used by POWER.
    * This conversion is necessary because GROWLER needs to worry about relevance, while POWER does not.
    *
    * @param c The CMCL to convert to a hypothesis
    * @return a Hypothesis version of that CMCL.
    */
  def cmclToHypothesis(c: CrossMappingCandidateList) =
//TODO: Ideally r should contain type info, but we're just injecting it here for now
  Hypothesis(c.bindings.foldRight(Map[Variable, Symbol]())((r, m) => m + (r.variable -> r.candidate.ref)), c.probability)

  /** The weightings to use when calculating relevance */
  val relevanceWeightings = Seq(10.0, 5.0, 2.0, 1.0)

  /** Normalize relevance values to between zero and 1/|relevance weightings|.
    * 0 --> 0
    * 10 --> ~1
    * infinity --> 1
    */
  def scaleRelevance(x: Double): Double = (1.0 / relevanceWeightings.length) / (1.0 + scala.math.exp(-0.5 * x)) * 2.0 - (1.0 / relevanceWeightings.length)

  /**
    * Create a single relevance value by taking each relevance factor, multiplying it
    * by its scaling factor defined in relevanceWeightings, normalizing each to between 0 and 0.25, and summing them up.
    */
  def weightRelevance(score: EntityScore): Double = {
    val relevanceVector = Seq[Double](score.inMainClause.toInt.toDouble, score.synProm, score.recency, score.bonus)
    val x = relevanceVector.zip(relevanceWeightings).map { case (x, y) => x * y }
    val y = x.map(scaleRelevance)
    val z = y.sum
    relevanceVector.zip(relevanceWeightings).map { case (x, y) => x * y }.map(scaleRelevance).sum
  }

//  /**
//    * Checks if a term contains a particular variable
//    *
//    * @param term         -- the term
//    * @param variableName -- the variable's name
//    * @return
//    */
//  def containsVar(term: Term, variableName: String): Boolean = term.getArgs.asScala.exists {
//    case vv: Variable => vv.getName.equals(variableName)
//    case tt: Term => containsVar(tt, variableName)
//    case _ => false
//  }
//
//  /**
//    * Checks if a term contains no variables other than a particular variable
//    *
//    * @param term         -- the term
//    * @param variableName -- the variable's name
//    * @return
//    */
//  def notContainsOtherVars(term: Term, variableName: String): Boolean = term.getArgs.asScala.forall {
//    case vv: Variable => vv.getName.equals(variableName)
//    case tt: Term => notContainsOtherVars(tt, variableName)
//    case _ => true
//  }

  /**
    * Turns a list of binding hypotheses for a single variable into a list of alternative (single-variable) multivariable binding hypotheses
    *
    * @param svc -- the list for the single variable
    * @return the list of multivariable binding hypotheses
    */
  def generateCrossMappingCandidateList(svc: SingleVarCandidateList): Seq[CrossMappingCandidateList] =
    svc.candidates.map { c =>
      CrossMappingCandidateList(List(RelevanceTheoreticBinding(svc.variable, c.candidate)), c.probability)
    }

  /**
    * Merges together two sequencesof multivariable binding hypotheses, combining probability values under a naive bayesian assumption
    *
    * @param firstSequence  -- the first sequence of multivariable binding hypotheses
    * @param secondSequence -- the second sequence of multivariable binding hypotheses
    * @return
    */
  def mergeCrossMappingCandidateLists(firstSequence: Seq[CrossMappingCandidateList],
                                      secondSequence: Seq[CrossMappingCandidateList]): Seq[CrossMappingCandidateList] = {
    for (c1 <- firstSequence; c2 <- secondSequence)
      yield CrossMappingCandidateList(c1.bindings ++ c2.bindings, c1.probability * c2.probability)
  }

}
