/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution

import ai.thinkingrobots.trade._

import java.util
import scala.collection.JavaConverters._
import scala.collection.mutable
import scala.collection.mutable.ListBuffer
import scala.language.implicitConversions
import edu.tufts.hrilab.fol._
import edu.tufts.hrilab.fol.util.Utilities
import org.slf4j.{Logger, LoggerFactory}

//TODO:brad: at some point this could be optimized to make some of the methods static via Scala companion object
class Resolver(groups: java.util.List[String]) {
  val orderer: ConstraintOrderer = new sizeOrderer
  final val log: Logger = LoggerFactory.getLogger(getClass)
  val TAU = 0.1
  final val GENTAU = 0.8

  val additionalConstraints = new TRADEServiceConstraints()
  if (!groups.isEmpty) {
    groups.asScala.foreach(additionalConstraints.inGroups(_))
  }

  class ConsultantInfo(val kbName: String, val tsi: TRADEServiceInfo, val tsc: TRADEServiceConstraints) {
    override def toString: String = kbName + "Info"
  }

  var consultants: Iterable[ConsultantInfo] = null
  protected var propertyCache: mutable.Map[ConsultantInfo, mutable.Buffer[Property]] = null

  updateConsultCache(null);

  //brad: for every new Resolver get list of consultants(as TSI+ kbName) and property cache (call get properties handled on everything)
  def updateConsultCache(actor: String) {
    //todo: (pete, brad) this feels kind of hacky. specifically distinguishing between the "self wants everything" vs "!self only wants its own agent group" fixable with better groups implementations?
    if (actor != null && !actor.contains("self")) {
      //TODO:brad:make agent lookup more general
      consultants = TRADE.getAvailableServices(new TRADEServiceConstraints().name("getKBName").argTypes().inGroups("agent:" + actor)).asScala.map(
        c => new ConsultantInfo(c.call(classOf[String]), c, new TRADEServiceConstraints().inGroups(c.getGroups.toArray(new Array[String](0)): _*))
      )
    } else {
      consultants = TRADE.getAvailableServices(additionalConstraints.name("getKBName").argTypes()).asScala.map(
        c => new ConsultantInfo(c.call(classOf[String]), c, new TRADEServiceConstraints().inGroups(c.getGroups.toArray(new Array[String](0)): _*))
      )
    }
    updatePropertyCache();
  }

  def updatePropertyCache(): Unit = {
    propertyCache = new mutable.HashMap[ConsultantInfo, mutable.Buffer[Property]]()
    consultants.map(c => propertyCache += (c -> TRADE.getAvailableService(c.tsc.name("getPropertiesHandled").argTypes())
      .call(classOf[java.util.List[Term]]).asScala.map(new Property(_))))
  }

  def getEntityForReference[E](ref: edu.tufts.hrilab.fol.Symbol, entityJavaType: Class[E]): E = {
    val c = consultants.filter(c => ref.getName.contains(c.kbName))

    if (c.size != 1) {
      log.warn("[getEntityForReference] Multiple consultants with the same kbName, using the first one " + c)
    }

    //call convertToType va tsi, return results
    TRADE.getAvailableService(c.head.tsc.name("convertToType").argTypes(classOf[Symbol], classOf[Class[E]]))
      .call(entityJavaType, ref, entityJavaType)
  }

  def getEntityForReference[E](ref: edu.tufts.hrilab.fol.Symbol, entityJavaType: Class[E], constraints: java.util.List[Term]): E = {
    val c = consultants.filter(c => ref.getName.contains(c.kbName))

    if (c.size != 1) {
      log.warn("[getEntityForReference] Multiple consultants with the same kbName, using the first one " + c)
    }

    //call convertToType va tsi, return results
    TRADE.getAvailableService(c.head.tsc.name("convertToType").argTypes(classOf[Symbol], classOf[Class[E]], classOf[java.util.List[Term]]))
      .call(entityJavaType, ref, entityJavaType, constraints)
  }

  def assertProperties(ref: edu.tufts.hrilab.fol.Symbol, properties: java.util.List[Term]): Boolean = {
    val c = consultants.filter(c => ref.getName.contains(c.kbName))

    if (c.size != 1) {
      log.warn("[assertProperties] Multiple consultants with the same kbName, using the first one " + c)
    }

    var success = false;
    try {
      success = TRADE.getAvailableService(c.head.tsc.name("assertProperties").argTypes(classOf[Symbol], classOf[java.util.List[Term]])).call(classOf[Boolean], ref, properties)
    } catch {
      case e: Exception => {
        log.error("exception calling assertProperties on: " + c.head.kbName, e);
      }
    }
    success;
  }

  def positReference(properties: java.util.List[Term], actor: Symbol): Symbol = {
    updateConsultCache(if (actor != null) actor.toString else null)
    val allProperties = consultants.map(
      c => (c, TRADE.getAvailableService(c.tsc.name("getPropertiesHandled").argTypes())
        .call(classOf[java.util.List[Term]]))
    )

    val matchingConsultants = allProperties.filter(
      c => {
        //check if all of the members of properties match (name and arg semantic type) the properties handled by any of the consultants
        properties.asScala.map(
          q => {
            c._2.asScala.map(
              p => p.getName == q.getName && p.getArgs.size() == q.getArgs.size() && p.getArgs.get(0).getType == q.getArgs.get(0).getType
            ).reduce((x, y) => x || y)
          }
        ).reduce((x, y) => x || y)
      }
    )

    if (matchingConsultants.isEmpty) {
      log.warn("[positReference] no consultant found that can handle: " + properties)
      return null;
    } else if (matchingConsultants.size != 1) {
      log.warn("[positReference] Multiple consultants with the same kbName, using the first one " + matchingConsultants)
    }

    val vars: util.List[Variable] = Utilities.getUnboundVariables(properties);
    try {
      val m = TRADE.getAvailableService(matchingConsultants.head._1.tsc.name("createReferences").argTypes(classOf[java.util.List[Variable]]))
        .call(classOf[java.util.Map[Variable, Symbol]], vars)

      TRADE.getAvailableService(matchingConsultants.head._1.tsc.name("assertProperties").argTypes(classOf[java.util.Map[Variable, Symbol]], classOf[java.lang.Double], classOf[java.util.List[Term]]))
        .call(classOf[Boolean], m, 1.0.asInstanceOf[Object], properties)

      if (m.size() > 1) {
        log.warn("[positReference] created more than 1 reference. Returning first reference. Properties: " + properties)
      }
      val refID = m.get(vars.get(0));
      log.debug("[positReference] posited new reference: " + refID);
      refID;
    } catch {
      case e: TRADEException => {
        log.error("[createReference] exception calling createReference", e)
        null;
      }
    }

  }

  //generate binding hypotheses, given a sequence of properties, a concretenessOrdering of the variables used in those properties, and a sequence of initial binding hypotheses, as well as a flag about whether to posit to LTM or not
  def resolve(query: Seq[Property], co: Seq[Variable], initialHyps: Seq[Hypothesis], posit: Boolean): Seq[Hypothesis] = {
    log.debug("[resolve]")
    val T: List[Variable] = getVariableMapping(query)
    log.debug("[resolve] T: {}", T)
    val boundOrderedQuery = query.map(_.boundForm(T)).filter(t =>
      initialHyps.headOption match {
        case Some(h) => true //t.predicateForm.getVars.exists(v => !h.assignments.exists(vId => vId._1.equals(v.getName)))
        case None => true
      }).sortWith(orderer.order)

    log.debug("[resolve] BOQ: {}", boundOrderedQuery)
    val (solutions: Seq[Hypothesis], remainingVars: Seq[Variable]) = getSolutions(boundOrderedQuery, co, initialHyps)
    val abandonedVariables = co.diff(remainingVars)
    log.debug("[resolve] Valid Solutions: ")
    solutions.foreach(x => log.debug("\t SOL: {}", x))

    if (posit && abandonedVariables.nonEmpty) {
      log.debug("[resolve] Positing before returning solutions")
      completeSolutions(solutions, abandonedVariables, boundOrderedQuery)
    } else {
      log.debug("[resolve] Returning solutions; no positing necessary")
      solutions
    }
  }

  def completeSolutions(s: Seq[Hypothesis], av: Seq[Variable], q: Seq[Property]): Seq[Hypothesis] = {
    //remove 'unfamiliar' property from q since it is not an actual identifying property
    var filtered: Set[Property] = q.toSet
    q.foreach { t =>
      if (t.nonDSPredicateForm.getName().equals("unfamiliar")) {
        filtered -= t
      }
    }
    val q2: Seq[Property] = filtered.toSeq
    //version without "unfamiliar" properties

    //Only complete most likely hypothesis, as there's no MHT currently
    log.trace("IN COMPLETE SOLUTIONS")
    log.trace("compSol: " + s + ", " + av + ", " + q2)
    val bestS = s.headOption.getOrElse(Hypothesis(Map[Variable, Symbol](), 1.0))

    val cons: Seq[ConsultantInfo] = av.flatMap {
      v => {
        q2.find(_.predicateForm.getVars.asScala.exists(x => x.getName.equals(v.getName)))
          .flatMap {
            p =>
              consultants.find {
                c =>
                  TRADE.getAvailableService(c.tsc.name("getPropertiesHandled").argTypes())
                    .call(classOf[java.util.List[Term]]).asScala.exists(new Property(_).matches(p))
              }
          }
      }
    } // .distinct
    //TODO:brad:why was .distinct commented out?

    val varConMap = av.zip(cons)

    val newAssignments = bestS.assignments ++
      varConMap.flatMap {
        case (v: Variable, c: ConsultantInfo) => {
          val vars = Seq(new Variable(v.getName, c.kbName)).asJava
          TRADE.getAvailableService(c.tsc.name("createReferences").argTypes(classOf[java.util.List[Variable]]))
            .call(classOf[java.util.Map[Variable, Symbol]], vars).asScala
        }
      }

    //add all the other properties
    // log.debug("CONS: " + cons)
    consultants.foreach { c =>
      try {
        TRADE.getAvailableService(c.tsc.name("assertProperties").argTypes(classOf[java.util.Map[Variable, Symbol]], classOf[java.lang.Double], classOf[java.util.List[Term]]))
          .call(classOf[Boolean],
            //        newAssignments.map(a => new Variable(a._1) -> new Symbol(a._2)).asJava,
            new java.util.HashMap[Variable, Symbol](newAssignments.asJava),
            1.0.asInstanceOf[Object], //brad:this needs to bee like this for Scala serialization
            q2.map(_.nonDSPredicateForm).asJava)
      } catch {
        case e: TRADEException => log.error("[completeSolutions] exception calling assertProperties", e)
      }
    }

    //return the new hypothesis
    Seq(Hypothesis(newAssignments, bestS.likelihood))
  }

  /**
   * Temporary helper method to convert between new java data structure and old scala bindings data structure
   *
   * @param j
   * @return
   */
  def convertBindingForm(j: java.util.Map[Variable, Symbol]): Seq[(String, String)] = {
    j.asScala.toList.map(b => (b._1.getName, b._2.getName))
  }

  def getSolutions(bOQ: Seq[Property], co: Seq[Variable], initialHyps: Seq[Hypothesis]): (Seq[Hypothesis], Seq[Variable]) = {
    val solutions = bfs(bOQ, initialHyps)
    // log.debug("OWR getSolutions solutions: "+solutions)
    // log.debug("OWR getSolutions co: "+co)
    if (solutions.isEmpty && co.nonEmpty) {
      val rr = removeReferences(bOQ, co.head)
      // log.debug("co: "+co+", rr: "+rr)
      if (co.tail.nonEmpty && rr.nonEmpty) {
        // log.debug("recursing: "+rr+", "+co.tail)
        return getSolutions(rr, co.tail, initialHyps)
      } else {
        // log.debug("OWR getSolutions returning [1]: "+(solutions,co.tail))
        return (solutions, co.tail)
      }
    }
    log.debug("[getSolutions] returning ({},{}) ", solutions, co, "")
    (solutions, co)
  }

  def removeReferences(query: Seq[Property], variable: Variable): Seq[Property] = {
    log.debug("removing {} from {}", variable, query, "")
    log.debug("types: {}", query.flatMap(_.predicateForm.getVars.asScala).map(_.getType))
    log.debug("names: {}", query.flatMap(_.predicateForm.getVars.asScala).map(_.getName))
    query.filterNot(_.predicateForm.getVars.asScala.exists(_.equals(variable)))
  }

  //
  /* COMMON HELPER METHODS */
  //

  //TODO:brad:convert to utility method?
  def scoreT(t: List[Variable], S: Seq[Property]): Double = {
    //changed product to sum so that we can get the highest likely match, not all or nothing
    val ret = S.foldLeft(1.0)(
      (prod, s) =>
        prod + Pts(s, s.boundForm(t))
    )
    t foreach { v =>
      log.debug("Variable " + v + " Type " + v.getType() + " Properties " + S + " Score " + ret)
    }
    ret
  }

  //TODO:brad:convert to utility method?
  def Pts(s: Property, st: Property): Double = {
    val ret = gamma(st) match {
      case 0 => 0.0
      case g => 1.0 / gamma(s)
    }
    var sarg = s.predicateForm.get(0).asInstanceOf[Term].get(0);
    var starg = st.predicateForm.get(0).asInstanceOf[Term].get(0);
    log.debug("Pts for " + sarg.asInstanceOf[Variable].getType() + " " + s + " / " + starg.asInstanceOf[Variable].getType() + " " + st + ": " + ret)
    ret
  }

  def gamma(s: Property): Int = {
    //    log.trace("[GAMMA] consultants: "+consultants)
    //    log.trace("[GAMMA] PC: "+propertyCache.values.flatten)
    //    log.trace("[GAMMA] match: "+propertyCache.values.flatten.filter(_.matches(s)))
    //    log.trace("[GAMMA] handled properties: "+consultants.flatMap(_.getPropertiesHandled.asScala))
    //    log.trace("[GAMMA] matching: "+consultants.flatMap(_.getPropertiesHandled.asScala).count(_.matches(s)))
    //consultants.flatMap(_.getPropertiesHandled.asScala).count(_.matches(s))
    val g = propertyCache.values.flatten.count(_.matches(s))
    g
  }

  protected def diff(t: (Hypothesis, Seq[Property])): Double = t._1.likelihood

  private val diffOrdering = Ordering.by(diff)

  //========================================================================
  // Calculation of optimal binding t:V->K
  //========================================================================

  def getVariableMapping(S: Seq[Property]): List[Variable] = {
    //    log.debug("gVM: CONSULTANTS: " + consultants.map(_.getKBName))
    val SV = S.flatMap(_.predicateForm.getOrderedVars.asScala).distinct
    SV foreach { x => log.debug("[getVariableMapping] {} -> {}", SV, x, "") }
    val T: Seq[List[Variable]] = SV.tail.foldLeft(
      //Brad: if the variable has a semantic type use it, otherwise assume you don't know which consultant...
      consultants.map(c => List(new Variable(SV.head.getName, if (SV.head.getType.isEmpty) c.kbName else SV.head.getType))).toList
    )(
      (M, V) => {
        (M, V) match {
          case (m: Seq[List[Variable]], v: Variable) =>
            for (mi: List[Variable] <- m; c: String <- consultants.map(_.kbName))
              yield mi :+ new Variable(v.getName, c)
        }
      }
    )
    //log.debug("mapping scores: " + T.map(t => (t, scoreT(t, S))))
    if (T.isEmpty) {
      log.error("Variable mapping failed. Likely because no consultant found supporting supplied kbName")
      return List()
    }
    val bestMapping = T.maxBy(t => scoreT(t, S))
    log.debug("Best MappingScore: " + bestMapping)
    bestMapping
  }

  //line 8
  protected def initialDomain(v: String): Seq[Symbol] = {

    consultants.find(_.kbName.equals(v)) match {
      case Some(c) => TRADE.getAvailableService(c.tsc.name("getInitialDomain").argTypes(classOf[java.util.List[Term]]))
        .call(classOf[java.util.List[Symbol]], new java.util.ArrayList(Seq[Property]().asJava)).asScala
      case None => throw new Exception("No consultant keyed with name " + v)
    }
  }

  //line 18
  //Calculates new likelihood for applyConstraint, or simply assess probability...
  def assess(h: Hypothesis, prop: Property): java.lang.Double = {
    log.trace("In assess with " + h + " -- " + prop)
    //val jLikelihood: java.lang.Double = h.likelihood
    //    val hMap = h.assignments.map { case (v, cid) => v -> long2Long(cid.split("_")(1).toLong) }.toMap
    //    val hMap = h.assignments.map { case (v, cid) => new Variable(v) -> new Symbol(cid) }
    //        val hMap = h.assignments.map { case (v, cid) => v -> cid }
    //    val hMap = h.assignments.map { case (v, cid) => Factory.createVariable(v) -> Factory.createSymbol(cid) }

    val propLikelihood: Double = getConsultant(h, prop) match {
      case Some(consultantInfo) =>
        log.debug("Chose consultant: " + consultantInfo.kbName)
        //        TRADE.callThe(consultantInfo.tsi, "process",prop, hMap.asJava).asInstanceOf[Double]
        try {
          TRADE.getAvailableService(consultantInfo.tsc.name("process").argTypes(classOf[Term], classOf[java.util.Map[Variable, Symbol]]))
            .call(classOf[Double], prop.nonDSPredicateForm, new java.util.HashMap[Variable, Symbol](h.assignments.asJava))
        } catch {
          case e: Exception => log.error("[Assess] Exception calling process", e)
            val zero: java.lang.Double = 0.0
            zero
        }
      case None =>
        log.debug("Couldn't find a consultant for hypothesis and property " + h + " " + prop)
        val zero: java.lang.Double = 0.0
        zero
    }
    h.likelihood * propLikelihood
  }

  def getAllActivatedEntities: java.util.Map[Symbol, java.lang.Double] = {
    //    log.debug("[gAAE] Consultants = " + consultants.map(_.getKBName))

    val result: java.util.Map[Symbol, java.lang.Double] =
      consultants.foldRight(new java.util.HashMap[Symbol, java.lang.Double]()) {
        (c: ConsultantInfo, r: java.util.HashMap[Symbol, java.lang.Double]) =>
          r.putAll(
            try {
              TRADE.getAvailableService(c.tsc.name("getActivatedEntities").argTypes())
                .call(classOf[java.util.Map[Symbol, java.lang.Double]])
            } catch {
              case e: Exception => {
                log.error("[getAllActivatedEntities] consultant: " + c.kbName, e);
                new util.HashMap[Symbol, java.lang.Double] {}
              }
            }
          );
          r
      }
    log.debug("[gAAE] Result: " + result)
    result
  }

  protected def getActivatedEntities(groups: util.Collection[String]): java.util.Map[Symbol, java.lang.Double] = {
    consultants.find(_.tsi.getGroups.containsAll(groups)) match {
      case Some(c) => TRADE.getAvailableService(c.tsc.name("getActivatedEntities").argTypes())
        .call(classOf[java.util.Map[Symbol, java.lang.Double]])
      case None => throw new Exception("No consultant keyed with groups " + groups)
    }
  }

  protected def getConsultant(h: Hypothesis, p: Property): Option[ConsultantInfo] = {
    val bf = p.boundForm(h.assignments.keys.toSeq)
    log.debug("FINDING CONSULTANT FOR: " + bf + " / " + bf.predicateForm.toString)

    log.debug("hypothesis ids: " + h.assignments.map { case (v, id) => id.getType })

    //Brad: eliminate consultants whose name doesn't match any of the ref ids in the hypothesis
    val filteredConsultants = consultants.filter(
      c => h.assignments.exists(
        a => a._2.getType.equals(c.kbName))
    )

    //    val filteredConsultants = consultants.filter(c => h.assignments.exists(a =>
    //      a._2.getName.split("_")(0).equals(c.getKBName)))

    log.debug("filtered consultants: {}", filteredConsultants)
    log.trace("property cache: {}", propertyCache)

    //    filteredConsultants.foreach(c => {
    //      log.trace(c + " handles: " + propertyCache(c).map(x => x.toString + " / " + x.predicateForm.toString))
    //      log.debug(c + " handles " + bf + "?" + propertyCache(c).exists(_.matches(bf)))
    //    })

    filteredConsultants.find {
      propertyCache(_).exists(_.matches(bf))
    }
  }

  //Private methods added by brad that are duplicates that exist in all derived classes
  implicit def convMap(al: Map[String, Long]): Map[String, java.lang.Long] =
    al.map {
      case (s, l) => {
        val ll: java.lang.Long = l
        (s, ll)
      }
    }

  //========================================================================
  // REG //brad: not sure why REGenerator was a separate file for so long
  //========================================================================

  //HELPERS//------------------------------------------------------------
  //TODO:brad:this is where bad stuff happens, so that assess can be reused...
  //assess and/or process should ultimately be refactored
  def b2h(b: mutable.Map[Symbol, Symbol]): Hypothesis = {
    val h: mutable.Map[Variable, Symbol] = b.map { case (k: Symbol, v: Symbol) => new Variable(k.getName) -> v }
    Hypothesis(h.toMap, 1.0)
  }

  def allComboBindings(args: Seq[Symbol]): Seq[mutable.ListMap[Symbol, Symbol]] = {
    log.debug("ACB CALLED WITH: " + args)
    val candLists: Seq[Seq[mutable.ListMap[Symbol, Symbol]]] = args.flatMap { case v: Variable =>
      //changed to prevent redundant type expression (i.e. objects_objects_4 instead of objects_4)
      consultants.find(_.kbName.equalsIgnoreCase(v.getType)).map(c =>
        TRADE.getAvailableService(c.tsc.name("getInitialDomain").argTypes(classOf[java.util.List[Term]]))
          .call(classOf[java.util.List[Symbol]], new util.ArrayList[Property]())
          .asScala.map(id => mutable.ListMap(v.asInstanceOf[Symbol] -> id))
      )
    }
    log.debug("candLists: " + candLists)
    //def mergeClists(fullList: Seq[Seq[(String,String)]], newList:Seq[(String,String)])
    val comboBindings: Seq[mutable.ListMap[Symbol, Symbol]] = candLists.tail.foldLeft(candLists.head) {
      case (a: Seq[mutable.ListMap[Symbol, Symbol]], b: Seq[mutable.ListMap[Symbol, Symbol]]) => for (ai <- a; bi <- b) yield ai ++= bi
    }
    log.debug("comboBindings: " + comboBindings)
    comboBindings
  }

  //---------------------------------------------------------------------
  def getProperties(ref: Symbol): java.util.List[Term] = {
    val consultantName = ref.getType;
    log.debug("CONSULTANTNAME: " + consultantName)
    val consultant: ConsultantInfo = consultants.find(_.kbName.equalsIgnoreCase(consultantName)).getOrElse(return new java.util.ArrayList[Term])
    try {
      TRADE.getAvailableService(consultant.tsc.name("getAssertedProperties").argTypes(classOf[Symbol]))
        .call(classOf[java.util.List[Term]], ref)
    } catch {
      case e: TRADEException => log.error("[getProperties] call of getAssertedProperties for ref: " + ref + " from: " + consultant.kbName, e)
        new java.util.ArrayList[Term]()
    }
  }

  //brad: the structure of this is wild...
  def generateRE(ref: Symbol): java.util.LinkedHashMap[Symbol, java.util.List[Term]] = {
    log.debug("generating RE -resolver")
    updateConsultCache(null)

    //    var idToUse = 0
    var found: java.util.LinkedHashMap[Symbol, java.util.List[Property]] = new java.util.LinkedHashMap[Symbol, java.util.List[Property]]()
    val refQueue: mutable.Queue[Symbol] = mutable.Queue(ref)
    while (refQueue.nonEmpty) {
      log.debug("Referent Queue: " + refQueue)
      val r = refQueue.dequeue()
      //check if r exists in found
      if (!found.asScala.contains(r)) {
        //should get all objects
        STMGenerateRETuple(r, found.asScala.keys.toSeq) match {
          case None => return new java.util.LinkedHashMap[Symbol, java.util.List[Term]]
          case Some(stmResult) =>
            val (stmTuple: (Symbol, java.util.List[Property]), distractors: List[Symbol]) = stmResult
            val newTuple = generateRETuple(r, found.asScala.keys.toSeq, stmTuple, distractors)
            log.debug("Just generated new Tuple: " + newTuple)
            newTuple.foreach { d =>
              found.put(d._1, d._2)
              val newRefs = refsIn(d._2.asScala.toList)
              refQueue ++= newRefs
            }
        }
      }
    }

    def STMGenerateRETuple(ref: Symbol, found: Seq[Symbol]): Option[((Symbol, java.util.List[Property]), List[Symbol])] = {
      val consultantName = ref.getType
      //val (consultantName) = "object"
      log.debug("CONSULTANTNAME: " + consultantName)
      val filteredConsultants: Iterable[ConsultantInfo] = consultants.filter(_.kbName.equalsIgnoreCase(consultantName))

      var description = new java.util.ArrayList[Property]()
      //changed to prevent distractors from being a list of "objects_objects_#" strings in the case of object consultants
      var domain = new java.util.ArrayList[Symbol]()
      filteredConsultants.foreach(c => domain.addAll(TRADE.getAvailableService(c.tsc.name("getInitialDomain").argTypes(classOf[java.util.List[Term]]))
        .call(classOf[java.util.List[Symbol]], new java.util.ArrayList[Term]())))
      //list of other existing refs
      log.debug("domain: " + domain)
      var distractors: List[Symbol] = domain.asScala.filterNot(_.equals(ref)).toList //This assumes it won't be confused with something from another domain.
      log.debug("distractors: " + distractors)

      //instead of getPropertiesHandled, we get the STM
      //proties of ref
      var propsList = new java.util.ArrayList[Property]()
      filteredConsultants.foreach(c =>
        propsList.addAll(
          //convert to properties for internal rr use
          TRADE.getAvailableService(c.tsc.name("getAssertedProperties").argTypes(classOf[Symbol]))
            .call(classOf[java.util.List[Term]], ref).asScala.map(t => new Property(t)).asJava
        )
      )

      val props = mutable.Stack(propsList.asScala.map((_, mutable.ListMap[Symbol, Symbol]())): _*) //LINE 1

      //val props = mutable.Stack(consultant.getPropertiesHandled.map((_, Map[String, String]())): _*) //LINE 1
      log.debug("PROPS: " + props)
      //TODO: old code didn't account for there only being a single object ref, thus no distractors
      while (props.nonEmpty && distractors.nonEmpty) {
        //LINE 2
        log.debug("Distractors: " + distractors + "; P: " + props)
        val (prop: Property, bindings: mutable.ListMap[Symbol, Symbol]) = props.pop()
        log.debug("TRYING: " + prop)
        val args: List[Symbol] = prop.predicateForm.get(0).asInstanceOf[Term].getArgs.asScala.toList

        log.debug("BINDINGS: " + bindings + " /// FOUND: " + found)
        if (bindings.values.toList.intersect(found).nonEmpty) {
          log.debug("Refers to something already described, dropping")
        } else {
          log.debug("LINE6 -- " + prop + " -- " + bindings) //LINE 6
          log.debug("args: " + args)
          args.find(s => !bindings.contains(s)) match {
            case None => throw new Exception("No variables left unfilled?")
            case Some(missing) =>
              log.debug("MISSING: " + missing)
              //rebinds query to refer to each distractor, then checks whether property holds for that distractor
              //elimDist is distractors which don't have the property we're looking for
              val elimDist = distractors.filter(d => assess(b2h(bindings + (ref -> d)), prop) < GENTAU)
              //LINE 8
              log.debug("DISTRACTORS ELIMINATED: " + elimDist)
              // if the set of distractors for which the property does not appear in STM != 0
              if (elimDist.nonEmpty || distractors.isEmpty) {
                //LINE 9
                //TODO:brad: why is this Variable?
                description.add(prop.boundForm(bindings.map(x => new Variable(x._1.getName, x._2.getName)).toList))
                //LINE 10
                distractors = distractors diff elimDist //LINE 11
                log.debug("DESCRIPTION: " + description)
                log.debug("DISTRACTORS: " + distractors)
              }
          }
        }
      }
      //commented out because it ruins reference expression generation for clarification requests
      /*while (props.nonEmpty && !distractors.nonEmpty) {
        val (prop: Property, bindings: mutable.ListMap[Symbol, Symbol]) = props.pop()
        description.add(prop.boundForm(bindings.map(x => new Variable(x._1.getName, x._2.getName)).toList))
        log.debug("DESCRIPTION: " + description)
      }*/
      Option((ref, description), distractors) // Line 12
    }

    def refsIn(props: List[Property]): Seq[Symbol] = {
      props.flatMap { p: Property => refsInS(p.predicateForm.get(0)) }
    }

    def refsInS(s: Symbol): Seq[Symbol] = {
      s match {
        case v: Variable =>
          log.debug("refsInS: considering: " + v.getName + " - " + v.getType)
          //TODO: is there a better name for this
          if (v.getType.contains("_")) Seq(Factory.createSymbol(v.getType + ":" + v.getType.split("_")(0))) else Seq()
        case t: Term => t.getArgs.asScala.toList.flatMap(refsInS)
        case _ => Seq()
      }
    }

    def generateRETuple(ref: Symbol, found: Seq[Symbol], stmTuple: (Symbol, java.util.List[Property]), d: List[Symbol]): Option[(Symbol, java.util.List[Property])] = {
      val consultantName = ref.getType
      //val (consultantName) = "object"
      log.debug("CONSULTANTNAME: " + consultantName)
      val consultant: ConsultantInfo = consultants.find(_.kbName.equalsIgnoreCase(consultantName)).getOrElse(return None)

      var description = stmTuple._2
      var distractors = d
      val propsList = TRADE.getAvailableService(new TRADEServiceConstraints().name("getPropertiesHandled").argTypes().inGroups(consultant.tsi.getGroups.toArray(new Array[String](0)): _*))
        .call(classOf[java.util.List[Term]]).asScala.map(new Property(_))
      val props = mutable.Stack(propsList.map((_, mutable.ListMap[Symbol, Symbol]())): _*) //LINE 1
      log.debug("PROPS: " + props)
      //TODO: old code didn't account for there only being a single object ref, thus no distractors
      while (props.nonEmpty && distractors.nonEmpty) {
        //LINE 2
        log.debug("Distractors: " + distractors + "; P: " + props)
        val (prop: Property, bindings: mutable.ListMap[Symbol, Symbol]) = props.pop()
        log.debug("TRYING: " + prop)
        val args: List[Symbol] = prop.predicateForm.get(0).asInstanceOf[Term].getArgs.asScala.toList
        //val args: List[Variable] = prop.predicateForm.get(0).asInstanceOf[Term].getOrderedVars.asScala.toList
        log.debug("args: " + args)
        if (args.size > 1 && bindings.isEmpty) {
          //LINE 3
          log.debug("FOLDING ON ACB")

          //val candidateVariables: Seq[Variable] = args.flatMap {
          val candidateVariables: Seq[Symbol] = args.flatMap {
            case v: Variable if v.getType.equalsIgnoreCase(consultantName) => Option(v)
            case _ => None
          }
          log.debug("candidate variables: " + candidateVariables)
          candidateVariables.flatMap {
            cv => allComboBindings(args.filterNot(x => x.getName.equals(cv.getName)))
          }.foreach {
            b => props.push((prop, b))
          }

        } else {
          log.debug("BINDINGS: " + bindings + " /// FOUND: " + found)
          if (bindings.values.toList.intersect(found).nonEmpty) {
            log.debug("Refers to something already described, dropping")
          } else {
            log.debug("LINE6 -- " + prop + " -- " + bindings) //LINE 6
            //val args:Seq[edu.tufts.hrilab.fol.Symbol] = prop.predicateForm.get(0).asInstanceOf[edu.tufts.hrilab.fol.Term].getArgs.toList
            log.debug("args: " + args)

            args.find(s => !bindings.contains(s)) match {
              case None => throw new Exception("No variables left unfilled?")
              case Some(missing) =>
                log.debug("Missing: " + missing)
                if (assess(b2h(bindings += (missing -> ref)), prop) >= GENTAU) {
                  //LINE 7
                  log.debug("IT'S A MATCH!")
                  val elimDist = distractors.filter(did => assess(b2h(bindings += (missing -> did)), prop) < GENTAU)
                  //LINE 8
                  log.debug("DISTRACTORS ELIMINATED: " + elimDist)
                  if (elimDist.nonEmpty || distractors.isEmpty) {
                    //LINE 9
                    //Ravenna: fixed bug here where name was used instead of type
                    description.add(prop.boundForm(bindings.map(x => new Variable(x._1.getName, x._2.getType)).toList))
                    //LINE 10
                    distractors = distractors diff elimDist //LINE 11
                    log.debug("DESCRIPTION: " + description)
                    log.debug("DISTRACTORS: " + distractors)
                  }
                }
            }
          }
        }
      }
      Option((ref, description)) // Line 12
    }

    log.debug("Returning REG: " + found)
    log.debug("In detail..." + found.asScala.values.flatMap(x => x.asScala.map(_.predicateForm.get(0).toUntypedString)))

    //convert Properties to Terms for use outside of RR
    val ret: java.util.LinkedHashMap[Symbol, java.util.List[Term]] = new java.util.LinkedHashMap[Symbol, java.util.List[Term]]()
    found.asScala.map(pair => ret.put(pair._1, pair._2.asScala.map(_.nonDSPredicateForm).asJava))
    ret
  }

  //========================================================================
  // Greedy Best First Search
  //========================================================================
  def bfs(query: Seq[Property], initialHyps: Seq[Hypothesis]): Seq[Hypothesis] = {
    //DIST-COWER Line 1
    val solutions = mutable.ListBuffer[Hypothesis]()

    val hypothesisQueue = new mutable.PriorityQueue[(
      Hypothesis, Seq[Property])]()(diffOrdering)

    //(lines 7-11)
    if (initialHyps.isEmpty)
      initializeHQ(query.headOption)
    else initialHyps.foreach { h =>
      val hseq = query.to[ListBuffer]
      hypothesisQueue.enqueue((h, hseq))
    }

    // log.debug("Calling bfsWithTable with hQ "+hypothesisQueue)
    //DIST-COWER Line 2
    while (hypothesisQueue.nonEmpty) {
      //DIST-COWER Line 3
      val n = hypothesisQueue.dequeue()
      //DIST-COWER Line 4
      if (n._2.nonEmpty)
        //DIST-COWER Line 5ish
        newVariable(n._1, n._2) match {
          //DIST-COWER Lines 5ish-10
          case Some(v: Variable) => expandVariable(n._1, n._2, v)
          //DIST-COWER Lines 11-17
          case None => applyConstraint(n._1, n._2)
          //DIST-COWER Line 18
        }
      //DIST-COWER Lines 19-21
      else solutions += n._1
    }

    //initializeHQ: initializes hypothesis queue by adding entries for
    //the initial domain of the first variable came across
    def initializeHQ(q: Option[Property]) {
      //line 7
      q.map(_.predicateForm.getVars.asScala.head) match {
        case Some(firstVar) =>
          //line 8
          initialDomain(firstVar.getType).foreach {
            consultantAndId =>
              log.trace("Initial Domain of " + firstVar + " includes " + consultantAndId)
              val newBuffer = Map[Variable, Symbol](firstVar -> consultantAndId)
              //line 9
              hypothesisQueue.enqueue((Hypothesis(newBuffer, 1.0), query map (x => x)))
          }
        //line 10
        case None => throw new Exception("Couldn't get the head variable for " + q)
      }
    }

    ////DIST-COWER Line 5ish
    //Looks through a given hypothesis for the first unassigned variable, if any.
    def newVariable(h: Hypothesis, mb: Seq[Property]): Option[Variable] = {
      val assignedVars: Seq[Variable] = h.assignments.keys.toList
      val candidates = mb.head.predicateForm.getVars.asScala
      candidates.find(v => !assignedVars.exists(x => x.getName.equals(v.getName)))
    }

    //DIST-COWER Lines 6-10
    //expandVariable: given a hypothesis and a new variable,
    //adds entries to the hypothesis queue that make the same assumptions
    //as the previous hypothesis, plus a new assumption from the initial
    //domain of the given variable.
    def expandVariable(h: Hypothesis, mb: Seq[Property], v: Variable) {
      //line 14
      initialDomain(v.getType).foreach {
        cAndId =>
          log.trace("Expanding variable " + v + " with candidate " + cAndId)
          //line 15
          hypothesisQueue.enqueue((Hypothesis(h.assignments + (v -> cAndId),
            h.likelihood), mb))
      }
      //line 16
    }

    //Dist-COWER Lines 12-16
    //adapts a hypothesis based on the first in a list of properties,
    //and pops that property if
    def applyConstraint(h: Hypothesis, mb: Seq[Property]) {
      //mapping variable to id
      //Dist-COWER Line 12-13
      val newLikelihood = assess(h, mb.head)
      // log.debug("new likelihood "+newLikelihood+" for "+mb.head+" under "+h)
      //Dist-COWER Line 14
      val mbTail = mb.tail
      //Dist-COWER Line 15
      if (newLikelihood > TAU) {
        //TODO:brad: took this from closed world implementation, wasn't in OW, do we still want/need it?
        //line 21
        if (mbTail.isEmpty)
          //line 22
          solutions += Hypothesis(h.assignments, newLikelihood)
        //line 23
        else
          //line 24
          //Dist-COWER Line 16
          hypothesisQueue.enqueue((Hypothesis(h.assignments, newLikelihood), mbTail))
      }
    }

    log.debug("Solutions: " + solutions)
    solutions
  }

}
