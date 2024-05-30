/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.learning;

import java.util.*;
import java.util.stream.Collectors;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
  LearningState is used to store relevant information during learning of a new action (task or skill), including
  the roles (argument and local variables), action steps, effects (success and failure), conditions (pre and operational),

  also contains additional information about skills and instructions
  
  This class is modified by the ActionLearning class
*/

public class LearningState {
  private static final Logger log = LoggerFactory.getLogger(LearningState.class);
  private final LearningState previousLearningState; // parent learning state
  private final String name;
  private String description;
  private List<Symbol> inputRoles = new ArrayList<>(); //arguments of learned action
  private List<EventSpec> eventSpecs= new ArrayList<>(); // list of action steps

  /**
   * Local variable declaration event specs
   */
  private List<EventSpec> declarationSpecs= new ArrayList<>();
  /**
   * Local variable destruction event specs
   */
  private List<EventSpec> destructionSpecs= new ArrayList<>();
  private Set<Condition> conditions= new HashSet<>(); // set of conditions (pre & overall)
  private Set<Effect> effects= new HashSet<>(); // set of effects

  //Map of symbolic values  to action bindings (for input or local roles) within the action
  private LinkedHashMap<Symbol, ActionBinding> valueActBindingMap= new LinkedHashMap<>();
  private Map<List<Term>,String> localVariableProperties = new HashMap<>();

  //TODO:brad: get these values from the currently active consultants
  private Set<String> consultantTypes = Set.of(new String[]{"physobj"});//,"gofapose","yumipose","mobileyumipose","location"});
  private int localVariableCounter=0;
  private int roleNameCounter = 0;

  public LearningState(Predicate predicate) {
    this(predicate, null);
  }

  public LearningState(Predicate signature, LearningState previousLearningState) {
    this.name = signature.getName();
    this.previousLearningState = previousLearningState;

    // create entries for each predicate argument in value-binding map
    // this will ensure ordering of required args
    boolean firstArg = true;
    for (Symbol arg : signature.getArgs()) {
      if (firstArg) {
        // assumes actor is always first arg
        firstArg = false;
        String actorName = "?actor"; // create designated actor role name
        // create role for actor
        //TODO:brad: could we somehow get the semantic type of the "agent" that is learning?
        ActionBinding actorRole = new ActionBinding.Builder(actorName, Symbol.class).setSemanticType("agent").build();
        valueActBindingMap.put(arg, actorRole); // put the value and associate role in map
      } else {
        inputRoles.add(arg); // add argument value to predicate arguments
        String roleName = getUniqueRoleName();
        valueActBindingMap.put(arg, new ActionBinding.Builder(roleName, arg.getClass()).setSemanticType(arg.getType()).setValue(null).build());
      }
    }
  }

  /**
   * Generates a new LearningState which is a copy of {@code adbe} but with the name from {@code action}
   * @param adbe ActionDBEntry to clone
   * @param action signature predicate representing new action
   */
  public LearningState(ActionDBEntry adbe, Predicate action) {
    this(action);
    List<ActionBinding> roles =new ArrayList<>(adbe.getRoles());

    eventSpecs.addAll(adbe.getEventSpecs());
    //add input roles
    List<Symbol> args = new ArrayList<>(action.getArgs());
    for (int i = 0; i < args.size(); i++) {
      valueActBindingMap.put(args.get(i), roles.get(i));
    }
    //add local roles
    for (ActionBinding role : roles) {
      if (role.isLocal) {
        localVariableCounter++;
        EventSpec match = null;
        for(EventSpec e: getEventSpecs()){
          if(e.getReturnArgs().contains(role.name)){
           //TODO:brad: is there a better way to do this?
           //assume it is a positReference call? e.g.
           // !var1 = ?actor.act:positReference(and(deepM3Hole(VAR0:physobj),top(VAR0:physobj)));
           List<Term> properties= new ArrayList<>();
           for(Symbol propertyArg: ((Term)e.getPredicateForm().getArgs().get(1)).getArgs()){
             properties.add((Term) propertyArg);
           }
           localVariableProperties.put(properties,role.getName());
           //TODO:should this key be a Symbol? should it exist at all?
           valueActBindingMap.put(Factory.createSymbol(role.name),role);
           declarationSpecs.add(e);
           match = e;
           break; //TODO:brad: could there be multiple?
          }
        }
        if(match != null){
          eventSpecs.remove(match);
        }else {
          //TODO:brad: what does it mean if this actually happens
          valueActBindingMap.put(Factory.createSymbol(role.name), role);
        }
      }
    }

    conditions.addAll(adbe.getPreConditions());
    conditions.addAll(adbe.getOverallConditions());
    effects.addAll(adbe.getEffects().stream().filter(e -> !e.isAutoGenerated()).collect(Collectors.toList()));
    this.description=adbe.getDescription();
    // need to make sure that the cost, timeout, posAff, negAff .... are all added properly
    // should the learning state take that into account?
  }

  public String getName() {
    return name;
  }
  public String getDescription() {
    return description;
  }

  public LearningState getPreviousLearningState() {
    return previousLearningState;
  }

  public Set<Condition> getConditions() {
    return conditions;
  }

  public void addCondition(Condition toAdd) {
    Predicate groundedSemantics = groundArgs(toAdd.getPredicates().keySet().iterator().next());
    Condition grounded = new Condition(groundedSemantics,toAdd.getType());
    this.conditions.add(grounded);
  }

  public void removeCondition(Condition toRemove) {
    //TODO:Brad: this doesn't ground disjunctions because there isn't an easy way to get them as a predicate....
    Predicate toRemoveGrounded = groundArgs(toRemove.getPredicates().keySet().iterator().next());
    for (Condition cond : conditions) {
      // TODO: compare the type -> really need to have condition properly implement it
      Predicate knownCond = cond.getPredicates().keySet().iterator().next();
      if (conditionsEqual(toRemoveGrounded, knownCond)) {
        conditions.remove(cond);
        break;
      }
    }
  }
  public void addEffect(Effect toAdd) {
    Predicate groundedSemantics = groundArgs(toAdd.getPredicate());
    Effect groundedEffect = new Effect(groundedSemantics, toAdd.getType());
    effects.add(groundedEffect);
  }

  //copied from ActionModification

  public void removeEffect(Effect toRemove) {
    Predicate toRemoveGrounded = groundArgs(toRemove.getPredicate());
    for (Effect eff : effects) {
      // TODO: compare the type -> really need to have effect implement it
      if (conditionsEqual(toRemoveGrounded, eff.getPredicate())) {
        effects.remove(eff);
        break;
      }
    }
  }

  private boolean conditionsEqual(Predicate newCondition, Predicate knownCondition) {
    if (newCondition.getName().equals(knownCondition.getName())) {
      List<Symbol> newArgs = newCondition.getArgs();
      List<Symbol> knownArgs = knownCondition.getArgs();
      if (newArgs.size() == knownArgs.size()) {
        //boolean test = newArgs.equals(knownArgs);
        for (int index = 0; index < newArgs.size(); index++) {
          if (!newArgs.get(index).equals(knownArgs.get(index))) {
            return false;
          }
        }
        return true;
      }

    }
    return false;
  }

  public Set<Effect> getEffects() {
    return effects;
  }

  private void declareLocalVariable(Symbol refID){

    String localVarName="!var"+localVariableCounter++;

    addLocalVariableDeclaration(refID,localVarName);

    ActionBinding.Builder lvBindingBuilder= new ActionBinding.Builder(localVarName,Symbol.class).setSemanticType(refID.getType()).setIsLocal(true);
    valueActBindingMap.put(Factory.createSymbol(localVarName),lvBindingBuilder.build());
  }

  private void addLocalVariableDeclaration(Symbol refID, String localVarName){
    log.debug("[declareLocalVariable] got reference arg:"+refID);
    List<Term> properties = getProperties(refID);
    log.debug("[declareLocalVariable] properties:"+properties);

    EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.ACTION);
    esBuilder.setActor("?actor");
    esBuilder.setCommand("positReference");
    Term propsList=Factory.createPredicate("and",properties);
    esBuilder.addInputArg(propsList.toString());
    esBuilder.addReturnArg(localVarName);

    EventSpec propertiesDeclaration = esBuilder.build();
    declarationSpecs.add(0,propertiesDeclaration);
    localVariableProperties.put(properties,localVarName);
  }

  private String getRoleNameForValue(Symbol value) {
    ActionBinding binding= valueActBindingMap.get(value);
    if(binding != null){
      return binding.getName();
    }

    if(consultantTypes.contains(value.getType())){
      List<Term> properties = getProperties(value);


      return getLocalVariableForProperties(properties);
    }
    return null;
  }

  String getLocalVariableForProperties(List<Term> properties){
    for(List<Term> key:localVariableProperties.keySet()){
      boolean allMatch = true;
      for(Term prop: properties){
        boolean matchFound = false;
        for(Term k:key) {
          if(prop.getName().equals(k.getName())){
            matchFound= true;
            break;
          }
        }
        if(!matchFound){
          allMatch=false;
          break;
        }
      }
      if(allMatch){
        return localVariableProperties.get(key);
      }
    }
    log.debug("[getLocalVariableForProperties] local variable not found for properties: "+properties);
    return null;
  }

  private List<Term> getProperties(Symbol ref){
    try {
      return TRADE.getAvailableService(new TRADEServiceConstraints().name("getProperties").argTypes(Symbol.class)).call(List.class,ref);
    } catch (TRADEException e){
      log.error("[getProperties] exception getting properties used to declare local var.",e);
     //TODO:brad: should this be null? what if the ref actually doesn't have properties?
      return new ArrayList<>();
    }
  }

  public List<ActionBinding> getRoles() {
    return new ArrayList<>(valueActBindingMap.values());
  }

  public List<EventSpec> getEventSpecs() {
    return eventSpecs;
  }

  /**
   * Add EventSpec described by {@code p} at {@code index}
   * @param index index in learning state at which to insert new EventSpec
   * @param p Predicate description of new Event spec
   */
  public String addEventSpec(int index, Predicate p){
    //TODO:brad is ist safe to get the actor like this?
    eventSpecs.add(index,buildEventSpecForAction(p.get(0),p));

    List<String> newInputArgs= eventSpecs.get(index).getInputArgs();
    for(String arg: newInputArgs){
      if(localVariableProperties.containsValue(arg)){
        log.warn("[addEventSpec] new local variable declared: "+arg);
        return arg;
      }
    }
    return "";
  }

  public void removeEventSpec(int index){
    removeEventSpec(index,"");
  }

  public void removeEventSpec(int index, String lvToUpdate){
    EventSpec removed= eventSpecs.remove(index);

    //check if it contains a local variable, because we (might) also want to remove the declaration of that local variable.
    for(String arg: removed.getAllArgs()) {
      if (localVariableProperties.containsValue(arg) && !lvToUpdate.equals("")) {
        List<EventSpec> found = new ArrayList<>();
        for (EventSpec declaration : declarationSpecs) {
          if (declaration.getReturnArgs().contains(arg) && !arg.equals(lvToUpdate)) {
            found.add(declaration);
            break;
          }
        }
        declarationSpecs.removeAll(found);
        List<Term> key= null;
        for(Map.Entry<List<Term>,String> lv: localVariableProperties.entrySet()){
          if(lv.getValue().equals(arg)){
            key=lv.getKey();
          }
        }
        if(key != null){
          String removedLv= localVariableProperties.remove(key);
          if(!lvToUpdate.isEmpty()){
            eventSpecs= eventSpecs.stream().map( e ->{if(e.getInputArgs().contains(removedLv)){
              return  new EventSpec(e.getType(),e.toString().replaceAll(removedLv,lvToUpdate));
            } else{
              return e;
            }
            }).collect(Collectors.toList());
          }
        }
      }
    }
  }

  public void removeEventSpecsForLocalVariable(Symbol lvRef){
    List<Term> properties = getProperties(lvRef);

    String lvToRemove= getLocalVariableForProperties(properties);

    eventSpecs= eventSpecs.stream().filter( e -> !e.getInputArgs().contains(lvToRemove)).collect(Collectors.toList());

    declarationSpecs=declarationSpecs.stream().filter( e -> !e.getReturnArgs().contains(lvToRemove)).collect(Collectors.toList());
  }

  /**
   * Helper method to find the action step matching the reference step.
   *
   * @param reference   predicate defining reference event spec
   * @return
   */

  public int getEventStepLocation(Predicate reference) {
    return  getEventStepLocation(eventSpecs,reference);
  }

  private int getEventStepLocation(List<EventSpec> steps, Predicate reference) {
    // find actionStep matching reference step
    int currentLoc = 0;
    Predicate groundedReference = groundArgs(reference);
    for (EventSpec step : steps) {
      if (groundArgs(step.getPredicateForm()).equals(groundedReference)) {
        return currentLoc;
      }
      currentLoc++;
    }
    return -1;
  }

  /**
   * Replaces args in @toGround with their bound values from nameValueMap
   */
  private Predicate groundArgs(Predicate toGround) {
    String predName = toGround.getName(); // should be modType, locType
    List<Symbol> groundedArgs = new ArrayList<>();

    //remove wrapper predicate
    if(ActionLearning.RelType.isRelType(predName) || ActionLearning.ModType.isModType(predName)) {
      for (Symbol arg : toGround.getArgs()) {
        if (arg.isPredicate()) {
          Predicate pArg = (Predicate) arg;
          groundedArgs.add(groundArgs(pArg));
        } else {
          log.error("[groundPredicate] badly formed action modification " + arg);
          return null;
        }
      }
    } else {
      List<String> innerArgs = new ArrayList<>();
      for (Symbol arg : toGround.getArgs()) {
        String argVar = getRoleNameForValue(arg);
        if (argVar == null) {
          if(!arg.isTerm() && consultantTypes.contains(arg.getType())){
            //Check if a local variable exists for the value
            List<Term> properties = getProperties(arg);
            if(getLocalVariableForProperties(properties)!=null){
              innerArgs.add(getLocalVariableForProperties(properties));
            } else{
              innerArgs.add(arg.toString());
            }
          }else {
            innerArgs.add(arg.toString());
          }
        } else {
          innerArgs.add(argVar);
        }
      }
      for (String arg : innerArgs) {
        groundedArgs.add(Factory.createFOL(arg));
      }
    }
    return Factory.createPredicate(predName, groundedArgs);
  }

  /**
   * Replaces args in @toGround with their bound values from @ls
   */
//  private Predicate groundArgs(LearningState ls,Predicate toGround) {
//    String predName = toGround.getName(); // should be modType, locType
//    List<Symbol> groundedArgs = new ArrayList<>();
//    for (Symbol arg : toGround.getArgs()) {
//      if(arg.isPredicate()) {
//        Predicate pArg = (Predicate) arg;
//        //Brad: relations can be more than one deep, but I don't think modifications can be
//        if (ActionLearning.RelType.isRelType(pArg.getName())) {
//          groundedArgs.add(groundArgs(ls, pArg));
//        } else { // predicate is an action, match value to varname
//          List<String> innerArgs = new ArrayList<>();
//          for (Symbol inArg : pArg.getArgs()) {
//            String argVar = ls.getRoleNameForValue(inArg);
//            if (argVar == null) {
//              innerArgs.add(inArg.toString());
//            } else {
//              innerArgs.add(argVar);
//            }
//          }
//          groundedArgs.add(Factory.createPredicate(pArg.getName(), innerArgs.toArray(new String[0])));
//        }
//      }else{
//        log.error("[groundPredicate] badly formed action modification"+arg);
//        return null;
//      }
//    }
//    return Factory.createPredicate(predName, groundedArgs);
//  }

  //build event specs

  /**
   * Main entry point method for adding new event specs ot learning state
   * @param g goal to generate event spec for
   */
  public void createEventSpec(Goal g){
    Predicate goalPred= g.getPredicate();
    Symbol actor = g.getActor();

    // TODO:  handle handle appropriate types of events
    if (goalPred.getName().equals("if") || goalPred.getName().equals("else")) {
      buildControlSpec(goalPred);
    } else {
      eventSpecs.add(buildEventSpecForAction(actor,goalPred));
    }
  }

  //TODO:brad: should this be an event spec constructor? it's just weired because it creates multiple event specs
  private void buildControlSpec(Predicate goalPred){
    if (goalPred.getName().equals("if")) {
      Predicate conditionPred = (Predicate) goalPred.get(0);
      Predicate thenPred = (Predicate) goalPred.get(1);
      if (!thenPred.getName().contains("then")) { //todo: this is not a particularly good check.
        log.error("[createEventSpec] condition form does not have correct 'then' form for body of condition");
      }
      Predicate bodyPred = (Predicate) thenPred.get(0);
      //then we are in a conditional case and expect the form if(conditionAction(),then(bodyAction))
      eventSpecs.add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("if").build());
      //get the condition
      //TODO:brad: is it safe to assume that the actor is arg 0 here?
      eventSpecs.add(buildEventSpecForAction(conditionPred.get(0), conditionPred));
      //then the body
      eventSpecs.add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("then").build());
      //todo: only handling a single actspec in the body. actor assumption is also an issue here.
      //TODO:brad: same actor assumption here
      eventSpecs.add(buildEventSpecForAction(bodyPred.get(0),bodyPred));
    } else if (goalPred.getName().equals("else")) {
      Predicate bodyPred = (Predicate) goalPred.get(0);
      //then we are in a conditional case and expect the form if(conditionAction(),then(bodyAction))
      eventSpecs.add(new EventSpec.Builder(EventSpec.EventType.CONTROL).setCommand("else").build());
      //then the body
      //todo: only handling a single actspec in the body. actor assumption is also an issue here.
      //TODO:brad: same actor assumption here
      eventSpecs.add(buildEventSpecForAction(bodyPred.get(0), bodyPred));
    }
  }


  /**
   handle the different types of goal instructions
   action | goal state = executable plan may be found
   need to ensure the robot was | will be executing the action before waiting for a trajectory
   conditional | control = contain info about structure of action as well as executable plans
   constraints | effects | conditions = non-executable additional information
   */
  private EventSpec buildEventSpecForAction(Symbol actor, Predicate actionPredicate) {
    EventSpec.Builder esBuilder = new EventSpec.Builder(EventSpec.EventType.ACTION);
    esBuilder.setActor(processEventArgument(actor));
    esBuilder.setCommand(actionPredicate.getName());

    List<Symbol> args = actionPredicate.getArgs();
    // add the arguments to the action step
    // assumes all instructions are actions
    for (int index = 1; index < args.size(); index++) {
      Symbol arg = args.get(index);
      String paramName = processEventArgument(arg);
      esBuilder.addInputArg(paramName);
    }
    return esBuilder.build();
  }

  //TODO:brad: combine this with ground args?
  private String processEventArgument(Symbol value) {
    StringBuilder param = new StringBuilder();
    if (value.isTerm() || value.isPredicate()) { // value is term or predicate
      Term valTerm = (Term)value;
      param.append(valTerm.getName());
      param.append("(");
      List<Symbol> args = valTerm.getArgs();
      for (int index=0; index < args.size(); index++) { // for each arg in the predicate
        Symbol arg = args.get(index);
        param.append(processEventArgument(arg)); // recursively process
        if (index < (args.size()-1)) {
          param.append(", "); // delimeter
        }
      }
      param.append(")");
    } else {

      String name = getRoleNameForValue(value);

      if (name == null) { // if the value doesn't exist or no associated name
        if(consultantTypes.contains(value.getType())) {//we want to declare a local value
          List<Term> properties = getProperties(value);
          if(getLocalVariableForProperties(properties)!=null){
            name = getLocalVariableForProperties(properties);
          } else {
            declareLocalVariable(value);
            name = getRoleNameForValue(value);
          }
        } else {
          name = value.toString();
        }
      }
      param.append(name);
    }
    return param.toString();
  }



  /**
   *
   * @param locationConstraint constraint predicate of the form before(X), after(X), and(X,Y)
   * @return index of first step/before or after constraint
   */
  public int getIndexOfConstraint(Predicate locationConstraint) {
    //TODO:brad: this needs to be updated to accommodate before/after as well as and?
    return getIndexGivenConstraints(locationConstraint, (Predicate) locationConstraint.get(0))+1;
//    String relation = locationConstraint.getName();
//    List<Symbol> refs = locationConstraint.getArgs();
//    int modLocation;
//    if (relation.equals("and")) {
//      // if modification should be combination of refs
//      modLocation = getAndModificationInterval(eventSpecs, locationConstraint).get(1);
//    } else {
//      // if modification should occur before or after ref
//      Predicate ref = (Predicate) refs.get(0);
//      modLocation = getEventStepLocation( ref);
//      if (modLocation >= 0) {
//        if (relation.equals("after")) { // if modification is after ref
//          modLocation++;
//        }
//      }
//    }
//    return modLocation;
  }

  /**
   *
   * @param location
   * @param modification
   * @return index of @modification, constrained by @location
   */
  public int getIndexGivenConstraints(Predicate location, Predicate modification) {
    String relation = location.getName();
    int refLoc = -1;
    if (relation.equals("and")) {
      int firstIndex = getIndexGivenConstraints((Predicate) location.get(0),modification);
      int secondIndex = getIndexGivenConstraints((Predicate) location.get(1),modification);
      if (firstIndex >= 0 && secondIndex >= 0) {
        List<EventSpec> tmp;
        if (firstIndex < secondIndex) {
          tmp = eventSpecs.subList(firstIndex, secondIndex);
          refLoc = getEventStepLocation(tmp, modification);
          if (refLoc >= 0) {
            refLoc = refLoc + firstIndex;
          }
        } else {
          tmp = eventSpecs.subList(secondIndex, firstIndex);
          refLoc = getEventStepLocation(tmp, modification);
          if (refLoc >= 0) {
            refLoc = refLoc + secondIndex;
          }
        }
      }
    } else {
      int splitLoc = getEventStepLocation((Predicate) location.get(0));
      if (splitLoc >= 0) {
        if (relation.equals("before")) {
          refLoc = getEventStepLocation(eventSpecs.subList(0,splitLoc), modification);
        } else {
          refLoc = getEventStepLocation(eventSpecs.subList(splitLoc, eventSpecs.size()), modification);
          if (refLoc >= 0) {
            refLoc = refLoc + splitLoc;
          }
        }
      }
    }
    return refLoc;
  }

  /**
   * Get a unique role name within the context of this learned action.
   * @return
   */
  private String getUniqueRoleName() {
    return "?var_"+roleNameCounter++;
  }

  /**
   * Generates a new ActionDBEntry and adds it to the Database.
   * Iterates through the learning queue and creates events and roles for new action
   *
   */
  public void generateActionDBEntry() {
    ActionDBEntry.Builder newDBEntry = new ActionDBEntry.Builder(name);
    // roles -- learning state creates new generic symbol action binding before returning if non exists
    newDBEntry.addRoles(new ArrayList<>(valueActBindingMap.values()));

    // effects
    newDBEntry.addEffects(effects);

    // conditions
    newDBEntry.addConditions(conditions);

    for(EventSpec declaration: declarationSpecs) {
      newDBEntry.addEventSpec(declaration);
    }

    // event spec
    for (EventSpec eventSpec : eventSpecs) {
      //TODO:brad: what is this for?
      if (Objects.equals(eventSpec.getCommand(), "cancelGoal")){
        // if the current value we're thinking about saving is
        // 'cancelGoal', let's 1) remove the last entry that way meant to
        // be canceled and 2) not save this one.
        newDBEntry.popEventSpec();
        continue;
      }
      newDBEntry.addEventSpec(eventSpec);
    }

    for(EventSpec destruction: destructionSpecs) {
      newDBEntry.addEventSpec(destruction);
    }

    ActionDBEntry x = newDBEntry.build(true);
    ActionScriptLanguageWriter writer= new ActionScriptLanguageWriter();
    log.debug("Built new action:\n" + writer.writeAction(x));
  }


}
