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
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.asl.AslBlockBuilder;
import edu.tufts.hrilab.action.asl.AslControlSpecFactory;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.util.Utilities;
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
  private AslBlockBuilder aslBuilder;

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
  private Set<String> consultantTypes = Set.of(new String[]{"physobj", "cognexobj"});//, "pose"});//,"gofapose","yumipose","mobileyumipose","location"});
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
    refreshBuilder();
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
        valueActBindingMap.put(Factory.createSymbol(role.name), role);
      }
    }

    conditions.addAll(adbe.getPreConditions());
    conditions.addAll(adbe.getOverallConditions());
    effects.addAll(adbe.getEffects().stream().filter(e -> !e.isAutoGenerated()).collect(Collectors.toList()));
    this.description=adbe.getDescription();
    refreshBuilder();
    // need to make sure that the cost, timeout, posAff, negAff .... are all added properly
    // should the learning state take that into account?
  }

  private void refreshBuilder() {
    List<String> predeclaredLocals = valueActBindingMap.values().stream()
        .filter(ActionBinding::isLocal)
        .map(ActionBinding::getName)
        .toList();
    Map<String, Class<?>> predeclaredTypes = new HashMap<>();
    valueActBindingMap.values().forEach(binding -> predeclaredTypes.put(binding.getName(), binding.getJavaType()));
    aslBuilder = new AslBlockBuilder(eventSpecs, predeclaredLocals, predeclaredTypes);
    eventSpecs = new ArrayList<>(aslBuilder.specs());
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

  public List<Symbol> getRoleNames() {
    return new ArrayList<>(inputRoles);
  }

  public List<EventSpec> getEventSpecs() {
    return new ArrayList<>(aslBuilder.specs());
  }

  /**
   * Add EventSpec described by {@code p} at {@code index}
   * @param index index in learning state at which to insert new EventSpec
   * @param p Predicate description of new Event spec
   */
  public String addEventSpec(int index, Predicate p){
    //TODO:brad is ist safe to get the actor like this?
    EventSpec spec = buildEventSpecForAction(p.get(0),p);
    if (!aslBuilder.getOpenBlocks().isEmpty()) {
      int tail = aslBuilder.specs().size();
      if (index != tail) {
        throw new IllegalArgumentException("Can only append taught steps while a structured block is open.");
      }
      aslBuilder.add(spec);
    } else {
      aslBuilder.insert(index, spec);
    }
    eventSpecs = new ArrayList<>(aslBuilder.specs());

    List<String> newInputArgs= spec.getInputArgs();
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
    EventSpec removed= getEventSpecs().get(index);
    aslBuilder.removeActionAt(index);
    eventSpecs = new ArrayList<>(aslBuilder.specs());

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
            refreshBuilder();
          }
        }
      }
    }
  }

  public void removeEventSpecsForLocalVariable(Symbol lvRef){
    List<Term> properties = getProperties(lvRef);

    String lvToRemove= getLocalVariableForProperties(properties);

    eventSpecs= eventSpecs.stream().filter( e -> !e.getInputArgs().contains(lvToRemove)).collect(Collectors.toList());
    refreshBuilder();

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
    String groundedReference = unquoted(groundArgs(reference));
    for (EventSpec step : steps) {
      if (unquoted(groundArgs(step.getPredicateForm())).equals(groundedReference)) {
        return currentLoc;
      }
      currentLoc++;
    }
    return -1;
  }

  // the stored steps carry quotes around names with spaces in them (processEventArgument), the
  // modifications asking for those steps do not, so compare without them
  private String unquoted(Predicate step) {
    return step.toString().replace("\"", "");
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
    Predicate goalPred = g.getGoalStatePredicate();
    Symbol actor = g.getActor();

    // TODO:  handle handle appropriate types of events
    if (goalPred.getName().equals("if") || goalPred.getName().equals("else")) {
      buildControlSpec(goalPred);
    } else {
      aslBuilder.add(buildEventSpecForAction(actor,goalPred));
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }
  }

  public void beginLearnedIf(Predicate guard) {
      aslBuilder.beginIf(toConditionSpecs(groundArgs(guard)));
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void beginLearnedElseIf(Predicate guard) {
      aslBuilder.beginElseIf(toConditionSpecs(groundArgs(guard)));
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void beginLearnedElse() {
      aslBuilder.beginElse();
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void endLearnedIf() {
      aslBuilder.endIf();
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void beginLearnedWhile(Predicate guard) {
      aslBuilder.beginWhile(toConditionSpecs(groundArgs(guard)));
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void endLearnedWhile() {
      aslBuilder.endWhile();
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void beginLearnedForeach(String iteratorVariable, String collectionVariable) {
      aslBuilder.beginForeach(iteratorVariable, collectionVariable);
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void endLearnedForeach() {
      aslBuilder.endForeach();
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void beginLearnedFor(String loopVariable, String initialValue, String comparisonOperator, String comparisonValue, String update) {
      aslBuilder.beginFor(loopVariable, initialValue, comparisonOperator, comparisonValue, update);
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void endLearnedFor() {
      aslBuilder.endFor();
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void declareLearnedLocal(String variableName, String javaType, String semanticType, String initializerExpression) {
      Class<?> varClass = Utilities.getClass(javaType);
      if (varClass == null) {
        throw new IllegalArgumentException("Unknown java type for local declaration: " + javaType);
      }
      ActionBinding binding = new ActionBinding.Builder(variableName, varClass)
          .setSemanticType(semanticType == null ? "" : semanticType)
          .setIsLocal(true)
          .build();
      valueActBindingMap.put(Factory.createSymbol(variableName), binding);
      int insertIndex = aslBuilder.specs().size();
      aslBuilder.addBlockLocalDeclaration(insertIndex, variableName, javaType, semanticType, initializerExpression);
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public boolean hasOpenBlocks() {
      return !aslBuilder.getOpenBlocks().isEmpty();
    }

    public void appendActionToBlockBody(int blockStartIndex, Predicate actionPredicate) {
      aslBuilder.appendToBlockBody(blockStartIndex, buildEventSpecForAction(actionPredicate.get(0), actionPredicate));
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void appendActionToBranch(int ifBlockStartIndex, int branchIndex, Predicate actionPredicate) {
      aslBuilder.appendToBranch(ifBlockStartIndex, branchIndex, buildEventSpecForAction(actionPredicate.get(0), actionPredicate));
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void removeCompleteBlock(int blockStartIndex) {
      aslBuilder.removeCompleteBlock(blockStartIndex);
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void replaceCompleteBlockWithAction(int blockStartIndex, Predicate actionPredicate) {
      aslBuilder.replaceCompleteBlock(blockStartIndex,
          List.of(buildEventSpecForAction(actionPredicate.get(0), actionPredicate)));
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void addBlockLocalDeclaration(int insertIndex, String variableName, String javaType, String semanticType, String initializerExpression) {
      Class<?> varClass = Utilities.getClass(javaType);
      if (varClass == null) {
        throw new IllegalArgumentException("Unknown java type for local declaration: " + javaType);
      }
      ActionBinding binding = new ActionBinding.Builder(variableName, varClass)
          .setSemanticType(semanticType == null ? "" : semanticType)
          .setIsLocal(true)
          .build();
      valueActBindingMap.put(Factory.createSymbol(variableName), binding);
      aslBuilder.addBlockLocalDeclaration(insertIndex, variableName, javaType, semanticType, initializerExpression);
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void renameBlockLocalDeclaration(int declarationIndex, String newName) {
      if (declarationIndex < 0 || declarationIndex >= eventSpecs.size()) {
        throw new IllegalArgumentException("Declaration index out of bounds: " + declarationIndex);
      }
      EventSpec declaration = eventSpecs.get(declarationIndex);
      if (!"declare".equals(declaration.getCommand()) || declaration.getInputArgs().isEmpty()) {
        throw new IllegalArgumentException("No declaration at index " + declarationIndex);
      }
      String oldName = declaration.getInputArgs().get(0);
      ActionBinding toRename = valueActBindingMap.values().stream()
          .filter(ActionBinding::isLocal)
          .filter(binding -> binding.getName().equals(oldName))
          .findFirst()
          .orElse(null);
      aslBuilder.renameBlockLocalVariable(declarationIndex, newName);
      eventSpecs = new ArrayList<>(aslBuilder.specs());
      if (toRename != null) {
        valueActBindingMap.values().removeIf(b -> b.isLocal() && b.getName().equals(oldName));
        valueActBindingMap.put(Factory.createSymbol(newName),
            new ActionBinding.Builder(newName, toRename.getJavaType()).setSemanticType(toRename.getSemanticType()).setIsLocal(true).build());
      }
    }

    public void removeActionAt(int index) {
      aslBuilder.removeActionAt(index);
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    public void replaceActionAt(int index, Predicate actionPredicate) {
      aslBuilder.replaceActionAt(index, buildEventSpecForAction(actionPredicate.get(0), actionPredicate));
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    }

    private List<EventSpec> toConditionSpecs(Predicate condition) {
      String name = condition.getName();
      if ("and".equals(name) || "or".equals(name)) {
        List<EventSpec> specs = new ArrayList<>();
        specs.add(AslControlSpecFactory.control("and".equals(name)
            ? AslControlSpecFactory.Marker.AND
            : AslControlSpecFactory.Marker.OR));
        for (Symbol arg : condition.getArgs()) {
          if (!arg.isPredicate()) {
            throw new IllegalArgumentException("Invalid condition argument for " + name + ": " + arg);
          }
          specs.addAll(toConditionSpecs((Predicate) arg));
        }
        specs.add(AslControlSpecFactory.control("and".equals(name)
            ? AslControlSpecFactory.Marker.ENDAND
            : AslControlSpecFactory.Marker.ENDOR));
        return specs;
      }

      if ("not".equals(name)) {
        if (condition.getArgs().size() != 1 || !condition.get(0).isPredicate()) {
          throw new IllegalArgumentException("Invalid not condition: " + condition);
        }
        List<EventSpec> specs = new ArrayList<>();
        specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.NOT));
        specs.addAll(toConditionSpecs((Predicate) condition.get(0)));
        specs.add(AslControlSpecFactory.control(AslControlSpecFactory.Marker.ENDNOT));
        return specs;
      }

      if ("true".equalsIgnoreCase(name) && condition.getArgs().isEmpty()) {
        return List.of(AslControlSpecFactory.control(AslControlSpecFactory.Marker.TRUE));
      }

      if ("false".equalsIgnoreCase(name) && condition.getArgs().isEmpty()) {
        return List.of(AslControlSpecFactory.control(AslControlSpecFactory.Marker.FALSE));
      }

      if (condition.getArgs().isEmpty()) {
        return List.of(new EventSpec.Builder(EventSpec.EventType.CONTROL)
            .setCommand("evaluate")
            .addInputArg(condition.toString())
            .build());
      }
      return List.of(buildEventSpecForAction(condition.get(0), condition));
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
      aslBuilder.beginIf(toConditionSpecs(conditionPred));
      //get the condition
      //TODO:brad: is it safe to assume that the actor is arg 0 here?
      //todo: only handling a single actspec in the body. actor assumption is also an issue here.
      //TODO:brad: same actor assumption here
      aslBuilder.add(buildEventSpecForAction(bodyPred.get(0),bodyPred));
      aslBuilder.endIf();
      eventSpecs = new ArrayList<>(aslBuilder.specs());
    } else if (goalPred.getName().equals("else")) {
      throw new IllegalArgumentException("Legacy compact else teaching form is ambiguous; use explicit begin/end conditional teaching APIs.");
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
          name = value.getName();
          //TODO: Need to tell at this point whether the argument is a string or some other type in order to properly
          //      escape it if necessary. Otherwise ASL writer/parser can break or have incorrect types supplied
          if (needsEscaping(name)) {
            name = "\"" + name + "\"";
          }
          name = value.hasType() ? name + ":" + value.getType() : name;
        }
      }
      param.append(name);
    }
    return param.toString();
  }

  private boolean needsEscaping(String s) {
    //Already escaped
    if (s.startsWith("\"") && s.endsWith("\"")) {
      return false;
    }

    //Word or digit without special characters
    if (s.matches("\\w+") || s.matches("[\\x00-\\x20]*[-]?((\\p{Digit}+)(\\.)?((\\p{Digit}+)?))[\\x00-\\x20]*")) {
      return false;
    }

    //Reference symbol
    if (s.matches(".*(.+)_\\d+:\\1")) {
      return false;
    }

    //Otherwise contains a special character and is not detected to be a reference, escape
    return true;
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

  public ActionDBEntry generateActionDBEntry() {
    return generateActionDBEntry(true);
  }

  /**
   * Generates a new ActionDBEntry from this learning state.
   *
   * @param addToDatabase whether to register the generated action in the action database
   */
  public ActionDBEntry generateActionDBEntry(boolean addToDatabase) {
    eventSpecs = new ArrayList<>(aslBuilder.build());
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

    ActionDBEntry x = newDBEntry.build(addToDatabase);
    ActionScriptLanguageWriter writer= new ActionScriptLanguageWriter();
    log.debug("Built new action:\n" + writer.writeAction(x));
    return x;
  }


}
