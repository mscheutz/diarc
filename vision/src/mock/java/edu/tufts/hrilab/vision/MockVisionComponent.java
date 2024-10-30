/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.vision;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.util.PragUtil;
import edu.tufts.hrilab.fol.util.Utilities;
import edu.tufts.hrilab.util.resource.Resources;
import edu.tufts.hrilab.vision.consultant.MockVisionConsultant;
import edu.tufts.hrilab.vision.scene.SceneCollection;
import edu.tufts.hrilab.vision.scene.SceneGenerator;
import edu.tufts.hrilab.vision.consultant.VisionConsultant;
import edu.tufts.hrilab.vision.scene.config.MiscSceneGenerator;
import edu.tufts.hrilab.vision.stm.MemoryObject;

import java.awt.Dimension;
import java.lang.reflect.Constructor;
import java.util.*;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.vision.stm.MemoryObjectUtil;
import edu.tufts.hrilab.vision.stm.NamedDescription;
import edu.tufts.hrilab.vision.util.PredicateHelper;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

public class MockVisionComponent extends DiarcComponent implements MockVisionInterface {

  /**
   * For reading/writing/printing POJO.
   */
  private final Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();
  /**
   * For populating mock vision with detection results for different scenes.
   */
  private SceneCollection sceneCollection;
  private String sceneCollectionFilename;
  /**
   * What searches to start on component start-up.
   */
  private List<String> runMemObjTypes;
  /**
   * Index into the sceneCollection specifying current scene;
   */
  private int sceneIndex;

  private Random random;

  private VisionConsultant visionConsultant;

  //fake fields
  private long nextTypeId;
  private Map<Long, List<Term>> typeIdDescriptors;
  private List<Term> knownDescriptorProperties;
  private Map<String, List<NamedDescription>> definitions;

  // if mock vision makes the requested observations
  private boolean makeObservations;

  private boolean registerForBeliefNotifications;

  private String refsConfigFile;
  private String refsConfigDir = "config/edu/tufts/hrilab/vision";

  private SceneGenerator selectedScene;

  // ********************************************************************
  // *** Local methods
  // ********************************************************************

  /**
   * MockVisionComponent constructor.
   */
  public MockVisionComponent() {
    super();

    sceneIndex = 0;
    random = new Random(System.currentTimeMillis());
    nextTypeId = 0;
    typeIdDescriptors = new HashMap<>();
    definitions = new HashMap();
    runMemObjTypes = new ArrayList<>();
    makeObservations = true;
    knownDescriptorProperties = new ArrayList<>();
    registerForBeliefNotifications = false;

    shouldRunExecutionLoop = true;
  }

  @Override
  protected void init() {

    // TODO: put this reading from file back in!
//    // instantiate scene collection
//    if (sceneCollectionFilename == null || sceneCollectionFilename.isEmpty()) {
//      log.error("This component must be started with a scene collection filename.");
//      System.exit(1);
//    }
//    sceneCollection = SceneGenerator.readSceneCollection(sceneCollectionFilename);

    sceneCollection = selectedScene.generateSceneCollection();
    visionConsultant = new MockVisionConsultant(this,"physobj", sceneCollection);

    try {
      Collection<String> consultantGroups=this.getMyGroups();
      consultantGroups.add(visionConsultant.getKBName());
      TRADE.registerAllServices(visionConsultant,consultantGroups);
    } catch (TRADEException e) {
      log.error("[init] exception registering visionConsultant",e);
    }

    // load initial list of known properties
    knownDescriptorProperties.addAll(visionConsultant.getPropertiesHandled());

    // start requested searches
    List<Predicate> descriptors = new ArrayList<>();
    for (String descriptorStr : runMemObjTypes) {
      Predicate descriptor = Factory.createPredicate(descriptorStr);
      descriptors.clear();
      descriptors.add(descriptor);
      getTypeId(descriptors);
    }
    if (refsConfigFile != null && !refsConfigFile.isEmpty()) {
      String filename = Resources.createFilepath(refsConfigDir, refsConfigFile);
      visionConsultant.loadReferencesFromFile(filename);
    }
  }

  @Override
  public void shutdownComponent(){
    if (visionConsultant != null) {
      try {
        TRADE.deregister(visionConsultant);
      } catch (TRADEException e) {
        log.error("[shutdownComponent] ", e);
      }
    }
  }

  @Override
  protected void executionLoop() {
    // TODO: this should be replaces with learn action calls instead of belief notifications
    if (registerForBeliefNotifications) {
      Term instanceOfTerm = Factory.createPredicate("instanceOf(X,Y)");
      Term definitionOfTerm = Factory.createPredicate("definitionOf(X,Y)");
      try {
        TRADEServiceInfo learnService = getMyService("learn", Term.class, List.class);
        TRADEServiceInfo registerService = TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForNotification").argTypes(Term.class,TRADEServiceInfo.class));
        registerService.call(void.class,"registerForNotification", instanceOfTerm, learnService);
        registerService.call(void.class,"registerForNotification", definitionOfTerm, learnService);
        registerForBeliefNotifications = false; //don't want to keep on registering
      } catch (TRADEException e) {
        log.debug("Could not register for belief notifications. Will try again.", e);
        registerForBeliefNotifications = true;
      }
    }
  }

  /**
   * Provide additional information for usage...
   */
  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("belief").desc("register for notifications from edu.tufts.hrilab.belief.BeliefComponent").build());
    options.add(Option.builder("makeObservations").hasArg().argName("true/false").desc("set if observations should be successfully observed or not (true by default)").build());
    options.add(Option.builder("runType").hasArgs().argName("types").desc("turn on detection/tracking of specified memory object type(s)").build());
    options.add(Option.builder("refs").longOpt("references").hasArg().argName("file").desc("load pre-defined object references and their properties").build());
    options.add(Option.builder("sceneClass").hasArgs().argName("scene class fully qualified name").desc("specify a SceneGenerator class to be used for this mock vision component").build());
//    options.add(Option.builder("sceneCollection").hasArg().argName("file").desc("load config file with fake scene information").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("belief")) {
      registerForBeliefNotifications = true;
    }
    if (cmdLine.hasOption("makeObservations")) {
      makeObservations = Boolean.valueOf(cmdLine.getOptionValue("makeObservations"));
    }

    if (cmdLine.hasOption("sceneClass")) {
      try {
        Class sceneClass = Class.forName(cmdLine.getOptionValue("sceneClass"));
        Constructor sceneClassConstructor = sceneClass.getConstructor();
        selectedScene = (SceneGenerator) sceneClassConstructor.newInstance();
      } catch (Exception e) {
        log.error("Exception in using scene generator class: " , e);
      }
    } else {
      selectedScene = new MiscSceneGenerator();
    }

    if (cmdLine.hasOption("runType")) {
      runMemObjTypes.addAll(Arrays.asList(cmdLine.getOptionValues("runType")));
      log.info("added runtypes: " + runMemObjTypes);
//      } else if (cmdLine.hasValue("sceneCollection")) {
//        sceneCollectionFilename = cmdLine.getOptionValue("sceneCollection");
    }
    if (cmdLine.hasOption("refs")) {
      refsConfigFile = cmdLine.getOptionValue("refs");
    }
  }

  @Override
  public void pauseCapture() {
    log.info("[pauseCapture]");
  }

  @Override
  public void resumeCapture() {
    log.info("[resumeCapture]");
  }

  @Override
  public void stopAllSearches() {
    log.info("[stopAllSearches]");
  }

  @Override
  public void restartAllStoppedSearches() {
    log.info("[restartAllStoppedSearches]");
  }

  @Override
  public List<Long> getAvailableTypeIds() {
    return new ArrayList<>(typeIdDescriptors.keySet());
  }

  @Override
  public void startType(long typeId) {
  }

  @Override
  public void stopType(long typeId) {
    log.info("[stopType] typeId: " + typeId);
  }

  @Override
  public void stopAndRemoveType(long typeId) {
    typeIdDescriptors.remove(typeId);
    return;
  }

  @Override
  public List<Long> getTypeIds() {
    return new ArrayList<>(typeIdDescriptors.keySet());
  }

  @Override
  public Long getTypeId(List<? extends Symbol> descriptors) {
    log.info("[getTypeId] descriptors: " + descriptors);

    List<Term> searchDescriptors = (List<Term>) descriptors;

    long returnTypeId = -1L;
    if (areKnownConstraints(searchDescriptors)) {

      // check if search already exists
      for (Long typeId : typeIdDescriptors.keySet()) {
        List<Term> typeDescriptors = typeIdDescriptors.get(typeId);
        if (Utilities.predicatesMatch(typeDescriptors, searchDescriptors)) {
          returnTypeId = typeId;
          break;
        }
      }

      // if no existing search was found, create new one
      if (returnTypeId == -1L) {
        returnTypeId = nextTypeId++;
        typeIdDescriptors.put(returnTypeId, new ArrayList<>(searchDescriptors));
      }
    } else {
      log.info("[getTypeId] unknown descriptors: " + getUnsatisfiableConstraints(descriptors));
    }
    log.info("[getTypeId] typeId: " + returnTypeId);
    return returnTypeId;
  }

  @Override
  public Long getTypeId(Symbol objectRef) {
    log.debug("[getTypeId] objectRef: " + objectRef);
    return visionConsultant.getTypeId(objectRef);
  }

  @Override
  public boolean nameDescriptors(List<? extends Symbol> descriptors, Symbol type) {
    log.debug("[nameDescriptors]: " + type);
    for (Term t : PredicateHelper.convertToVisionForm(type)) {
      if (t.isPredicate()) {
        knownDescriptorProperties.add(t);
      } else {
        knownDescriptorProperties.add(t);
      }


      // add to definitions
      String typeStr = PredicateHelper.getRepresentativeString(t);
      List<NamedDescription> descriptions = definitions.get(typeStr);
      if (descriptions == null) {
        descriptions = new ArrayList();
        definitions.put(typeStr, descriptions);
      }

      descriptions.add(new NamedDescription(t, PredicateHelper.convertToVisionForm(descriptors)));
    }

    return true;
  }

  @Override
  public boolean nameDescriptors(Long typeId, Symbol type) {
    log.debug("[nameDescriptors]: " + type);
    nameDescriptors(typeIdDescriptors.get(typeId), type);
    return true;
  }

  @Override
  public List<Term> getDescriptors(Long typeId) {
    log.debug("[getDescriptors] typeId: " + typeId);
    return typeIdDescriptors.get(typeId);
  }

  @Override
  public List<Long> getTokenIds() {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public List<Long> getTokenIds(long typeId) {
    log.debug("[getTokenIds] typeId: " + typeId);
    List<MemoryObject> tokens = getTokens(typeId);

    List<Long> results = new ArrayList<>();
    for (MemoryObject token : tokens) {
      results.add(token.getTokenId());
    }
    return results;
  }

  @Override
  public List<Long> getTokenIds(List<? extends Symbol> descriptors) {
    Long typeId = getTypeId(descriptors);
    return getTokenIds(typeId);
  }

  @Override
  public List<MemoryObject> getTokens() {
    List<MemoryObject> results = new ArrayList<>();
    if (sceneCollection != null) {
      float noise = sceneCollection.getNoise();
      results = sceneCollection.getSceneDetectionResults();
      if (random.nextFloat() <= noise && noise != 0.0f) {
        log.error("Applying noise to MemoryObject is currently not supported.");
        // TODO: apply noise to all descriptors, accounting for single/double arity predicates, and handling different variable names
//        List<Term> types = sceneCollection.getProperties();
//        for (MemoryObject mo : detectionResults) {
        // select wrong type -- do this by using the correctIndex as a stand in for the last index
//          int wrongIndex = random.nextInt(types.size() - 2); // (-2) to help ignore correct type
//          int correctIndex = types.indexOf(mo.getDescriptors().get(0));
//          wrongIndex = (wrongIndex == correctIndex) ? (types.size() - 1) : wrongIndex;
//          mo.addDescriptor(types.get(wrongIndex), (1.0f - noise)); // TODO: what should confidence be?
//        }
      }
    }
    return results;
  }

  @Override
  public List<MemoryObject> getTokens(long typeId) {
    List<MemoryObject> tokensOut = new ArrayList<>();

    if (typeIdDescriptors.containsKey(typeId)) {
      if (typeIdDescriptors.get(typeId).get(0).getName().equals("any")) {
        return sceneCollection.getSceneDetectionResults();
      }

      // get current scene
      List<MemoryObject> sceneObjects = sceneCollection.getSceneDetectionResults();

      // iterate through scene objects and filter for those matching typeId's descriptors
      for (MemoryObject mo : sceneObjects) {
        Set<Term> moDescriptors = MemoryObjectUtil.getSceneGraphDescriptors(mo);
        if (Utilities.containsAllPredicates(new ArrayList<>(moDescriptors), typeIdDescriptors.get(typeId))) {
          mo.setTypeId(typeId);
          tokensOut.add(mo);
        }
      }

    } else {
      log.error("TypeId does not exist: " + typeId);
    }

    return tokensOut;
  }

  @Override
  public List<MemoryObject> getTokens(List<? extends Symbol> descriptors) {
    Long typeId = getTypeId(descriptors);
    return getTokens(typeId);
  }


  @Override
  public MemoryObject getToken(long tokenId) {
    // NOTE: This only checks the top-level scene graphs for matching tokenIds
    for (MemoryObject mo : sceneCollection.getSceneDetectionResults()) {
      if (mo.getTokenId() == tokenId) {
        return mo;

      }
    }

    return null;
  }

  @Override
  public Long createNewType() {
    Long id = nextTypeId++;
    typeIdDescriptors.put(id, new ArrayList<>());
    return id;
  }

  @Override
  public boolean addDescriptor(long typeId, Symbol descriptor) {
    List<Term> visionDescriptors = PredicateHelper.convertToVisionForm(descriptor);
    if (typeIdDescriptors.containsKey(typeId)) {
      typeIdDescriptors.get(typeId).addAll(visionDescriptors);
      return true;
    } else {
      return false;
    }
  }

  @Override
  public boolean removeDescriptor(long typeId, Symbol descriptor) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public void endDescriptorChanges(long typeId) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public boolean confirmToken(long tokenId) {
//    throw new UnsupportedOperationException("Not supported yet.");
//Hack!
    return true;
  }

  @Override
  public boolean confirmToken(MemoryObject token) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public byte[] getFrame() {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public byte[] getDisparityFrame() {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public byte[] getDepthFrame() {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public void takeSnapshot(String filename) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public Dimension getImageSize() {
    throw new UnsupportedOperationException("Not supported yet.");
  }
  // ============== END Visual Search Methods ====================================

  @Override
  public List<Symbol> getUnsatisfiableConstraints(List<? extends Symbol> descriptors) {
    List<Symbol> unsatisfiableConstraints = new ArrayList<>();
    for (Term d : PredicateHelper.convertToVisionForm(descriptors)) {
      if (!termListContains(knownDescriptorProperties, d)) {
        unsatisfiableConstraints.add(d);
      }
    }
    log.debug("[getUnsatisfiableConstraints] of " + descriptors + " are: " + unsatisfiableConstraints);
    return unsatisfiableConstraints;
  }

  private boolean termListContains(List<Term> properties, Term term) {
    for (Term p : properties) {
      if (Utilities.predicatesMatch(p, term)) return true;
    }
    return false;
  }

  @Override
  public List<Symbol> getUnsatisfiableConstraints(Symbol descriptor) {
    List<Symbol> descriptorList = new ArrayList<>();
    descriptorList.add(descriptor);
    return getUnsatisfiableConstraints(descriptorList);
  }

  @Override
  public boolean learn(Term learnTerm, List<Map<Variable, Symbol>> bindings) {
    log.debug("[learn] called. learnTerm: " + learnTerm + " bindings: " + bindings);

    if (learnTerm.size() != 2) {
      log.error("[learn] expects a Term with two arguments: " + learnTerm);
      return false;
    }

    boolean somethingLearned = false;

    // iterate through sets of bindings
    Term nameToLearn;
    List<Symbol> descriptorsToLearn;
    for (Map<Variable, Symbol> binding : bindings) {

      // separate the name of the concept being learned and the definition of that concept
      Term boundLearningTerm = PragUtil.getBoundTerm(binding, learnTerm);

      List<Symbol> arg0 = new ArrayList<>();
      if (PredicateHelper.isObjectRef(boundLearningTerm.get(0))) {
        arg0.add(boundLearningTerm.get(0));
      } else {
        arg0.addAll(PredicateHelper.convertToVisionForm(boundLearningTerm.get(0)));
      }
      List<Term> arg1 = PredicateHelper.convertToVisionForm(boundLearningTerm.get(1));
      log.debug("arg0: " + arg0 + " arg1: " + arg1);
      if (arg1.size() == 1) {
        nameToLearn = arg1.get(0);
        descriptorsToLearn = arg0;
      } else {
        log.error("[learn] couldn't learn: " + boundLearningTerm);
        continue;
      }

      // learn something based on learning mode (i.e., definitionOf, instanceOf)
      boolean didLearn = false;
      Term nameToLearnProperty = new Predicate(nameToLearn);
      if (learnTerm.getName().equals("definitionOf")) {
        didLearn = nameDescriptors(descriptorsToLearn, nameToLearn);
      } else if (learnTerm.getName().equals("instanceOf")) {
        knownDescriptorProperties.add(nameToLearnProperty);
        didLearn = true;
      } else {
        log.error("[learn] learning term not supported: " + learnTerm);
        return false;
      }

      // check if thing to learn is a POWER ref (e.g., "instanceOf(objects_3, knife)") and add property to it
      if (PredicateHelper.isObjectRef(descriptorsToLearn)) {
        Symbol objectRef = Factory.createSymbol(descriptorsToLearn.get(0).getName());

        // replace variable in nameToLearn to match objectRef's variable, e.g., knife(VAR5) --> knife(Y)
        Predicate nameToLearnMatchingVar = PredicateHelper.replace(nameToLearn, nameToLearn.get(0), visionConsultant.getVariable(objectRef));
        List<Term> properties = new ArrayList<>();
        properties.add(nameToLearnMatchingVar);
        visionConsultant.assertProperties(objectRef, properties);
      }

      if (didLearn) {
        somethingLearned = true;

        // update POWER with new concept
//        try {
//          log.debug("adding new property: " + nameToLearn);
//          TRADE.callThe("addNewProperty", visionConsultant.getKBName(), new PredicateProperty(new Predicate(nameToLearn)));
//        } catch (TRADEException e) {
//          log.error("Could not update POWER with new concept.", e);
//        }
      }
    }

    log.debug("[learn] Object Ref Summary:\n" + visionConsultant.getReferenceSummaries());
    return somethingLearned;
  }

  @Override
  public boolean unlearn(Term learnTerm, List<Map<Variable, Symbol>> bindings) {
    log.debug("[unlearn] called. learnTerm: " + learnTerm + " bindings: " + bindings);

    if (learnTerm.size() != 2) {
      log.error("[unlearn] expects a Term with two arguments: " + learnTerm);
      return false;
    }

    boolean somethingUnlearned = false;

    // iterate through sets of bindings
    Term nameToUnlearn;
    List<Term> descriptorsToUnlearn;
    for (Map<Variable, Symbol> binding : bindings) {

      // separate the name of the concept being learned and the definition of that concept
      Term boundLearningTerm = PragUtil.getBoundTerm(binding, learnTerm);

      List<Term> arg0 = new ArrayList<>();
      if (PredicateHelper.isObjectRef(boundLearningTerm.get(0))) {
        // HACK: adding variable here so arg0 doesn't have to be of type List<Symbol> which causes other issues
        // later in this method
        arg0.add(new Term(boundLearningTerm.get(0).getName(), new Variable("HACK")));
      } else {
        arg0.addAll(PredicateHelper.convertToVisionForm(boundLearningTerm.get(0)));
      }
      List<Term> arg1 = PredicateHelper.convertToVisionForm(boundLearningTerm.get(1));
      if (arg1.size() == 1) {
        nameToUnlearn = arg1.get(0);
        descriptorsToUnlearn = arg0;
      } else {
        log.error("[unlearn] couldn't unlearn: " + boundLearningTerm);
        continue;
      }

      // learn something based on learning mode (i.e., definitionOf, instanceOf)
      boolean didUnlearn = false;
      if (learnTerm.getName().equals("definitionOf")) {

        // a definition for something expressed in terms of other vision capabilities e.g., definitionOf(partOf(orange, knife), handle)
        // find existing definition to remove
        List<NamedDescription> descriptions = definitions.get(PredicateHelper.getRepresentativeString(nameToUnlearn));
        for (NamedDescription description : descriptions) {
          if (Utilities.predicatesMatch(description.getDescriptors(), descriptorsToUnlearn)) {
            descriptions.remove(description);
            if (descriptions.isEmpty()) {
              definitions.remove(PredicateHelper.getRepresentativeString(nameToUnlearn));
            }
            break;
          }
        }

      } else if (learnTerm.getName().equals("instanceOf")) {
        log.warn("[unlearn] instanceOf not implemented yet.");
      } else {
        log.error("[unlearn] learning term not supported: " + learnTerm);
        return false;
      }

      // check if thing to unlearn is a POWER ref (e.g., "definitionOf(objects_3, knife)")
      if (PredicateHelper.isObjectRef(descriptorsToUnlearn)) {
        Symbol objectRef = Factory.createSymbol(descriptorsToUnlearn.get(0).getName());

        // remove property from existing objectRef
        List<Term> properties = new ArrayList<>();
        properties.add(nameToUnlearn);
        visionConsultant.retractProperties(objectRef, properties);

      }

      if (didUnlearn) {
        somethingUnlearned = true;

        //remove from knownProperties ?
//        knownDescriptorProperties.remove()
      }
    }

    log.debug("[unlearn] Object Ref Summary:\n" + visionConsultant.getReferenceSummaries());
    return somethingUnlearned;
  }

  @Override
  public List<Map<Variable, Symbol>> makeObservations(Term observation) {
    log.info("[makeObservations] called with: " + observation);
    if (observation.getVars().size() > 0) {
      log.error("Vision observations can't handle free variables.");
      return new ArrayList<>();
    }

    // TODO: generalize this to match real vision component
//    if (observation.get(0).getName().equals("self")) {
//      return null;
//    }

    List<Map<Variable, Symbol>> bindings = new ArrayList<>();
    if (makeObservations) {
      bindings.add(new HashMap<>()); // adding this means the observation has been successfully made
    }
    return bindings;
  }


  private boolean areKnownConstraints(List<Term> constraints) {
    for (Term c : constraints) {
      boolean found = false;
      for (Term known : knownDescriptorProperties) {
        if (Utilities.predicatesMatch(c, known)) {
          found = true;
          break;
        }
      }

      if (!found) {
        return false;
      }
    }

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// methods specific to mock vision /////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public boolean setSceneIndex(int index) {
    sceneIndex = index;
    return sceneCollection.setSceneIndex(index);
  }

  @Override
  public void setObsevations(boolean makeObservations) {
    this.makeObservations = makeObservations;
  }

}
