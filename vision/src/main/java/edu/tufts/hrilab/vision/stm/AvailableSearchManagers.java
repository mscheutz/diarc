/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.stm;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.util.Utilities;
import edu.tufts.hrilab.vision.util.NonEDTListModel;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.detector.swig.NativeDetector;
import edu.tufts.hrilab.vision.util.PredicateHelper;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashSet;
import java.util.Set;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.stream.Collectors;
import javax.swing.AbstractListModel;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.tufts.hrilab.util.xml.Xml;

/**
 * Factory class to manage all SearchManagers (i.e., face, teabox, cube) in the
 * system. Contains the information needed to instantiate each SearchManager and
 * also contains a reference to each instantiated SearchManager that has been
 * handed out to clients. Thread-safe, although thread-safety is naively
 * implemented with synchronized methods.
 *
 * @author Evan Krause
 */
public final class AvailableSearchManagers {

  //contains all SearchManagers, hashed by unique ID
  private Map<Long, SearchManager> types_byId = new HashMap();
  private Map<SearchManager, HashSet<Object>> instantiatedSearchManagerClients = new HashMap();
  //contains descriptions to instantiate searches defined in xml config file
  private Map<String, List<NamedDescription>> typeDescriptions_byName = new HashMap();
  // searches stopped via the stopAllSearches method
  List<SearchManager> stoppedSearches = new ArrayList();
  //for GUI
  private NonEDTListModel<SearchManager> orderedTypes = new NonEDTListModel<>();
  private NonEDTListModel<SearchManager> orderedSimpleSearchTypes = new NonEDTListModel<>();
  //flags to control vision searches
  private boolean incrementalDetector = false;
  private boolean incrementalImgProc = false;
  private boolean serialProcessing = false;
  private boolean singleIteration = false; //for both detectors and imgprocs
  private static Logger log = LoggerFactory.getLogger(AvailableSearchManagers.class);

  public AvailableSearchManagers(final String configFile) {
    //parse configuration file
    if (configFile != null && !configFile.isEmpty()) {
      Predicate name;
      NativeDetector.DetectorType type;
      String filename;
      String[] dependencies;
      String[] trackers;

      try {
        //get directory    
        File file = new File(configFile);
        String dir = file.getParent() + "/";
        log.debug(String.format("Load SearchManagers config directory: %s.", dir));

        //get main node
        Xml memoryObjectTypes = new Xml(configFile, "searchTypes");

        //get individual flags
        Xml flags = memoryObjectTypes.child("flags");
        incrementalImgProc = flags.boolValue("incrementalImgProc");
        incrementalDetector = flags.boolValue("incrementalDetector");
        serialProcessing = flags.boolValue("serialProcessing");
        singleIteration = flags.boolValue("singleIteration");

        //get pre-set SearchTypes
        for (Xml moType : memoryObjectTypes.children("searchType")) {
          //parse "advertisement" predicate
          String searchTypeDescriptor = moType.string("descriptor");
          Term searchType = Factory.createPredicate(searchTypeDescriptor + "(X)");

          //parse predicates for description
          List<Term> description = new ArrayList();
          for (Xml predicate : moType.children("predicate")) {
            description.add(Factory.createPredicate(predicate.content().trim()));
          }

          //add descriptions to hashmap
          String predicateStr = PredicateHelper.getRepresentativeString(searchType);
          List<NamedDescription> descriptions = typeDescriptions_byName.get(predicateStr);
          if (descriptions == null) {
            descriptions = new ArrayList();
            typeDescriptions_byName.put(predicateStr, descriptions);
          }
          descriptions.add(new NamedDescription(searchType, description));
        }

      } catch (FileNotFoundException e) {
        log.error(String.format("Error parsing config file: %s.", configFile), e);
      }
    }
  }

  ////////////////////////////////// START GUI methods /////////////////////////////////////////////////////

  /**
   * Only to be used by GUI to get all instantiated SearchManagers.
   *
   * @return
   */
  public synchronized AbstractListModel getAllSearchManagers() {
    return orderedTypes;
  }

  /**
   * Only to be used by GUI to get all instantiate SingleSearchManagers.
   *
   * @return
   */
  public synchronized AbstractListModel getSimpleSearchManagers() {
    return orderedSimpleSearchTypes;
  }

  /**
   * Used by instantiated SearchManagers to update GUI of any changes to itself.
   *
   * @param entry
   */
  public synchronized void updateGuiEntry(final SearchManager entry) {
    orderedTypes.updateGuiEntry(entry);
    orderedSimpleSearchTypes.updateGuiEntry(entry);
  }

  /**
   * Create a new empty SimpleSearchManager. This should only be used from the
   * GUI for debugging and testing purposes only!
   *
   * @return SearchManager
   */
  public synchronized SearchManager getTestingInstance(final Object client) {
    SearchManager newSearch = new SimpleSearchManager(incrementalImgProc, incrementalDetector, serialProcessing, singleIteration);
    long id = newSearch.getTypeId();
    types_byId.put(id, newSearch);
    orderedTypes.add(newSearch);
    orderedSimpleSearchTypes.add(newSearch);

    //we only care about keeping track of different SearchManager clients, 
    //so we can lump together all other users using null.
    if (client instanceof SearchManager) {
      log.error("[getTestingInstance] This method should only be used by the GUI for testing!");
      addClient(client, newSearch);
    } else {
      addClient(null, newSearch);
    }

    return newSearch;
  }
  ///////////////////////////////// END GUI methods //////////////////////////////////////////

  /**
   * Attempt to stop all searches that are currently running. Only searches that were started by the caller
   * can be stopped.
   *
   * @param caller - requesting object (usually "this")
   */
  public synchronized void stopAllSearches(Object caller) {
    // stop all running searches
    List<SearchManager> runningSearches = types_byId.values().stream().filter(s -> s.isRunning()).collect(Collectors.toList());
    runningSearches.stream().forEach(s -> s.stop(caller, false));

    // only want to keep track of most recently stopped searches
    stoppedSearches.clear();
    stoppedSearches.addAll(runningSearches);
  }

  /**
   * Restart all searches that were stopped by the most recent call to stopAllSearches. This method should be
   * thought of as a pair with stopAllSearches.
   * @param caller - requesting object (usually "this")
   */
  public synchronized void restartAllStoppedSearches(Object caller) {
    stoppedSearches.stream().forEach(s -> s.start(caller));
  }

  /**
   * Find out if a SearchManager already exists
   * that is capable of detecting objects that meet the specified
   * descriptors/constraints.
   *
   * @param descriptors - list of Terms describing search constraints
   * @return - boolean
   */
  public synchronized boolean hasExistingSearchManager(final List<? extends Term> descriptors) {
    return getExistingSearchManager(descriptors) == null ? false : true;
  }

  /**
   * Find out if a SearchManager can be built (without actually building one)
   * that is capable of detecting objects that meet the specified
   * descriptors/constraints.
   *
   * @param descriptors - list of Terms describing search constraints
   * @return - boolean
   */
  public synchronized boolean canCreateCapableSearchManager(final List<? extends Term> descriptors) {
    List<Term> unsatisfiableConstraints = getUnsatisfiableConstraints(descriptors);
    if (unsatisfiableConstraints.isEmpty()) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Check if there are any unsatisfiable constraints contained in the passed in
   * descriptors/constraints.
   *
   * @param descriptors - list of Terms describing search constraints
   * @return list of (possibly empty) descriptors/constraints
   */
  public synchronized List<Term> getUnsatisfiableConstraints(final List<? extends Term> descriptors) {

    // find which descriptors do not map to a valid vision process
    List<Term> unsatisfiableConstraints = new ArrayList<>();
    for (Term descriptor : descriptors) {
      if (!Vision.availableDetectors.hasCapableDetector(descriptor)
              && !Vision.availableValidationProcessors.hasCapableProcessorType(descriptor)
              && typeDescriptions_byName.get(PredicateHelper.getRepresentativeString(descriptor)) == null) {
        unsatisfiableConstraints.add(descriptor);
      }
    }

    return unsatisfiableConstraints;
  }

  /**
   * Create a new empty SearchManager. This is useful for creating SearchManager
   * incrementally and from the GUI. Clients must call release on a
   * SearchManager instance when finished using it, so that SearchManager can be
   * properly managed and garbage collected.
   *
   * @param client java object requesting the instance
   * @return SearchManager
   */
  public synchronized SearchManager getInstance(final Object client) {
    SearchManager newSearch = new CompositeSearchManager(incrementalImgProc, incrementalDetector, serialProcessing, singleIteration);
    long id = newSearch.getTypeId();
    types_byId.put(id, newSearch);
    orderedTypes.add(newSearch);

    //we only care about keeping track of different SearchManager clients, 
    //so we can lump together all other users using null.
    if (client instanceof SearchManager) {
      addClient(client, newSearch);
    } else {
      addClient(null, newSearch);
    }

    return newSearch;
  }

  /**
   * Get SearchManager matching description. First tries to find an existing
   * SearchManager. If one cannot be found, the creation of a new SearchManager
   * is attempted. Clients must call release on a SearchManager instance when
   * finished using it, so that SearchManager can be properly managed and
   * garbage collected.
   *
   * @param client      - calling object, usually "this"
   * @param descriptors - Predicate description of desired SearchManager
   * @return SearchManager, or null if one could not be found or created
   */
  public synchronized SearchManager getInstance(final Object client, List<? extends Term> descriptors, boolean useExistingSearches) {
    SearchManager match = null;

    // check if descriptors are satisfiable
    if (!canCreateCapableSearchManager(descriptors)) {
      return match;
    }

    // exchange each learned named descriptor with its description
    // e.g., medkit(X) --> red(Y),cross(Y),white(Z),box(Z),on(Y,Z)
    descriptors = replaceNamedDescriptors(descriptors);

    // TODO: when should existing searches be used by new hierarchical searches?
    // if not adding onto an existing search manager, try and find existing SearchManager match
//    if (!(client instanceof SearchManager)) {
    if (useExistingSearches) {
      match = getExistingSearchManager(descriptors);
    }

    // TODO: how to ensure variable consistency between incoming searches that use existing searches
    // with potentially different variable names ??

    //finally, if existing match isn't found, try to build new search
    if (match == null) {
      match = createSearchManager(client, descriptors);
    }

    if (match == null) {
      log.warn("SearchManager could not be created for search: " + Arrays.toString(descriptors.toArray()));
    } else {
      //
      // TODO: create power references -- where should this be done?
//      List<Variable> vars = PredicateHelper.getUnboundVariables(descriptors);
//      Map<Variable, Symbol> objRefs = Vision.consultant.assertProperties(vars, null, 0.0, null);
//      Vision.consultant.assertProperties(null, objRefs, 1.0, VisionConsultant.convertToProperties(descriptors));
//      Vision.consultant.setTypeId();

      // keep track of SearcManager clients
      // NOTE: we only care about keeping track of different SearchManager clients,
      // so we can lump together all other users using null.
      if (client instanceof SearchManager) {
        addClient(client, match);
      } else {
        addClient(null, match);
      }
    }

    return match;
  }

  /**
   * Tries to get SearchManager with specified ID. If the SearchManager doesn't
   * exist, null is returned. Clients must call release on a SearchManager
   * instance when finished using it, so that SearchManager can be properly
   * managed and garbage collected.
   *
   * @param client
   * @param searchTypeId SearchManager ID
   * @return SearchManager or null
   */
  public synchronized SearchManager getInstance(final Object client, final long searchTypeId) {
    SearchManager searchType = types_byId.get(searchTypeId);

    //TODO: decide if this is really necessary
//    if (searchType != null) {
//      //we only care about keeping track of different SearchManager clients, 
//      //so we can lump together all other users using null.
//      if (client instanceof SearchManager) {
//        addClient(client, searchType);
//      } else {
//        addClient(null, searchType);
//      }
//    }
    return searchType;
  }

  /**
   * Get an already instantiated SearchManager if one exists, otherwise return
   * null. This only checks top-level SearchManagers, not sub-SearchManagers
   * within a hierarchical SearchManager.
   *
   * @param descriptors
   * @return
   */
  private SearchManager getExistingSearchManager(final List<? extends Term> descriptors) {
    for (SearchManager currSearchType : types_byId.values()) {
      if (currSearchType.matchesDescriptors(descriptors)) {
        return currSearchType;
      }
    }
    return null;
  }

  /**
   * Create new SearchManager matching descriptors. Does not check to see if a
   * SearchManager matching this description already exists, so use caution to
   * ensure duplicate SearchManagers are not created! Assumes descriptors have
   * already been flattened.
   *
   * @param descriptors - Predicate description of desired SearchManager
   * @return SearchManager, or null if one could not be created
   */
  private synchronized SearchManager createSearchManager(Object client, final List<? extends Term> descriptors) {

    //separate predicates by unbound variable
    Map<List<Variable>, List<Term>> separatedDescriptors = PredicateHelper.splitIntoEntities(descriptors);

    // build a new search manager
    SearchManager searchManager = null;
    if (separatedDescriptors.size() == 1 && separatedDescriptors.keySet().iterator().next().size() == 1) {
      //if only a single variable, just need to construct a simple search
      searchManager = createSimpleSearchManager(client, separatedDescriptors.values().iterator().next());
    } else {
      // check all the relation descriptors (i.e., those with two vars) to see if any of them can only be satisfied with
      // a Detector. If that's the case, create a RelationSearchManager, else (if all relation descriptors can be
      // satisfied with a Validator) create a CompositeSearchManager
      boolean relationSearch = false;
      for (Map.Entry<List<Variable>, List<Term>> entry : separatedDescriptors.entrySet()) {
        // if has two vars and one term
        if (entry.getKey().size() == 2 && entry.getValue().size() == 1) {
          // and has Detector option but no Validator option
          if (Vision.availableDetectors.hasCapableDetector(entry.getValue().get(0))
                  && !Vision.availableValidationProcessors.hasCapableProcessorType(entry.getValue().get(0))) {
            relationSearch = true;
          }
        }
      }

      // determine if should be composite or relation search manager
      if (relationSearch) {
        searchManager = createRelationSearchManager(client, separatedDescriptors);
      } else {
        searchManager = createCompositeSearchManager(client, separatedDescriptors);
      }
    }

    return searchManager;
  }

  /**
   * Instantiate a new SimpleSearchManager. This method assumes that all
   * descriptors belong to the same search (usually means they all have the same
   * Variable).
   *
   * @param client      - used to determine if new search manager will be part of a
   *                    composite search manager or stand-alone search manager
   * @param descriptors
   * @return
   */
  private SearchManager createSimpleSearchManager(Object client, final List<? extends Term> descriptors) {
    SearchManager newSearch;
    // FIXME: this instanceOf checking is terrible -- need a better way to determine of client's typeId should be used
    if (client instanceof RelationSearchManager) {
      newSearch = new SimpleSearchManager(incrementalImgProc, incrementalDetector, serialProcessing, singleIteration);
    } else if (client instanceof SearchManager) {
      SearchManager clientSM = (SearchManager) client;
      newSearch = new SimpleSearchManager(clientSM.getTypeId(), incrementalImgProc, incrementalDetector, serialProcessing, singleIteration);
    } else {
      newSearch = new SimpleSearchManager(incrementalImgProc, incrementalDetector, serialProcessing, singleIteration);
    }

    for (Term descriptor : descriptors) {
      if (!newSearch.addConstraint(descriptor)) {
        return null;
      }
    }

    long id = newSearch.getTypeId();
    // only add top-level SearchManagers to types_byId
    if (!types_byId.containsKey(id)) {
      types_byId.put(id, newSearch);
    }
    orderedTypes.add(newSearch);
    orderedSimpleSearchTypes.add(newSearch);
    return newSearch;
  }

  /**
   * Instantiate a new CompositeSearchManager.
   *
   * @param client
   * @param separatedDescriptors
   * @return
   */
  private SearchManager createCompositeSearchManager(Object client, Map<List<Variable>, ? extends List<? extends Term>> separatedDescriptors) {
    CompositeSearchManager newSearch;
    // FIXME: this instanceOf checking is terrible -- need a better way to determine of client's typeId should be used
    if (client instanceof RelationSearchManager) {
      newSearch = new CompositeSearchManager(incrementalImgProc, incrementalDetector, serialProcessing, singleIteration);
    } else if (client instanceof SearchManager) {
      SearchManager clientSM = (SearchManager) client;
      newSearch = new CompositeSearchManager(clientSM.getTypeId(), incrementalImgProc, incrementalDetector, serialProcessing, singleIteration);
    } else {
      newSearch = new CompositeSearchManager(incrementalImgProc, incrementalDetector, serialProcessing, singleIteration);
    }

    if (!newSearch.addConstraints(separatedDescriptors)) {
      newSearch.terminate();
      return null;
    }

    long id = newSearch.getTypeId();
    types_byId.put(id, newSearch);
    orderedTypes.add(newSearch);
    return newSearch;
  }

  /**
   * Instantiate a new RelationSearchManager.
   *
   * @param client
   * @param separatedDescriptors
   * @return
   */
  private SearchManager createRelationSearchManager(Object client, Map<List<Variable>, ? extends List<? extends Term>> separatedDescriptors) {
    RelationSearchManager newSearch = new RelationSearchManager(incrementalImgProc, incrementalDetector, serialProcessing, singleIteration);

    if (!newSearch.addConstraints(separatedDescriptors)) {
      newSearch.terminate();
      return null;
    }

    long id = newSearch.getTypeId();
    types_byId.put(id, newSearch);
    orderedTypes.add(newSearch);
    return newSearch;
  }

  /**
   * Get all instantiated top-level SearchManagers.
   *
   * @return List of SearchManagers
   */
  public synchronized List<SearchManager> getAll() {
    return new ArrayList(types_byId.values());
  }

  /**
   * Get IDs of all instantiated SearchManagers. The SearchManagers are not
   * necessarily running.
   *
   * @return List of SearchManager IDs
   */
  public synchronized List<Long> getTypeIds() {
    List<Long> ids = new ArrayList();
    for (SearchManager currSearchType : types_byId.values()) {
      ids.add(currSearchType.getTypeId());
    }
    return ids;
  }

  /**
   * Name an existing SearchManager. Name does not have to be unique, and a
   * single SearchManager can have multiple names. This is useful so that
   * collections of predicates that describe a particular search can be named
   * (e.g., white box with red cross can be named medkit).
   *
   * @param searchTypeId
   * @param searchTypeName
   */
  public synchronized void addTypeName(Long searchTypeId, Term searchTypeName) {
    // add to SearchManager
    SearchManager searchType = types_byId.get(searchTypeId);
    if (searchType != null) {
      // name instantiated SearchManager
      // TODO: could this be problematic if descriptors are added to SearchManager
      // after it's named?
      searchType.setName(searchTypeName);

      // assign name to the collection of descriptors for general use
      // while instantiating searches
      nameDescriptors(searchType.getDescriptors(), searchTypeName);

      //update GUI to reflect new Name option
      //updateGuiEntry(searchType);
    }
  }

  /**
   * Name a list of descriptors. E.g., assign the list of descriptors
   * "red(Y),cross(Y),white(X),box(X),on(Y,X)" the name "medkit(X)".
   *
   * @param descriptors
   * @param typeName
   */
  public synchronized void nameDescriptors(final List<? extends Term> descriptors, final Term typeName) {
    //add to Factory's list of names
    String searchNameStr = PredicateHelper.getRepresentativeString(typeName);
    List<NamedDescription> descriptions = typeDescriptions_byName.get(searchNameStr);
    if (descriptions == null) {
      descriptions = new ArrayList();
      typeDescriptions_byName.put(searchNameStr, descriptions);
    }

    // TODO: when to allow multiple definitions?
    // clear old definitions
//    descriptions.clear();

    descriptions.add(new NamedDescription(typeName, descriptors));
  }

  /**
   * Remove a list of descriptors from the named descriptors. E.g., remove the list of descriptors
   * "red(Y),cross(Y),white(X),box(X),on(Y,X)" from the name "medkit(X)".
   *
   * @param descriptors
   * @param typeName
   */
  public synchronized void removeNamedDescriptors(final List<? extends Term> descriptors, final Term typeName) {
    // remove from Factory's list of names
    String searchNameStr = PredicateHelper.getRepresentativeString(typeName);
    List<NamedDescription> descriptions = typeDescriptions_byName.get(searchNameStr);
    if (descriptions != null) {
      for (NamedDescription namedDescription : descriptions) {
        if (Utilities.predicatesMatch(namedDescription.getDescriptors(), descriptors)) {
          descriptions.remove(namedDescription);
          break;
        }
      }
    }

    if (descriptions.isEmpty()) {
      typeDescriptions_byName.remove(searchNameStr);
    }
  }

  /**
   * Replace descriptors that have been learned via nameDescriptors with
   * their corresponding description. E.g., replace "medkit(X)" with
   * "red(Y),cross(Y),white(X),box(X),on(Y,X)".
   *
   * @param descriptors
   * @return
   */
  protected List<Term> replaceNamedDescriptors(final List<? extends Term> descriptors) {
    List<Term> exchangedDescriptors = new ArrayList<>();
    for (Term d : descriptors) {
      String searchNameStr = PredicateHelper.getRepresentativeString(d);
      if (typeDescriptions_byName.containsKey(searchNameStr)) {
        List<NamedDescription> descriptions = typeDescriptions_byName.get(searchNameStr);
        List<Term> consistentDescriptors = createConsistentDescriptors(d, descriptions.get(0), descriptors);
        exchangedDescriptors.addAll(consistentDescriptors);
      } else {
        exchangedDescriptors.add(d);
      }
    }

    log.debug("[replaceNamedDescriptors] original: " + descriptors);
    log.debug("[replaceNamedDescriptors] exchanged: " + exchangedDescriptors);
    return exchangedDescriptors;
  }

  protected List<Term> appendNamedDescriptors(final List<? extends Term> descriptors) {
    List<Term> exchangedDescriptors = new ArrayList<>();
    for (Term d : descriptors) {
      String searchNameStr = PredicateHelper.getRepresentativeString(d);
      if (typeDescriptions_byName.containsKey(searchNameStr)) {
        List<NamedDescription> descriptions = typeDescriptions_byName.get(searchNameStr);
        List<Term> consistentDescriptors = createConsistentDescriptors(d, descriptions.get(0), descriptors);
        exchangedDescriptors.add(d);
        exchangedDescriptors.addAll(consistentDescriptors);
      } else {
        exchangedDescriptors.add(d);
      }
    }

    log.debug("[replaceNamedDescriptors] original: " + descriptors);
    log.debug("[replaceNamedDescriptors] exchanged: " + exchangedDescriptors);
    return exchangedDescriptors;
  }

  /**
   * Ensure the descriptors in list one are consistent with the descriptors in
   * list two. This means that (1) variable names in the first list of
   * descriptors are consistent with the variable names in the second list of
   * descriptors, and (2) that duplicate descriptors do not exist between the
   * two lists.
   *
   * @param namedDescriptor  original descriptor being replaced with
   *                         namedDescription
   * @param namedDescription description replacing the original namedDescriptor
   * @param descriptors      list that results must be consistent with
   */
  private List<Term> createConsistentDescriptors(Term namedDescriptor, final NamedDescription namedDescription, final List<? extends Term> descriptors) {
    List<Term> consistentDescriptors = new ArrayList<>();

    // first do consistency check between namedDescriptor and namedDescription
    Map<Variable, Variable> varMapping = new HashMap<>();
    for (int i = 0; i < namedDescription.getName().size(); ++i) {
      // check if variable args of name descriptor have different names
      Symbol arg1 = namedDescription.getName().get(i);
      Symbol arg2 = namedDescriptor.get(i);
      if (arg1.isVariable() && arg2.isVariable() && !arg1.getName().equals(arg2.getName())) {
        // if variables have different names -- add to mapping from old to new
        varMapping.put((Variable) arg1, (Variable) arg2);
      }
    }

    // now do consistency check against the rest of the descriptors
    for (Term descriptorToModify : namedDescription.getDescriptors()) {
      // only need to check single arg terms -- follows from ASSUMPTION that 
      // currently in vision all terms with more than 1 arg are relational 
      // and all relational terms' variables must exist in another single arg
      // term (to not have disjoint search descriptors)
      if (descriptorToModify.size() == 1) {
        for (Term descriptor : descriptors) {
          if (descriptorToModify.getName().equals(descriptor.getName())) {
            // check if variable args have different names
            Symbol arg1 = descriptorToModify.get(0);
            Symbol arg2 = descriptor.get(0);
            if (arg1.isVariable() && arg2.isVariable() && !arg1.getName().equals(arg2.getName())) {
              // if variables have different names -- add to mapping from old to new
              varMapping.put((Variable) arg1, (Variable) arg2);
            }
            break;  // found matching descriptor -- break
          }
        }
      }
    }

    // apply var mappings
    for (Term descriptorToModify : namedDescription.getDescriptors()) {
      List<Symbol> newArgs = new ArrayList<>();
      for (Symbol arg : descriptorToModify.getArgs()) {
        if (arg.isVariable() && varMapping.containsKey((Variable) arg)) {
          newArgs.add(varMapping.get((Variable) arg));
        } else {
          newArgs.add(arg);
        }
      }
      consistentDescriptors.add(new Term(descriptorToModify.getName(), newArgs));
    }

    // get rid of duplicate descriptors
    Iterator<Term> iterator = consistentDescriptors.iterator();
    while (iterator.hasNext()) {
      Term consistentDescriptor = iterator.next();
      for (Term descriptor : descriptors) {
        if (consistentDescriptor.toString().equals(descriptor.toString())) {
          iterator.remove();
        }
      }
    }

    return consistentDescriptors;
  }

  /**
   * Get names of all searches that have been named. Not all searches are
   * required to be named. Performs a shallow copy.
   *
   * @return Set of Strings
   */
  public synchronized Set<String> getTypeNames() {
    return new HashSet<>(typeDescriptions_byName.keySet());
  }

  /**
   * Get definition (name plus definition) of all searches that have been named.
   * Performs a shallow copy.
   *
   * @return List of NamedDescription
   */
  public synchronized List<NamedDescription> getDefinitions() {
    List<NamedDescription> definitions = new ArrayList<>();

    for (List<NamedDescription> def : typeDescriptions_byName.values()) {
      definitions.addAll(def);
    }
    return definitions;
  }

  /**
   * Helper function to add clients to list.
   *
   * @param client Object requesting the instance
   * @param search instance being handed out to client
   */
  private void addClient(final Object client, final SearchManager search) {
    HashSet<Object> clientList = instantiatedSearchManagerClients.get(search);
    if (clientList == null) {
      clientList = new HashSet();
      instantiatedSearchManagerClients.put(search, clientList);
    }

    if (clientList.contains(client)) {
      if (client != null) {
        log.error("[addClient] Calling object is already a client of the SearchManager.");
      }
    } else {
      //add client to SearchManager's client list
      clientList.add(client);
    }
  }

  /**
   * Client needs to let managing factory know when it's done using a
   * SearchManager instance, so that SearchManager can be properly managed and
   * garbage collected.
   */
  public synchronized void release(final Object client, final SearchManager searchType) {
    //we only care about keeping track of different SearchManager clients, 
    //so we can lump together all other users as null.
    Object effectiveClient = null;
    if (client instanceof SearchManager) {
      effectiveClient = client;
    }

    HashSet<Object> clientList = instantiatedSearchManagerClients.get(searchType);
    if (clientList == null || !clientList.contains(effectiveClient)) {
      log.warn("[release] Calling object is not a client of the SearchManager.");
      return;
    } else {

      //make sure client is removed from start callers
      if (searchType.isRunning()) {
        searchType.stop(effectiveClient, true);
      }

      //remove from client list
      clientList.remove(effectiveClient);

      //if SearchManager doesn't have any clients, clean up references
      if (clientList.isEmpty()) {
        removeSearchType(searchType);
      }
    }
  }

  private synchronized void removeSearchType(SearchManager searchType) {
    //remove from types_byId
    types_byId.remove(searchType.getTypeId());

    //remove it from all containers so it can be garbage collected
    instantiatedSearchManagerClients.remove(searchType);

    //should be stopped
    if (searchType.isRunning()) {
      log.error("[removeSearchType] Trying to remove type while it's running!");
    }

    //remove from GUI containers
    orderedTypes.remove(searchType);
    orderedSimpleSearchTypes.remove(searchType);

    //signal the release of VisionProcessors
    searchType.terminate();

  }

}
