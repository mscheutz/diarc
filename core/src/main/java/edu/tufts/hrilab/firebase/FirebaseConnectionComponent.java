/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.firebase;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageParser;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.temiv3.UIComponent;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

//TODO: add auth logic to this class and make configurable whether to actually use it seriously or to use fake creds and
//  manually set the user group
//TODO: Make the methods in this class safe against missing data in firestore documents (currently have unsafe typecasts
//   are scattered throughout)
public abstract class FirebaseConnectionComponent extends UIComponent {

  private boolean addedAgentHierarchy;

  //Android: The group name used is pulled from firebase at runtime from the corresponding entry in the
  //  base "users" collection for this firestore project. The UUID used for the corresponding
  //  document is the auth UUID of the account used for sign in from the SignInActivity
  //Desktop: Passed in on the command line
  protected String groupName;

  //TODO: allow this to be set through config?
  private String learnedActionsFileString;

  //Not used rn
  Map<String, Symbol> locations;
  private List<String> locationBackupNames;
  private List<String> videoNames;

  private boolean useAuth;


  public FirebaseConnectionComponent() {
    super();
    groupName = "tr_test";

    learnedActionsFileString = "";

    locations = new HashMap<>();
    locationBackupNames = new ArrayList<>();
    videoNames = new ArrayList<>();

    useAuth = false;
  }

  @Override
  protected void init() {
    super.init();
  }

  protected void addFirebaseListeners() {
    attachUserInputListener();
    attachEditedASLListener();
    attachLoadASLListener();
    attachHomophoneListener();
    attachBeliefQueryListener();
    attachNavigationBackupListener();

    Map<String, Object> groupDocData = getDocumentData("groups/" + groupName);

    initSystemGoals();

    ////foodOrdering specific //TODO: don't have this live here and/or always occur by default
    //attachTakenMealsListener();
  }


  public interface Collectionlistener {
    //Add doc id under special key
    //Add docChangeType under special key?
    void onCollectionTrigger(List<Map<String, Object>> data);
  }

  public abstract boolean writeToDocument(String documentPath, Map<String, Object> data);

  public abstract boolean updateDocument(String documentPath, Map<String, Object> data);

  public abstract boolean writeToCollection(String collectionPath, Map<String, Object> data);

  public abstract boolean deleteDocument(String path);

  public abstract boolean deleteCollection(String path, int batchSize);

  public abstract List<Map<String, Object>> getCollectionDocumentsData(String path);

  public abstract Map<String, Object> getDocumentData(String path);

  public abstract void attachCollectionListener(String path, Collectionlistener collectionlistener);

  public abstract void attachWhereGreaterThanCollectionListener(String param, Object value, String path, Collectionlistener collectionlistener);

  public abstract String deleteFromStorage(String path);

  public abstract boolean writeToStorage(byte[] data, Map<String, Object> metadata);

  //TODO: add and use auth methods from app
  //public abstract String createUser(String email, String pw);
  //public abstract String signIn(String email, String pw);
  //public abstract boolean connectToEmulator(String firebaseIp, String authPort, String firestorePort, String databasePort, String storagePort);
  //public abstract boolean disconnectFromEmulator();
  @Override
  protected void executionLoop() {
    super.executionLoop();
    if (!addedAgentHierarchy) {
      try {
        Map<Symbol,Set<Symbol>> agentHierarchy = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllAgentTeams")).call(Map.class);
        if (agentHierarchy != null) {
          Map<String, List<String>> untypedStringAgentHierarchy = new HashMap<>();
          for (Map.Entry<Symbol, Set<Symbol>> agentTeam : agentHierarchy.entrySet()) {
            String teamName = agentTeam.getKey().toUntypedString();
            Set<Symbol> teamMembers = agentTeam.getValue();
            List<String> untypedTeamMembers = new ArrayList<>();
            for (Symbol teamMember : teamMembers) {
              String name = teamMember.toUntypedString();
              if (!name.equals(teamName)) {
                untypedTeamMembers.add(teamMember.toUntypedString());
              }
            }
            untypedStringAgentHierarchy.put(teamName, untypedTeamMembers);
          }
          Map<String, Object> data = new HashMap<>();
          data.put("agentHierarchy", untypedStringAgentHierarchy);
          writeToDocument("groups/" + groupName, data);
          addedAgentHierarchy = true;
        }
      } catch (TRADEException e) {
        log.error("[FirebaseConnectionComponent] error calling getDiarcAgents", e);
      }
    }
  }

  protected void localshutdown() {
    log.warn("logs sent from local shutdown");
  }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Start Firebase Listeners ///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Reset all instance-specific collections on startup, prevent out of date information from displaying in the GUI
  //  on restarts
  protected void resetExistingCollections() {
    log.debug("[resetExistingCollections] deleting all documents in existing leaf collections");
    deleteCollection("groups/" + groupName + "/systemGoals/", 3);
    deleteCollection("groups/" + groupName + "/agentGoals/", 3);
    deleteCollection("groups/" + groupName + "/actionHistory/", 3);
    deleteCollection("groups/" + groupName + "/beliefNotification/", 3);
    deleteCollection("groups/" + groupName + "/beliefQuery/", 3);
    deleteCollection("groups/" + groupName + "/beliefResponse/", 3);
    deleteCollection("groups/" + groupName + "/takenMeals/", 3);
    deleteCollection("groups/" + groupName + "/editedASL/", 3);
    deleteCollection("groups/" + groupName + "/loadASLFile/", 3);
    deleteCollection("groups/" + groupName + "/loadASLResponses/", 3);
    deleteCollection("groups/" + groupName + "/writeASLResponses/", 3);
    deleteCollection("groups/" + groupName + "/ttsRequests/", 3);
    deleteCollection("groups/" + groupName + "/generatedASL/", 3);
    deleteCollection("groups/" + groupName + "/dictionary", 10);
  }


  private void initSystemGoals() {
    HashMap<String, Object> systemGoalsInfo = new HashMap<>();
    List<HashMap<String, Object>> goalsInfo = new ArrayList<>();
    systemGoalsInfo.put("pending", goalsInfo);
    systemGoalsInfo.put("active", goalsInfo);
    writeToDocument("groups/" + groupName + "/systemGoals/goals", systemGoalsInfo);
  }

  //Set up userInput db listener - receives utterances from the webapp or ASR app and passes on to dialogue
  private void attachUserInputListener() {
    String collectionPath = "groups/" + groupName + "/userInput";

    Collectionlistener userInputListener = (collectionData) -> {
      for (Map<String, Object> documentData : collectionData) {
        if (documentData.get("documentChangeType") == "ADDED") {
          log.debug("getting changes in document: " + documentData.get("docId"));
          //get values from input and package into utterance
          String speakerName = (String) documentData.get("speaker");
          String utteranceBody = (String) documentData.get("utterance");
          String docListener = (String) documentData.get("listener");
          log.debug("speaker name: " + speakerName);
          if (speakerName != null && utteranceBody != null) {
            //Utterance that was originally supplied
            List<String> utteranceWords = utteranceToWords(utteranceBody);

            //New for assembly demo - storing any translation that has occurred alongside base utterance
            Map<String, List<String>> translations = new HashMap<>();
            String language = (String) documentData.get("language");
            Map<String, Object> translationsObject = (Map<String, Object>) documentData.get("translations");
            if (translationsObject != null) {
              for (String lang : translationsObject.keySet()) {
                translations.put(lang, utteranceToWords((String) translationsObject.get(lang)));
              }
            }

            log.info("utterance words from userInput listener: " + utteranceWords);
            //Utterance utterance = new Utterance(new Symbol(speakerName), new Symbol(docListener), utteranceWords, UtteranceType.UNKNOWN, language, translations);
            //TODO:brad: add back translation info?
            Utterance utterance = new Utterance(new Symbol(speakerName), new Symbol(docListener), utteranceWords, UtteranceType.UNKNOWN, true);
            deleteDocument(collectionPath + "/" + documentData.get("docId"));
            try {
              TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class)).call(void.class, utterance);
            } catch (TRADEException err) {
              log.error("reportRecognizedSpeech failed.", err);
            }
          }
        }
      }
    };

    attachWhereGreaterThanCollectionListener("time", System.currentTimeMillis(), collectionPath, userInputListener);
  }

  private List<String> utteranceToWords(String utterance) {
    List<String> words = new ArrayList<>();
    if (utterance.contains(" ")) {
      words = Arrays.asList(utterance.split(" "));
    } else {
      words.add(utterance);
    }
    return words;
  }

  //Setup editedASL listener
  //The current iteration of the webapp has a component which displays all entries written to the 'generatedASL'
  //  collection on firebase. This collection is currently populated from this component through listeners attached to
  //  the Goal Manager's actionAdded/actionRemoved notification pipeline. Actions to be written are filtered by
  //  the -dbfile and -dbaction commandline args; in addition, all learned actions and plans are written.
  //  ASL for these actions can be manually edited and resubmitted by users through the webapp, at which point it will
  //  be recieved by this listener here.
  //This listener attempts to parse submitted ASL and, depending on variables set by the webapp user or through this
  //  component, may add parsed entries to the current database and/or write the modified ASL to file.
  private void attachEditedASLListener() {
    log.debug("[FirebaseConnectionComponent] setting up listener for editedASL collection");
    String collectionPath = "groups/" + groupName + "/editedASL";

    //Attach listener
    Collectionlistener editedASLListener = (collectionData) -> {

      log.debug("[FirebaseConnectionComponent] listener for edited asl collection triggered");
      //Modified ASL text submitted from webapp which we will attempt to parse
      String asl;
      //Local file to write ASL text to. Default behavior is not to write to file, otherwise the user has the option to
      // supply a filename through the webapp when submitting ASL.
      String filename = learnedActionsFileString;
      //Boolean indicating whether to load the resulting DBEntry into the database or not. The user is able to signal
      //  this through the webapp
      boolean separate = false;
      //Response message to populate alert back in the webapp. Used to let the user know whether the submitted asl
      //  was able to be parsed or not
      String responseMessage;
      //Agents to be associated with this action - currently will always be the agents the original version of this
      //  action contained
      List<Symbol> agents = null;

      //No error, iterate over all changes (should mainly be one at a time)
      for (Map<String, Object> s : collectionData) {
        //Get modified ASL
        asl = (String) s.get("asl");
        log.info("[editedASL] Have asl from webapp: " + asl);

        //Update default values described above
        if (s.get("filename") != null) {
          filename = (String) s.get("filename");
        }
        if (s.get("separate") != null) {
          separate = (Boolean) s.get("separate");
        }
        List<String> agentStrings = (ArrayList<String>) s.get("actors");
        responseMessage = "Wrote asl to file " + filename + " successfully";

        //Prevent bugs in the webapp from causing exceptions
        if (asl == null || agentStrings == null) {
          log.warn("[editedASLListener] document written to firebase missing information, not adding asl");
          continue;
        }

        //Get agents associated with the original version of this action
        if (!agentStrings.isEmpty()) {
          agents = new ArrayList<>();
          for (String agentString : agentStrings) {
            agents.add(Factory.createSymbol(agentString));
          }
        }
        log.debug("[editedASL] Have actors: " + agents);

        //Attempt to parse
        ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
        List<ActionDBEntry> entries = parser.parseFromString(filename, agents, asl);
        //Error parsing ASL
        if (entries == null) {
          log.warn("could not parse submitted asl");
          responseMessage = "Could not parse supplied asl";
          if (!separate) {
            responseMessage += ". This action has been reloaded in its broken state and will no longer work";
          }
        }
        //Successfully parsed ASL
        else {
          //Add to current database/update existing entries if desired
          if (!separate) {
            entries.forEach(action -> Database.getInstance().addActionDBEntry(action));
          }

          //TODO: This actually depends on the machine this class is implemented on
          //If filename has been set either through the webapp or this component, write to file
          if (!filename.isEmpty()) {
            File tempASLFile = new File(filename);
            try {
              log.debug("[editedASL] saving asl to file: " + filename);
              File parentFile = tempASLFile.getParentFile();
              if (parentFile != null) {
                parentFile.mkdirs();
              } else {
                throw new IOException("parent file for supplied filename doesn't exist");
              }
              tempASLFile.createNewFile();
              FileOutputStream outputStream = new FileOutputStream(tempASLFile, true);
              byte[] strToBytes = ("\n" + asl).getBytes();
              outputStream.write(strToBytes);
              outputStream.close();
            } catch (IOException e) {
              log.error("Unable to to create temp asl file for webapp modified asl: " + asl, e);
              responseMessage = "Unable to find or create file to write asl to";
              if (!separate) {
                responseMessage += ". This action has still been reloaded for the current session";
              }
            }
          }
        }

        //Write response status back to webapp for error handling
        Map<String, Object> response = new HashMap<>();
        response.put("message", responseMessage);
        writeToDocument("groups/" + groupName + "/writeASLResponses/" + s.get("docId"), response);

        //Completed processing, delete entry
        log.debug("[editedASL] deleting edited asl document");
        deleteDocument(collectionPath + "/" + s.get("docId"));
      }
    };

    attachCollectionListener(collectionPath, editedASLListener);
  }

  //Initially implemented for users at show; The idea was to was use this in conjunction the above editedASL
  //  functionality to load new actions on the fly while using docker
  //Set up loadASLFile listener - Users currently have the option through the webapp to load ASL entries dynamically
  //  from specified files.
  private void attachLoadASLListener() {
    String collectionPath = "groups/" + groupName + "/loadASLFile";

    Collectionlistener loadASLListener = (collectionData) -> {
      for (Map<String, Object> s : collectionData) {
        if (s.get("documentChangeType") == "ADDED") {
          String filename = (String) s.get("filename");
          //TODO: designate agent(s)?
          ActionScriptLanguageParser adbp = new ActionScriptLanguageParser();
          List<ActionDBEntry> parsedActions = adbp.parseFromFile(filename, null);
          parsedActions.forEach(action -> Database.getInstance().addActionDBEntry(action));
          Map<String, Object> response = new HashMap<>();
          if (parsedActions == null || parsedActions.isEmpty()) {
            response.put("message", "Could not find or parse asl for supplied file name: " + filename);
          } else {
            response.put("message", "Successfully parsed file: " + filename);
          }
          writeToDocument("groups/" + groupName + "/loadASLResponses", response);
          deleteDocument(collectionPath + "/" + s.get("docId"));
        }
      }
    };

    attachCollectionListener(collectionPath, loadASLListener);
  }

  //Listener on 'homophones' collection for definition of homophones from webapp. Given
  //  a base morpheme and corresponding homophone, all dictionary entries for the base morpheme
  //  will be duplicated and added using the new homophone as the morpheme.
  private void attachHomophoneListener() {
    log.debug("[FirebaseConnectionComponent] setting up listener for homophone collection");

    String path = "groups/" + groupName + "/homophones";

    Collectionlistener homophoneListener = (collectionData) -> {
      for (Map<String, Object> s : collectionData) {
        if (s.get("documentChangeType") == "ADDED") {
          String base = (String) s.get("base");
          String homophone = (String) s.get("homophone");
          boolean writeToFile = Objects.equals((String) s.get("write"), "on");
          try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("addHomophone").argTypes(String.class,String.class,Boolean.class)).call(void.class, base, homophone, writeToFile);
          } catch (TRADEException e) {
            log.error("unable to add dictionary entry for homophone: " + homophone, e);
          }

          log.debug("deleting homophone document");
          deleteDocument(path + "/" + s.get("docId"));
        }
      }
    };

    attachCollectionListener(path, homophoneListener);
  }

  //Listener on 'beliefQuery' collection for belief queries from webapp.
  //Query response is written to the 'beliefResponse' collection to be processed by the webapp
  private void attachBeliefQueryListener() {
    log.debug("[FirebaseConnectionComponent] setting up listener for beliefQuery collection");

    String path = "groups/" + groupName + "/beliefQuery";

    Collectionlistener beliefQueryListener = (collectionData) -> {
      for (Map<String, Object> s : collectionData) {
        String queryString = (String) s.get("text");

        //TODO: rather than converting to Term and querying belief, link to querySQL and directly pass in string?
        Term queryTerm = Factory.createPredicate(queryString);
        List<Map<Variable, Symbol>> response = null;
        try {
          response = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Term.class)).call(List.class, queryTerm);
        } catch (TRADEException e) {
          log.error("unable to add query belief with term: " + queryTerm, e);
        }

        deleteDocument(path + "/" + s.get("docId"));

        String responsePath = "groups/" + groupName + "/beliefResponse";
        HashMap<String, Object> responseDoc = new HashMap<>();
        log.info("[beliefQueryListener] response: " + response);
        if (response == null) {
          responseDoc.put("text", "Query:\n   " + queryString + "\nResponse:\n   Invalid Query");
        } else if (response.isEmpty()) {
          responseDoc.put("text", "Query:\n   " + queryString + "\nResponse:\n   " + false);
        } else {
          responseDoc.put("text", "Query:\n   " + queryString + "\nResponse:\n   " + response);
        }
        writeToCollection(responsePath, responseDoc);
      }
    };

    attachCollectionListener(path, beliefQueryListener);
  }

  //foodOrderingSpecific
  //Set up takenMeals listener. When the user presses a relevant button in the webapp, retract the belief associated
  //  with the existence of the completed meal (i.e. a customer 'taking' their order)
  private void attachTakenMealsListener() {
    String path = "groups/" + groupName + "/takenMeals";

    Collectionlistener takenMealsListener = (collectionData) -> {
      for (Map<String, Object> s : collectionData) {
        String queryString = (String) s.get("bindings");
        //TODO: create completed meal Pred from bindings and pass that into retractBelief

        Term queryTerm = Factory.createPredicate(queryString);
        try {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief").argTypes(Term.class)).call(void.class, queryTerm);
        } catch (TRADEException e) {
          log.error("unable to call retractBelief with term: " + queryTerm, e);
        }

        deleteDocument(path + "/" + s.get("docId"));
      }
    };

    attachCollectionListener(path, takenMealsListener);
  }

  //Add listener to navigation backups collection in order to keep notion of backups up to date at all times
  private void attachNavigationBackupListener() {
    String path = "groups/" + groupName + "/navigationBackups";

    Collectionlistener navigationBackupListener = (collectionData) -> {
      List<String> dbBackupNames = new ArrayList<>();
      for (Map<String, Object> s : collectionData) {
        dbBackupNames.add((String) s.get("docId"));
      }
      locationBackupNames = dbBackupNames;
    };

    attachCollectionListener(path, navigationBackupListener);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// End Firebase Listeners ////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void onAgentActionUpdated(String actor, String action, String status) {
    //Check to see if action/step is terminated
    boolean addToHistory = false;
    GoalStatus goalStatus = GoalStatus.fromString(status);
    if (goalStatus != null) {
      if (goalStatus.isTerminated()) {
        addToHistory = true;
      }
    } else {
      ActionStatus actionStatus = ActionStatus.fromString(status);
      if (actionStatus != null && actionStatus.isTerminated()) {
        addToHistory = true;
      }
    }
    //If so add to history
    if (addToHistory) {
      Map<String, Object> docData = new HashMap<>();
      docData.put("goal", action);
      docData.put("actor", actor);
      docData.put("timeComplete", System.currentTimeMillis());
      docData.put("status", status);
      writeToCollection("groups/" + groupName + "/actionHistory/", docData);
    }

    //Modify relevant current entry
    if (action.equals("None")) {
      HashMap<String, Object> goalDoc = new HashMap<>();
      goalDoc.put("goal", "None");
      writeToDocument("groups/" + groupName + "/agentGoals/" + actor, goalDoc);
    } else {
      //Set current
      HashMap<String, Object> goalDoc = new HashMap<>();
      goalDoc.put("goal", action);
      goalDoc.put("action", action);
      goalDoc.put("actor", actor);
      goalDoc.put("gid", System.currentTimeMillis());
      goalDoc.put("status", status);
      writeToDocument("groups/" + groupName + "/agentGoals/" + actor, goalDoc);
    }
  }

  @Override
  public void onBeliefNotificationUpdate(String queryTermString, Map<String, Object> bindingsStrings) {
    writeToDocument("groups/" + groupName + "/beliefNotification/" + queryTermString, bindingsStrings);
  }

  @Override
  public void onActiveGoalsUpdated(List<String> agentStrings, List<String> goalStrings, List<Long> gids, List<String> statuses, List<String> priorityTiers) {
    updateGoalCollectionInfoToFirebase("active", agentStrings, goalStrings, gids, statuses, priorityTiers);
  }

  @Override
  public void onPendingGoalsUpdated(List<String> agentStrings, List<String> goalStrings, List<Long> gids, List<String> statuses, List<String> priorityTiers) {
    updateGoalCollectionInfoToFirebase("pending", agentStrings, goalStrings, gids, statuses, priorityTiers);
  }

  private void updateGoalCollectionInfoToFirebase(String fieldName, List<String> agentStrings, List<String> goalStrings, List<Long> gids, List<String> statuses, List<String> priorityTiers) {
    List<HashMap<String, Object>> goalsInfo = new ArrayList<>();
    for (int i = 0; i < goalStrings.size(); i++) {
      try {
        goalsInfo.add(goalToFirebaseDoc(agentStrings.get(i), goalStrings.get(i), gids.get(i), statuses.get(i), priorityTiers.get(i)));
      } catch (Exception e) {
        log.error("[onPendingGoalsUpdated] exception while iterating over goal queue", e);
      }
    }
    Map<String, Object> pendingGoalsInfoMap = new HashMap<>();
    pendingGoalsInfoMap.put(fieldName, goalsInfo);
    updateDocument("groups/" + groupName + "/systemGoals/goals", pendingGoalsInfoMap);
    //TODO: add listener and log success/failure
  }

  @Override
  public void onActionGenerated(ActionDBEntry action, boolean onStartup) {
    writeASLToFirebase(action, onStartup);
  }

  //  @Override
  //  public void onSystemGoalsUpdated(Map<String,Object> goalInfo) {
  //    writeSystemGoalsToFirebase(goalInfo);
  //  }
  ////Write System level (generally 'self') goals and queue to firebase. Currently only used alongside the QueueExecutionManager
  //@TRADEService
  //public void writeSystemGoalsToFirebase(Map<String,Object> goalInfo) {
  //  log.debug("[writeSystemGoalsToFirebase] in method");
  //  DocumentReference doc = firestore.collection("groups/" + groupName + "/systemGoals/").document("goals");
  //
  //  HashMap<String,Object> totalGoalDoc = new HashMap<>();
  //  totalGoalDoc.put("currentGoal", goalInfo.get("predicate"));
  //  GoalStatus status = (GoalStatus) goalInfo.get("status");
  //  if (status.isTerminated()) {
  //    totalGoalDoc.put("currentGid", System.currentTimeMillis());
  //  } else {
  //    totalGoalDoc.put("currentGid", goalInfo.get("gid"));
  //  }
  //  totalGoalDoc.put("currentStatus", goalInfo.get("status"));
  //  List<HashMap<String, Object>> goalsInQueue = new ArrayList<>();
  //  List<String> goals = (List<String>) goalInfo.get("queueGoals");
  //  List<Long> goalIds = (List<Long>) goalInfo.get("queueGids");
  //  List<GoalStatus> goalsStatus = (List<GoalStatus>) goalInfo.get("queueStatuses");
  //  for (int i=0; i<goals.size(); i++) {
  //    try {
  //      goalsInQueue.add(goalToFirebaseDoc(goals.get(i), goalIds.get(i), goalsStatus.get(i)));
  //    } catch (Exception e) {
  //      log.error("[writeSystemGoalsToFirebase] exception while iterating over goal queue", e);
  //    }
  //  }
  //  totalGoalDoc.put("queue", goalsInQueue);
  //  doc.set(totalGoalDoc);
  //}


  //Helper to create a form good for uploading to firebase
  private HashMap<String, Object> goalToFirebaseDoc(String agentString, String goalString, Long gid, String status, String priorityTier) {
    HashMap<String, Object> goalDoc = new HashMap<>();
    goalDoc.put("agent", agentString);
    goalDoc.put("goal", goalString);
    goalDoc.put("gid", gid);
    goalDoc.put("status", status);
    goalDoc.put("priority", priorityTier);
    return goalDoc;
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// External Component Listener Callbacks Start /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Listener registered with dialogue to receive dialogue history updates to write to firebase for display in external
  // chat components
  @TRADEService
  public void updateFirebaseDialogueHistory(Utterance u) {
    log.debug("[updateFirebaseDialogueHistory] got utterance: " + u);
    HashMap<String, Object> newDialogue = new HashMap<>();
    newDialogue.put("speaker", u.getSpeaker().getName());
    newDialogue.put("listener", u.getListeners().get(0).getName());
    newDialogue.put("time", System.currentTimeMillis());
    newDialogue.put("text", u.getWordsAsString());
    newDialogue.put("language", u.getLanguage());
    Map<String, String> translationsText = new HashMap<>();
    for (String lang : u.getTranslations().keySet()) {
      translationsText.put(lang, String.join(" ", u.getTranslations().get(lang)));
    }
    newDialogue.put("translations", translationsText);
    log.debug("[updateFirebaseDialogueHistory] to db: " + newDialogue);
    try {
      writeToCollection("groups/" + groupName + "/dialogueHistory", newDialogue);
    } catch (Exception e) {
      log.warn("Could not write to dialogue history for utterance " + u, e);
    }
  }

  //Helper method to write contents of an ActionDBEntry to firebase for display on the webapp.
  // @param init Indicate whether this trigger of the callback occurred on startup or later in execution. Used by the
  //             webapp to determine whether to overwrite the current display with this entry (generally desired in the
  //             case of learned or planned actions)
  @TRADEService
  public void writeASLToFirebase(ActionDBEntry e, boolean init) {
    ActionScriptLanguageWriter genericWriter = new ActionScriptLanguageWriter();
    String name = e.getType();
    String aslString = genericWriter.writeAction(e);
    List<Symbol> agents = e.getAgents();
    List<String> agentsStrings = agents.stream().filter(Objects::nonNull).map(Symbol::toString).collect(Collectors.toList());
    log.debug("[writeASLToFirebase] have asl, " + aslString);
    log.debug("[writeASLToFirebase] have actors, " + agentsStrings);

    HashMap<String, Object> aslDoc = new HashMap<>();
    aslDoc.put("asl", aslString);
    aslDoc.put("agents", agentsStrings);
    aslDoc.put("first", !init);
    //TODO: maybe if current goal is not null and action is planned, don't submit? Or at least not as first
    writeToDocument("groups/" + groupName + "/generatedASL/" + name, aslDoc);
  }

  //Write all dictionary keys to firebase to populate dropdown on the webapp
  @Override
  public void onDictionaryEntriesUpdated(Set<String> dictKeys) {
    log.debug("[onDictionaryEntriesUpdated] called");
    HashMap<String, Object> document = new HashMap<>();
    document.put("keyList", new ArrayList<>(dictKeys));

    writeToDocument("groups/" + groupName + "/dictionary/entries", document);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// External Component Listener Callbacks End //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// Methods added from temi app /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  @TRADEService
  public String getGroupName() {
    return groupName;
  }

  @TRADEService
  public boolean writeLocationToFirebase(String locationName, HashMap<String, Object> locationData) {
    return writeToDocument("groups/" + groupName + "/navigation/" + locationName, locationData);
  }

  @TRADEService
  public boolean deleteLocationFromFirebase(String locationName) {
    return deleteDocument("groups/" + groupName + "/navigation/" + locationName);
  }

  @TRADEService
  public boolean saveCurrentLocationsAsBackupSet(String collectionName) {
    log.info("[saveCurrentLocationsAsBackup] Copying location data to navigationBackups under collection: " + collectionName);

    //Seemingly the backup root document needs to be set separately in order for it to be queried?
    //   i.e. setting the child does not automatically allow the parent to be queryable?
    Map<String, Object> backupDocument = new HashMap<>();
    backupDocument.put("name", collectionName);
    if (!writeToDocument("groups/" + groupName + "/navigationBackups/" + collectionName, backupDocument)) {
      log.error("[saveCurrentLocationsAsBackupSet] error creating navigation backup collection root");
      return false;
    }

    List<Map<String, Object>> documents = getCollectionDocumentsData("groups/" + groupName + "/navigation");
    if (documents == null) {
      log.error("[saveCurrentLocationsAsBackupSet] error getting existing location set");
      return false;
    }

    for (Map<String, Object> docData : documents) {
      String docId = (String) docData.remove("docId");
      if (!writeToDocument("groups/" + groupName + "/navigationBackups/" + collectionName + "/locations/" + docId, docData)) {
        log.error("[saveCurrentLocationsAsBackupSet] error copying location " + docId + " to backup collection");
      }
    }

    return true;
  }

  @TRADEService
  public boolean deleteLocationBackupSet(String documentName) {
    return deleteDocument("groups/" + groupName + "/navigationBackups/" + documentName);
  }

  @TRADEService
  public boolean restoreLocationsFromBackup(String documentName) {
    deleteCollection("groups/" + groupName + "/navigation", 3);

    List<Map<String, Object>> locationBackupData = getCollectionDocumentsData("groups/" + groupName + "/navigationBackups/" + documentName + "/locations");
    for (Map<String, Object> locationData : locationBackupData) {
      String docId = (String) locationData.remove("docId");
      if (!writeToDocument("groups/" + groupName + "/navigation/" + docId, locationData)) {
        return false;
      }
    }

    return true;
  }

  @TRADEService
  public List<String> getLocationBackupSetNames() {
    return locationBackupNames;
  }

  @TRADEService
  public boolean deleteVideoFromFirebase(String videoName) {
    String errorMessage = deleteFromStorage(groupName + "/videos/" + videoName);
    if (errorMessage.isEmpty()) {
      log.info("[deleteVideoFromFirebase] deleting video " + videoName + " successful");
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("removeEntry")).call(void.class, videoName.toLowerCase().trim(), "VID", videoName.toLowerCase().trim(), "");
        videoNames.remove(videoName);
        return true;
      } catch (TRADEException e) {
        log.error("[deleteVideoFromFirebase] error calling removeEntry after deleting video " + videoName);
      }
    } else {
      log.warn("[deleteVideoFromFirebase] deleting video " + videoName + " failed");
      if (errorMessage.equals("Object does not exist at location.")) {
        try {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("removeEntry").argTypes(String.class,String.class,String.class,String.class)).call(void.class, videoName.toLowerCase().trim(), "VID", videoName.toLowerCase().trim(), "");
          videoNames.remove(videoName);
          return true;
        } catch (TRADEException e) {
          log.error("[deleteVideoFromFirebase] error calling removeEntry after deleting video " + videoName);
        }
      }
    }
    return false;
  }

  @TRADEService
  public List<String> getFirebaseVideoNames() {
    return videoNames;
  }

  @TRADEService
  public boolean uploadMap(ByteArrayOutputStream map, float width, float height, float originX, float originY, float resolution) {
    Map<String, Object> metadata = new HashMap<>();
    metadata.put("ContentType", "image/png");
    metadata.put("width", String.valueOf(width));
    metadata.put("height", String.valueOf(height));
    metadata.put("originX", String.valueOf(originX));
    metadata.put("originY", String.valueOf(originY));
    metadata.put("resolution", String.valueOf(resolution));
    return writeToStorage(map.toByteArray(), metadata);
  }
}

