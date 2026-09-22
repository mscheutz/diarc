package edu.tufts.hrilab.llm.serviceproviders.tr.social;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.llm.gson.tool.ToolDefinition;
import edu.tufts.hrilab.llm.gson.tool.ToolFunction;
import edu.tufts.hrilab.llm.gson.tool.ToolParameter;
import edu.tufts.hrilab.llm.gson.tool.ToolParameterV2;
import edu.tufts.hrilab.llm.openai.ToolParameterHelper;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.spot.consultant.SpotLocationReference;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

public class SocialSpotLLMServiceProvider extends SocialTRLLMServiceProvider {

  @Override
  public void init() {
    super.init();

    //// init prompt vars
    // Generating string representations of toolset as reference for NLG and scheduleTask (want to present them, but not actually supply them as targets)
    // TODO: move into initPromptVars
    List<Tool> instructTools = generateTaskableInstructTools(false);
    chatTemplates.put("nluextended", new ChatTemplates(getNluSystemPrompt(), instructTools));

    promptVars.put("%instructNLGList", instructTools.stream().map(t -> toolToDescString(t, false)).collect(Collectors.joining("\n#\n")));
    promptVars.put("%instructList", instructTools.stream().map(t -> toolToDescString(t, true)).collect(Collectors.joining("\n#\n")));
    //TODO: Need general way to remove certain actions from the advertised list in specific contexts
    promptVars.put("%taskInstructList", instructTools.stream().filter(t -> !t.toolDefinition.function.name.startsWith("measureWifiAtLocations") && !t.toolDefinition.function.name.startsWith("measureTemperatureAtLocations")).map(t -> toolToDescString(t, true)).collect(Collectors.joining("\n#\n")));
    promptVars.putAll(initPromptVars());
  }

  @Override
  public Map<String,ChatTemplates> generateFocusedContextChatTemplates() {
    Map<String,ChatTemplates> chatTemplates = new HashMap<>();
    // Focused context toolsets:
    // Tour Teaching
    // Using custom "stillGatheringInformation" tool which also explicitly keeps track of the "current template parse" at
    //  each step. The idea is that writing it down each step provides something concretely to validate each step and
    //  better context for incremental LLM population. This could be first application/attempt of "general template
    //  population" structure, which we may want to use for at least general action learning if not every context.
    List<Tool> learnMissionTools = new ArrayList<>();
    LinkedHashMap<String,ToolParameterV2> argMap = new LinkedHashMap<>(Map.of("actionSequence", new ToolParameterV2("array", "The list of actions defining the steps of this mission definition. Execution of this mission will consist of iterating over this list and performing each task. Each individual value in this list should be a json object defining a valid tool call definition matching a form in the supplied actionList.", "nluextended")));
    learnMissionTools.add(new Tool(new ToolDefinition(new ToolFunction(
            "startMission",
            "Submit the details of the fully defined task sequence sequence for storage and future use. Call this when you believe you have all information about the task finalized and are ready to save.",
            argMap,
            new String[]{"actionSequence"})),
            UtteranceType.INSTRUCT
    ));
    LinkedHashMap<String,ToolParameterV2> textResponseArgMap = new LinkedHashMap<>();
    textResponseArgMap.put("response",new ToolParameterV2("string","What you want to say to the user. Generate this argument first."));
    textResponseArgMap.put("currentMissionDefinition",new ToolParameterV2("object","Your current understanding of the steps of the task being taught to you.", argMap,new String[]{"actionSequence"}));
    Tool modifiedTextResponseTool2 = new Tool(new ToolDefinition(new ToolFunction("stillGatheringInformation",
            "You are still being taught new or modifying existing details of the task sequence by the user. You will respond to" +
                    "the user while following the Response Rules and Style Requirements.",
            textResponseArgMap,
            new String[]{"response","currentMissionDefinition"})),
            UtteranceType.INSTRUCT);
    learnMissionTools.add(modifiedTextResponseTool2);
    chatTemplates.put("learnMission", new ChatTemplates(Prompts.getPrompt("focused/learnMission"), learnMissionTools));
    return chatTemplates;
  }

  /**
   * Initialize Prompt variables whose values are static during runtime or updated by event-based listeners
   */
  protected Map<String, String> initDomainPromptVars() {
    Map<String, String> vars = new LinkedHashMap<>();
    // TODO: If Ava components are ever extended to allow runtime saving of these entities, make corresponding updates
    //    to allow for dynamic updating of prompt vars (ideally through subscriber/listener setup)
    List<String> locations = getLocations("self");
    List<String> rooms = getRoomNames();
    String locationList = "";
    if (!locations.isEmpty()) {
      locationList = " -- " + String.join("\n -- ", locations);
      locationList += "\n";
    }
    String roomList = "";
    if (!rooms.isEmpty()) {
      roomList = " -- " + String.join("\n -- ", rooms);
      roomList += "\n";
    }
    vars.put("%locationList", locationList);
    vars.put("%roomList", roomList);
    vars.put("%language", language);
    return vars;
  }

  @Override
  protected Map<String,String> updateDomainPromptVars(Utterance utterance, boolean parsing) {
    return new HashMap<>();
  }

  @Override
  protected String getBaseRoleDescription() {
    return "You are Spot, a Boston Dynamics Spot robot with an associated back-end system.";
  }

  //TODO: believe getBaseContextualInformation should be overriden and include locations

  @Override
  protected List<String> getRefTypes() {
    return List.of("location");
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// Should be domain specific

  // TODO: Revisit old error cases and explicitly handle if finding they are still possible
  // Modify semantics based on domain specific postprocessing
  // Return - Error String to add onto baseToolResponse to be sent when hanlding this uetterance as an error.
  //          Empty String if no issues found
  public String correctCommonNLUMistakes(Utterance utterance) {
    Symbol actor = utterance.getAddressee();
    try {
      String command = utterance.getSemantics().getName();
      if (command.equals("getActionSteps")) {
        //Not currently prompted to do this
        if (utterance.getSemantics().hasArgs()) {
          Symbol innerArg = ((Term)utterance.getSemantics()).get(1);
          List<Symbol> argsWithActor = new ArrayList<>();
          argsWithActor.add(actor);
          if (innerArg.isTerm()) {
            argsWithActor.addAll(((Term) ((Term) utterance.getSemantics()).get(1)).getArgs());
          }
          utterance.setSemantics(Factory.createPredicate(command, actor, Factory.createPredicate(innerArg.getName(), argsWithActor)));
        }
      } else if (command.equals("submitScheduledTaskDefinition")) {
        List<Symbol> args = new ArrayList<>();
        args.add(null);
        args.add(null);
        args.add(null);
        args.add(null);
        StringBuilder errorStringBuilder = new StringBuilder();
        boolean hasStart = false;
        boolean hasPeriod = false;
        boolean hasEnd = false;
        for (Symbol arg : ((Predicate)utterance.getSemantics()).getArgs()) {
          //Need to remove actor from scheduled predicate - will be filtered out by default
          if (arg.isPredicate()) {
            Predicate argPred = (Predicate) arg;
            if (arg.getName().equals("start")) {
              if (argPred.hasArgs() && !argPred.get(0).getName().equals("immediately")) {
                if (argPred.size() == 1) {
                  // "start(date-string)"
                  if (ScheduledGoal.parseDateString(argPred.get(0).toString()) == null) {
                    errorStringBuilder.append("- The start value of the submitted scheduled task definition could not be parsed. It must be the exact value start(immediately) or contain a date string with the following form start(HH-mm-ss). ").append(argPred.get(0).toString()).append("was not detected to follow these rules, please update the supplied format.");
                  }
                } else if (argPred.size() == 2) {
                  // "start(num, time-unit)"
                  try {
                    long delay = Long.parseLong(argPred.get(0).toString());
                  } catch (NumberFormatException e) {
                    errorStringBuilder.append("- The start value of the submitted scheduled task definition could not be parsed. A relative time must contain a valid integer and a TimeUnit ").append(argPred.get(0).toString()).append("was not detected as a valid integer, please fix your supplied start time.");
                  }
                  try {
                    TimeUnit millis = TimeUnit.valueOf(argPred.get(1).toString().toUpperCase());
                  } catch (IllegalArgumentException e) {
                    errorStringBuilder.append("- The start value of the submitted scheduled task definition could not be parsed. A relative time must contain a valid integer and a TimeUnit ").append(argPred.get(1).toString()).append("was not detected as a valid Java TimeUnit, please fix your supplied start time.");
                  }
                } else {
                  errorStringBuilder.append("- The start value of the submitted scheduled task definition could not be parsed.").append(argPred.get(0).toString()).append("was not detected to follow the defined rules for this argument, please update the supplied format.");
                }
                hasStart = true;
                args.add(1,arg);
              }
            } else if (arg.getName().equals("repeat")) {
              if (argPred.hasArgs() && !argPred.get(0).getName().equals("nonrepeating")) {
                if (argPred.size() == 2) {
                  // "period(num, time-unit)"
                  try {
                    long delay = Long.parseLong(argPred.get(0).toString());
                  } catch (NumberFormatException e) {
                    errorStringBuilder.append("- The repeat value of the submitted scheduled task definition could not be parsed. A relative time must contain a valid integer and a TimeUnit ").append(argPred.get(0).toString()).append("was not detected as a valid integer, please fix your supplied start time.");
                  }
                  try {
                    TimeUnit millis = TimeUnit.valueOf(argPred.get(1).toString().toUpperCase());
                  } catch (IllegalArgumentException e) {
                    errorStringBuilder.append("- The repeat value of the submitted scheduled task definition could not be parsed. A relative time must contain a valid integer and a TimeUnit ").append(argPred.get(1).toString()).append("was not detected as a valid Java TimeUnit, please fix your supplied start time.");
                  }
                } else {
                  errorStringBuilder.append("- The repeat value of the submitted scheduled task definition could not be parsed.").append(argPred.get(0).toString()).append("was not detected to follow the defined rules for this argument, please update the supplied format.");
                }
                hasPeriod = true;
                arg = Factory.createPredicate("period",argPred.getArgs());
                args.add(2,arg);
              }
            } else if (arg.getName().equals("end")) {
              if (argPred.hasArgs() && !argPred.get(0).getName().equals("indefinitely") && !argPred.get(0).getName().equals("nonrepeating")) {
                if (argPred.size() == 1) {
                  // "start(date-string)"
                  if (ScheduledGoal.parseDateString(argPred.get(0).toString()) == null) {
                    errorStringBuilder.append("- The end value of the submitted scheduled task definition could not be parsed. It must be the exact value end(nonrepeating), end(indefinitely), or contain a date string with the following form end(HH-mm-ss). ").append(argPred.get(0).toString()).append("was not detected to follow these rules, please update the supplied format.");
                  }
                } else if (argPred.size() == 2) {
                  // "start(num, time-unit)"
                  try {
                    long delay = Long.parseLong(argPred.get(0).toString());
                  } catch (NumberFormatException e) {
                    errorStringBuilder.append("- The end value of the submitted scheduled task definition could not be parsed. A relative time must contain a valid integer and a TimeUnit ").append(argPred.get(0).toString()).append("was not detected as a valid integer, please fix your supplied start time.");
                  }
                  try {
                    TimeUnit millis = TimeUnit.valueOf(argPred.get(1).toString().toUpperCase());
                  } catch (IllegalArgumentException e) {
                    errorStringBuilder.append("- The end value of the submitted scheduled task definition could not be parsed. A relative time must contain a valid integer and a TimeUnit ").append(argPred.get(1).toString()).append("was not detected as a valid Java TimeUnit, please fix your supplied start time.");
                  }
                } else {
                  errorStringBuilder.append("- The end value of the submitted scheduled task definition could not be parsed.").append(argPred.get(0).toString()).append("was not detected to follow the defined rules for this argument, please update the supplied format.");
                }
                hasEnd = true;
                args.add(3,arg);
              }
            }
            else {
              args.add(0,arg);
            }
          }
        }
        if (!hasStart && !hasPeriod) {
          errorStringBuilder.append("- The submitted scheduled task definition must have a future start time or an interval to repeat on. Scheduled tasks may not occur immediately and without repeating on some interval. Let the user know this fact and ask for one of these values.");
        } else if (hasEnd && !hasPeriod) {
          errorStringBuilder.append("- The submitted scheduled task definition contains an explicit end time, but no defined interval to repeat on. Either discard the end time if the task is not meant to repeat, or fill out the repeat interval appropriately. If you cannot do so on your own, you may let the user know a properly defined execution interval is necessary.");
        }

        if (errorStringBuilder.isEmpty()) {
          args.removeIf(Objects::isNull);
          utterance.setSemantics(Factory.createPredicate("scheduled", args));
        } else {
          return errorStringBuilder.toString();
        }
        //} else if (command.equals("startMission")) {
        //  List<Symbol> args = ((Predicate)utterance.getSemantics()).getArgs();
        //  if (args.size() != 3) {
        //    log.error("[correctCommonNLUMistakes] tourDefinition has unexpected number of arguments, this should have been caught in tool processing?");
        //  } else {
        //    String tourName = args.get(1).toString();
        //    if (!tourName.matches("[A-Za-z0-9_]+")) {
        //      Symbol cleanedName = Factory.createSymbol(tourName.trim().replaceAll("\\s+", "_").replaceAll("[^a-zA-Z0-9_]", ""));
        //      args.remove(1);
        //      args.add(1, cleanedName);
        //      utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
        //    }
        //  }
      } else if (command.equals("checkPastInteractionHistoryForMissingInformation")) {
        Predicate semantics = (Predicate) utterance.getSemantics();
        List<Symbol> args = semantics.getArgs();
        args.add(utterance.getSpeaker());
        args.add(utterance.getAddressee());
        utterance.setSemantics(Factory.createPredicate(semantics.getName(), args));
      }
    }
    catch (Exception e) {
      log.warn("[correctCommonNLUMistakes] unexpected semantics format {}", utterance.getSemantics());
      //TODO: explicitly fail with a specific message here? Or just return unmodified utterance?
    }
    return "";
  }

  private String getMatchingNameIgnoreCase(String query, List<String> targets) {
    for (String target : targets) {
      if (query.equalsIgnoreCase(target)) {
        return target;
      }
    }
    return "";
  }

  //TODO: calls to this/getMapReferences wildly inefficient, fix
  private List<String> getLocations(String actorName) {
    List<String> locNames = new ArrayList<>();
    Map<Symbol, PoseReference> refs = new HashMap<>();
    //TODO: should be based on group
    try {
      refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("getActivatedEntities").argTypes(List.class)).call(Map.class, List.of("location"));
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getReference").argTypes(Symbol.class).inGroups("location"));
      for (Symbol refId : refs.keySet()) {
        SpotLocationReference ref = tsi.call(SpotLocationReference.class, refId);
        locNames.add(ref.properties.get(0).getName()+":location");
      }
    } catch (TRADEException e) {
      log.error("[getLocations] ",e);
    }

    return locNames;

    //try {
    //  Map<Symbol, PoseReference> refs = new HashMap<>();
    //  //TODO: should be based on group
    //  TRADEServiceInfo tsi = getServiceForAgent(actorName, "getMapReferences");
    //  //TODO: resolver issue? Sometimes location list is empty?
    //  int maxAttempts = 3;
    //  for (int i=0; i<maxAttempts;i++) {
    //    if (tsi != null) {
    //      refs = tsi.call(Map.class);
    //    }
    //    if (refs != null && !refs.isEmpty()) {
    //      return refs.values().stream().map(ref -> {
    //        if (!ref.properties.isEmpty()) {
    //          return ref.properties.get(0).getName() + ":location";
    //        } else {
    //          return "";
    //        }
    //      }).toList();
    //    }
    //  }
    //} catch (TRADEException e) {
    //  log.error("[getLocations]", e);
    //}

    //return new ArrayList<>();
  }

  @TRADEService
  public List<String> getRoomNames() {
    List<String> roomNames = new ArrayList<>();
    Predicate query = Factory.createPredicate("inRoom","X","Y");
    List<Map<Variable, Symbol>> bindings = new ArrayList<>();
    try {
      //TODO: race conditions on room names fully being asserted to belief
      while (bindings.size() != 3) {
        bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class, query);
      }
      for (Map<Variable, Symbol> binding : bindings) {
        Symbol name = binding.get(new Variable("Y"));
        if (!roomNames.contains(name.getName())) {
          roomNames.add(name.getName());
        }
      }
    } catch (TRADEException e) {
      log.error("[getRoomNames] ", e);
    }
    return roomNames;

    //return List.of("main lab space:room", "experiment room:room");
  }

//  private String getWorldStateString(Symbol agent) {
//    Predicate query = Factory.createPredicate("at","X","Y");
//    List<Map<Variable, Symbol>> bindings;
//    String robotLocationString = "";
//    try {
//      bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class,query);
//      TRADEServiceInfo tsi = getServiceForAgent("self", "getMapReferences");
//      Map<Symbol, PoseReference> refs = new HashMap<>();
//      if (tsi != null) {
//        refs = tsi.call(Map.class);
//      }
//      if (refs != null && bindings != null) {
//        for (Map<Variable, Symbol> binding : bindings) {
//          Symbol y = binding.get(new Variable("Y"));
//          //TODO: should be based on group
//          PoseReference ref = refs.get(y);
//          if (ref != null) {
//            if (!ref.properties.isEmpty()) {
//              robotLocationString = String.format("You were last at the following location: %s.\n", ref.properties.get(0).getName());
//            }
//          }
//        }
//      }
//    } catch (TRADEException e) {
//      log.error("[getWorldState] ", e);
//      //return "Nothing is known by the robot";
//      return "You do not know where you are";
//    }
//
//    String output = robotLocationString;
//    if (output.isEmpty()) {
//      log.warn("[getWorldState] no matching beliefs found for agent {}", agent);
//      //output = "<Nothing is known by the robot>";
//      output = "You do not know where you are";
//    }
//    return output;
//  }

  TRADEServiceInfo getServiceForAgent(String agentName, String serviceName, Class<?>... argTypes) {
    HashSet<TRADEServiceInfo> services = (HashSet<TRADEServiceInfo>) TRADE.getAvailableServices(new TRADEServiceConstraints().name(serviceName).argTypes(argTypes));
    for (TRADEServiceInfo tsi : services) {
      Collection<String> groups = tsi.getGroups();
      if (agentName.equals("self") && groups.isEmpty()) {
        return tsi;
      } else {
        for (String group : groups) {
          if (group.contains(agentName)) {
            return tsi;
          }
        }
      }
    }
    log.warn("[getServiceForAgent] could not find service {} for agent {}", serviceName, agentName);
    return null;
  }

    /**
     * Generate the base INSTRUCT UtteranceType toolset available for this domain
     */
  //TODO: Apply prompt variables dynamically (e.g. update location lists)
  //TODO: Think of a better way to define / gather this information
  //TODO: rephrase descriptions to align better with final "social" system prompt
  public List<Tool> generateInstructTools(boolean xml) {
    List<Tool> instructTools = new ArrayList<>();
    ToolDefinition definition;
    List<String> locationNames = getLocations("self");

    definition = new ToolDefinition(new ToolFunction("cancelCurrentTask",
            "You stop and fully discard your current task. \nExample Instruction: \"cancel\" or \"stop pursuing your current task\""));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition(new ToolFunction("cancelAllQueuedTasks",
            "You cancel all goals currently in the pending queue. This does not include separately scheduled goals. \nExample Instruction: \"cancel all upcoming tasks.\""));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition(new ToolFunction("cancelAllCurrentAndQueuedTasks",
            "You cancel all currently assigned goals. This includes the current active goal and all pending goals which have yet to begin execution. \nExample Instruction: \"cancel all tasks.\""));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    String exampleCallString = xml ?
            "<function=cancelSingleTaskInQueue><parameter=index>0</parameter></function>" :
            "{\"name\":\"cancelSingleTaskInQueue\", \"arguments\":{\"index\":0}}";
    definition = new ToolDefinition(new ToolFunction("cancelSingleTaskInQueue",
            "You cancel and remove a task in the task queue specified by its index. \nExample Instruction: \"Remove the first task from the queue.\" \nExample Call: " + exampleCallString,
            new LinkedHashMap<>(Map.of("index",new ToolParameterV2("integer","The 0-based index of the task in the queue to remove"))),new String[]{"index"}));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition(new ToolFunction("suspendCurrentTask",
            "You pause your current task, which can be later resumed. \nExample Instruction: \"stop\" or \"pause what you're doing.\""));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition(new ToolFunction("resumeCurrentTask",
            "You resume the currently paused task. \nExample Instruction: \"resume previous task.\""));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

    exampleCallString = xml ?
            "<function=urgentPriority><parameter=actionPredicate>{\"name\":\"goToLocation\",\"arguments\":{\"destination\":\""+promptVars.get("%exampleLocation")+"\"}}</parameter></function>" :
            "{\"name\":\"urgentPriority\", \"arguments\":{\"actionPredicate\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\""+promptVars.get("%exampleLocation")+"\"}}}}";
    definition = new ToolDefinition(new ToolFunction("urgentPriority",
            "You submit an action with urgent priority. This means that this action will interrupt execution of any other currently submitted actions and be immediately prioritized. \nExample Instruction: \"go to "+promptVars.get("%exampleLocation")+" right now.\" \nExample Call: " + exampleCallString,
            new LinkedHashMap<>(Map.of("actionPredicate",new ToolParameterV2("object","Populated JSON of another tool call representing the action to submit with urgent priority. This json minimally needs \"name\" and \"argument\" fields, like shown in the example.", "nlu"))),
            new String[]{"actionPredicate"}));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

    definition = new ToolDefinition(new ToolFunction("dontRespond",
            "You do not verbally respond. \nExample Instruction: \"shut up.\""));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

    definition = new ToolDefinition(new ToolFunction("learnMission",
            "You begin a dialogue interaction with the user to learn how to execute a newly assigned mission. The mission will consist of navigating between a set of saved locations while performing actions at each location. \nExample Instruction: \"I have a new mission ready for you\" or the user beginning to define an action sequence for you to save"));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

    instructTools.addAll(generateTaskableInstructTools(true));

    return instructTools;
  }

  public List<Tool> generateTaskableInstructTools(boolean xml) {
    List<Tool> instructTools = new ArrayList<>();
    ToolDefinition definition;
    String[] locationNames = getLocations("self").toArray(new String[0]);


    String exampleCallString = xml ?
            "<function=goToLocationWrapper><parameter=location>exampleLocation:location</parameter></function>." :
            "{\"name\":\"goToLocationWrapper\", \"arguments\":{\"location\":\"exampleLocation:location\"}}.";
    definition = new ToolDefinition(new ToolFunction("goToLocationWrapper",
            "You go to the supplied destination location.\n- Example instruction: \"please navigate to exampleLocation\" \n- Example Call: " + exampleCallString,
            new LinkedHashMap<>(Map.of("location",new ToolParameterV2("string","The destination :location to go to. ",locationNames,false))),new String[]{"location"}));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

    //This action shouldn't exist
    definition = new ToolDefinition(new ToolFunction("goToRoom",
            "You go to the supplied destination room.",
            new LinkedHashMap<>(Map.of("room", new ToolParameterV2("string", "The destination room to go to", getRoomNames().toArray(new String[0]), true))), new String[]{"room"}));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //TODO: could also turn off validation for this? (only for local user?)
    exampleCallString = xml ?
            "<function=sayLocally><parameter=words>\"welcome to the event!\"</parameter></function>\"" :
            "{\"name\":\"say\", \"arguments\":{\"words\":\"welcome to the event\"}}";
    definition = new ToolDefinition(new ToolFunction("sayLocally",
            "The speaker on the robot will emit the supplied phrase. Do not use this when you want to respond to the user. This should be targeted when a user is explicitly instructing you to say something out loud in person, most commonly in the context of another task, to another person who is not the user, or at a later time. \n- Example instruction: \"say to everyone: welcome to the event!.\" \n- Example Call: " + exampleCallString,
            new LinkedHashMap<>(Map.of("words",new ToolParameterV2("string","The phrase to be said. This should be exactly what the user instructed you to say, not paraphrasing."))),new String[]{"words"}));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //exampleCallString = xml ?
    //         "<function=escortToLocation><parameter=name>owen</parameter><parameter=destination>%exampleLocation</parameter></function>" :
    //        "{\"name\":\"escortToLocation\", \"arguments\":{\"name\":\"owen\",\"destination\":\"%exampleLocation\"}}";
    //definition = new ToolDefinition("escortToLocation",
    //        "You lead someone from your current location to a different one. \nExample Instruction: \"bring Owen to %exampleLocation.\" \nExample Call: " + exampleCallString,
    //        List.of(new ToolParameterHelper("name", "string", new ArrayList(), "Name of the entity to escort", true),
    //                new ToolParameterHelper("destination", "string", locationNames, "The destination location to escort to. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)
    //        ));
    //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //TODO: don't clog LLM with this, but still allow ability to hit in AvaHybridParser
    //definition = new ToolDefinition("goHome",
    //        "You go to your default charging dock and charge. \nExample Instruction: \"go back to charge.\"",
    //        new ArrayList());
    //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

    //TODO: maybe remove
    exampleCallString = xml ?
            "<function=waitForDuration><parameter=seconds>60</parameter></function>":
            "{\"name\":\"waitForDuration\", \"arguments\":{\"seconds\":\"60\"}}";
    definition = new ToolDefinition(new ToolFunction("waitForDuration",
            "You wait idly for a number of seconds. \nExample Instruction: \"Then wait for a minute.\" \nExample Call: " + exampleCallString,
            new LinkedHashMap<>(Map.of("seconds",new ToolParameterV2("integer","The amount of seconds to wait. Must be a positive whole number"))),new String[]{"seconds"}));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

    exampleCallString = xml ?
            "<function=searchRoomForPeople><parameter=room>?room</parameter></function>." :
            "{\"name\":\"searchRoomForPeople\", \"arguments\":{\"room\":\"?room\"}}.";
    definition = new ToolDefinition(new ToolFunction("searchRoomForPeople",
            "You sweep a room for people by scanning for people at all associated room waypoints\n- Example instruction: \"look for civilians in ?room\" \n- Example Call: " + exampleCallString,
            new LinkedHashMap<>(Map.of("room",new ToolParameterV2("string","The room to sweep. Accepted values are only those found in the Room List.",getRoomNames().toArray(new String[0]),false))),new String[]{"room"}));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition(new ToolFunction("scanForPeople",
            "Use your camera to scan the area in front of you for people"));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    exampleCallString = xml ?
            "<function=clearRoomForDoor><parameter=destination>?room</parameter></function>." :
            "{\"name\":\"clearRoomForDoor\", \"arguments\":{\"destination\":\"?room\"}}.";
    definition = new ToolDefinition(new ToolFunction("clearRoomForDoor",
            "You sweep a room for unknown doors by scanning for doors at all associated room waypoints\n- Example instruction: \"Are there any doors in ?room\" \n- Example Call: " + exampleCallString,
            new LinkedHashMap<>(Map.of("room",new ToolParameterV2("string","The room to sweep. Accepted values are only those found in the Room List.",getRoomNames().toArray(new String[0]),false))),new String[]{"room"}));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition(new ToolFunction("scanForDoor",
            "Use your camera to scan the area in front of you for a door"));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //What arguments will this take in / preconditions required?
    definition = new ToolDefinition(new ToolFunction("breakOpenDoor",
            "You open the previously detected door in front of you"));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition(new ToolFunction("escortCivilianToSafety",
            "You escort civilians to safety",
            new LinkedHashMap<>(Map.of("destination",new ToolParameterV2("string","The destination location to escort the civilian back to.",locationNames,false))),new String[]{"destination"}));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //approachDoor?

    //QoL
    //  goToPose?
    return instructTools;
  }

  /**
   * Generate the base QUESTION UtteranceType toolset available for this domain.
   * TODO: Currently a stand-in to define DIARC-LLM ITK tools, not user-DIARC ITK. Need to update how we're defining
   *        this once the ITK pipeline is safe to use
   */
  //TODO: add more itk tools
  //  - extended dialogue history
  //  - other agent dialogue history
  //  - goal state rather than put directly in prompts?
  //  - ...
  //TODO: need to force the LLM to call these tools to get an updated notion of dynamic information when relevant (rather than use old results)
  //TODO: Rewrite Action related ITK - have a lot of the information in the system prompt already + don't have learned actions integrated yet
  public List<Tool> generateITKTools() {
    List<Tool> itkTools = new ArrayList<>();
    ToolDefinition definition;

    //definition = new ToolDefinition("getCurrentLocationString",
    //        "Use this to ask the system what saved location on your map you believe you are currently at. You may not be localized and not know where you are. An example of when you may need this information is when a user asks \"what is your current location.\"",
    //        new ArrayList());
    //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    //definition = new ToolDefinition("getAllKnownLocations",
    //        "Use this to ask the system for a list of all currently saved locations. An example of when you may need this information is when a user asks \"what are your saved locations.\"",
    //        new ArrayList());
    //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition(new ToolFunction("checkPastInteractionHistoryForMissingInformation",
            "Use this when you believe you are missing important context necessary to understand user input. For example, when the user explicitly references a past conversation or says something implying common ground you have no memory of."));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));

    definition = new ToolDefinition(new ToolFunction("getCurrentLocationString",
            "Use this to ask the system what room and location on your saved map you believe you are currently at. Use the result to inform answers to any relevant user questions. You may not be localized and not know where you are. An example of when you may need this information is when a user asks \"what is your current location.\""));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition(new ToolFunction("getAllKnownRooms",
            "Use this to ask the system what areas you know of in your currently saved map. Use the result to inform answers to any relevant user questions. An example of when you may need this information is when a user asks \"what areas are you aware of.\""));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));

    //// Nice util, but not for demo
    //definition = new ToolDefinition("getBatteryLevelString",
    //        "Use this to ask the system what the current battery level of the robot as a percentage is. An example of when you may need this information is when a user asks \"how much charge do you have.\"",
    //        new ArrayList());
    //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    //String exampleCallString = xml ?
    //         "<function=getActionSteps><parameter=action>escort</parameter></function>" :
    //"{\"name\":\"getActionSteps\", \"arguments\":{\"action\":\"escort\"}}";
    //definition = new ToolDefinition("getActionSteps",
    //        "Use this to ask the system for a description of an action, in the form of the list the steps which would be performed when it is executed. ?action can be replaced by populated JSON representing another tool OR be just the name of another tool if no specific arguments are supplied. An example of when you may need this information is when a user asks \"How would you escort someone.\" \nExample Call: " + exampleCallString,
    //        List.of(new ToolParameterHelper("action", "string", new ArrayList(), "The tool whose execution steps to describe", true)));
    //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    //definition = new ToolDefinition("getTaughtActionsList",
    //        "Use this to ask the system for a list of all of the actions which have been taught to you by a user. An example of when you may need this information is when a user asks \"What actions have you been taught.\"",
    //        new ArrayList());
    //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    return itkTools;
  }

  /**
   * Generate all possible REPLY UtteranceType tools available for this domain. Will be included in any LLM Completion
   * request dynamically based on matching expected response forms for current system questions.
   */
  public Map<String,Tool> generateReplyTools() {
    Map<String,Tool> replyTools = new HashMap();
    ToolDefinition definition;
    List<String> locationNames = new ArrayList<>(getLocations("self"));
    locationNames.add("none");

    //TODO: Move some general targets to base class
    definition = new ToolDefinition(new ToolFunction("abort",
            "You inform the system that the user is explicitly abandoning answering its current OUTSTANDING QUESTION",
            new LinkedHashMap<>(Map.of(
                    "target",
                    new ToolParameterV2("string","",new String[]{"question"},true)
            )),
            new String[]{"target"}
    ));
    replyTools.put("abort",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("abort()"), new Tool(definition,UtteranceType.REPLY));
    //TODO: should val still be included?
    definition = new ToolDefinition(new ToolFunction("val",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION with a yes/no answer.",
            new LinkedHashMap<>(Map.of(
                    "response",
                    new ToolParameterV2("string","Whether the user answered affirmatively or negatively",new String[]{"yes","no"},true)
            )),
            new String[]{"response"}
    ));
    replyTools.put("val",new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition(new ToolFunction("validated",
            "You inform the system that the user has answered its current outstanding validation question.",
            new LinkedHashMap<>(Map.of(
                    "response",
                    new ToolParameterV2("string","The response value should be yes when the user has validated your interpretation of the their instruction, the response value should be no when the user denies your interpretation of their instruction or adds clarifying or additional details to your interpretation",new String[]{"yes","no"},true)
            )),
            new String[]{"response"}
    ));
    replyTools.put("validated",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("val(X)"), new Tool(definition,UtteranceType.REPLY));

    //TODO: are any of these ever necessary? (doesn't matter too much because if no Q is asked with them as a possible response form, then they will never be inserted. But would be nice for cleanup)
    definition = new ToolDefinition(new ToolFunction("location",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION by specifying a certain saved location",
            new LinkedHashMap<>(Map.of(
                    "name",
                    new ToolParameterV2("string","The specified :location. \"none\" is the one special value, indicating the user is done supplying location names")
            )),
            new String[]{"name"}
    ));
    replyTools.put("location",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("location(X:location)"), new Tool(definition,UtteranceType.REPLY));
    //String exampleCallString = xml ?
    //      "<function=taskPredicate><parameter=toolCall><function=goToLocation><parameter=destination>"+promptVars.get("%exampleLocation")+"</parameter></function></parameter></function>" :
    //      "{\"name\":\"taskPredicate\", \"arguments\":{\"toolCall\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\""+promptVars.get("%exampleLocation")+"\"}}}}";
    String exampleCallString = "<function=taskPredicate><parameter=toolCall><function=goToLocation><parameter=destination>"+promptVars.get("%exampleLocation")+"</parameter></function></parameter></function>";
    definition = new ToolDefinition(new ToolFunction("taskPredicate",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of another tool call. \nExample Instruction: \"the task is to go to "+promptVars.get("%exampleLocation")+"\" \nExample Call: " + exampleCallString,
            new LinkedHashMap<>(Map.of(
                    "toolCall",
                    new ToolParameterV2("object","This argument should be replaced by definition of another tool call and populated arguments from the previous examples", "nlu")
            )),
            new String[]{"toolCall"}
    ));
    replyTools.put("taskPredicate",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("taskPredicate(X)"), new Tool(definition,UtteranceType.REPLY));
    //exampleCallString = xml ?
    //      "<function=relativeTime><parameter=duration>2</parameter><parameter=timeunit>HOURS</parameter></function>" :
    //      "{\"name\":\"relativeTime\", \"arguments\":{\"duration\":\"2\",\"timeunit\":\"HOURS\"}}";
    exampleCallString = "<function=relativeTime><parameter=duration>2</parameter><parameter=timeunit>HOURS</parameter></function>";
    LinkedHashMap<String,ToolParameterV2> argMap = new LinkedHashMap<>();
    argMap.put("duration",new ToolParameterV2("number","an integer, the duration of the specified time"));
    argMap.put("timeUnit",new ToolParameterV2("string","the duration's corresponding timeunit", Arrays.stream(TimeUnit.values()).map(TimeUnit::toString).toArray(String[]::new),true));
    definition = new ToolDefinition(new ToolFunction("relativeTime",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of a time relative to now. \nExample Instruction: \"in two hours\" \nExample Call: " + exampleCallString,
            argMap,
            new String[]{"duration","timeUnit"}
    ));
    replyTools.put("relativeTime",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("relativeTime(X,Y)"), new Tool(definition,UtteranceType.REPLY));
    //exampleCallString = xml ?
    //      "<function=absoluteTime><parameter=timeOfDay>12:30</parameter><parameter=partOfDay>pm</parameter></function>" :
    //      "{\"name\":\"absoluteTime\", \"arguments\":{\"timeOfDay\":\"12:30\",\"partOfDay\":\"pm\"}}";
    exampleCallString = "<function=absoluteTime><parameter=timeOfDay>12:30</parameter><parameter=partOfDay>pm</parameter></function>";
    argMap = new LinkedHashMap<>();
    argMap.put("timeOfDay",new ToolParameterV2("string","The time of day in the format hour:minute"));
    argMap.put("partOfDay",new ToolParameterV2("string","", new String[]{"am","pm"},true));
    definition = new ToolDefinition(new ToolFunction("absoluteTime",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of an absolute time. \nExample Instruction: \"half past noon\" \nExample Call: " + exampleCallString,
            argMap,
            new String[]{"timeOfDay","partOfDay"}
    ));
    replyTools.put("absoluteTime",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("absoluteTime(X,Y)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition(new ToolFunction("timingOf",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of one of the aspects of execution timing of a repeated task. Example instructions and corresponding argument values could be \"when it first happens\" -> timingOf(execution) or \"the duration between when it happens\" -> timingOf(interval)",
            new LinkedHashMap<>(Map.of(
                    "timingType",
                    new ToolParameterV2("string","execution references when something occurs and interval references the amount of time between something occurring.", new String[]{"execution","interval"},true)
            )),
            new String[]{"timingType"}
    ));
    replyTools.put("timingOf",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("timingOf(X)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition(new ToolFunction("taskName",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of the name of a task to be learned",
            new LinkedHashMap<>(Map.of(
                    "name",
                    new ToolParameterV2("string","")
            )),
            new String[]{"name"}
    ));
    replyTools.put("taskName",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("taskName(X)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition(new ToolFunction("option",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through selection of a presented option.",
            new LinkedHashMap<>(Map.of(
                    "number",
                    new ToolParameterV2("number","the number of the chosen option from the previously presented list")
            )),
            new String[]{"number"}
    ));
    replyTools.put("option",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("option(X)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition(new ToolFunction("actionType",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through selection of a type of action. Example instructions and corresponding argument values could be \"the taught actions\" -> actionType(taught) or \"the ones you already knew\" -> actionType(predefined) or \"all of them\" -> actionType(both)",
            new LinkedHashMap<>(Map.of(
                    "type",
                    new ToolParameterV2("string","How the system came to know about the discussed action. Taught references actions tuaght by the user, predefined references actions known automatically by the system on startup, and both references all actions.", new String[]{"taught","predefined","both"},true)
            )),
            new String[]{"type"}
    ));
    replyTools.put("actionType",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("actionType(X)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition(new ToolFunction("amount",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of an amount.",
            new LinkedHashMap<>(Map.of(
                    "number",
                    new ToolParameterV2("number","The only acceptable value is either an integer or \"all\". Where \"all\" is referring the maximum amount of something")
            )),
            new String[]{"number"}
    ));
    replyTools.put("amount",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("amount(X)"), new Tool(definition,UtteranceType.REPLY));

    return replyTools;
  }
}