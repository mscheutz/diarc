package edu.tufts.hrilab.llm.serviceproviders.tr.social;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.db.DBEntry;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.llm.gson.tool.ToolDefinition;
import edu.tufts.hrilab.llm.gson.tool.ToolFunction;
import edu.tufts.hrilab.llm.gson.tool.ToolParameterV2;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;

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
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

public class SocialAvaLLMServiceProvider extends SocialTRLLMServiceProvider {
   // Temperature measurements and reporting
   private boolean includeDemoTargets = false;

   @Override
   public void init() {
      super.init();

      try {
         TRADE.requestNotification(this, "joined", new TRADEServiceConstraints().name("getKBName"),null, "newConsultantCallback");
      } catch (TRADEException e) {
         log.error("[init] error registering for TRADE notification for new consultant join");
      }

      //// init prompt vars
      // Generating string representations of toolset as reference for NLG and startSchedulingTask (want to present them, but not actually supply them as targets)
      // TODO: move into initPromptVars
      List<Tool> instructTools = generateInstructTools(false);
      //TODO: need general way to provide instruct versions of actions for teaching, scheduling, etc vs ITK versions of
      //  actions for general interaction to populate corresponding toolsets effectively
      LinkedHashMap<String,ToolParameterV2> argMap = new LinkedHashMap<>();
      argMap.put("locations", new ToolParameterV2("array","The list of :locations to report the wifi signal of. Empty if the current location at the time of this goal being performed",getLocations("self",true).toArray(new String[0]),false));
      ToolDefinition definition = new ToolDefinition(new ToolFunction("reportPreviouslyMeasuredWifiOfLocations",
              "You report the wifi signal strength you had previously measured at certain :locations",
              argMap,
              new String[]{"locations"}));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      if (includeDemoTargets) {
         argMap = new LinkedHashMap<>();
         argMap.put("locations",new ToolParameterV2("array","The list of :locations to report the temperature of. Empty if the current location at the time of this goal being performed",getLocations("self", true).toArray(new String[0]),false));
         definition = new ToolDefinition(new ToolFunction("reportPreviouslyMeasuredTemperatureOfLocations",
                 "You report the temperature you had previously measured at certain :locations",
                 argMap,
                 new String[]{"locations"}));
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      }
      argMap = new LinkedHashMap<>();
      argMap.put("words",new ToolParameterV2("string","The exact phrase instructed to be said"));
      definition = new ToolDefinition(new ToolFunction("sayLocally",
              "The speaker on the robot will emit the supplied phrase",
              argMap,
              new String[]{"words"}));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition(new ToolFunction("stopFollowing",
              "You stop following the user"));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      chatTemplates.put("nluextended", new ChatTemplates(getNluSystemPrompt(), instructTools));

      promptVars.put("%instructNLGList", instructTools.stream().map(t -> toolToDescString(t, false)).collect(Collectors.joining("\n#\n")));
      promptVars.put("%instructList", instructTools.stream().map(t -> toolToDescString(t, true)).collect(Collectors.joining("\n#\n")));
      promptVars.put("%scheduleInstructList", instructTools.stream().map(t -> toolToDescString(t, false)).collect(Collectors.joining("\n#\n")));
      //TODO: Need general way to remove certain actions from the advertised list in specific contexts
      promptVars.put("%taskInstructList", instructTools.stream().filter(t -> !t.toolDefinition.function.name.startsWith("measureWifiAtLocations") && !t.toolDefinition.function.name.startsWith("measureTemperatureAtLocations")).map(t -> toolToDescString(t, false)).collect(Collectors.joining("\n#\n")));
      promptVars.putAll(initPromptVars());
   }

   @Override
   public void shutdownComponent() {
      super.shutdownComponent();
      try {
         TRADE.cancelNotification(this, "joined",null);
      } catch (TRADEException e) {
         log.error("[shutdownComponent] error canceling TRADE notification for new consultant join");
      }

      TRADEServiceInfo newConsultantPropertyCallback = getMyService("newConsultantPropertyCallback");
      TRADE.getAvailableServices(new TRADEServiceConstraints().name("unregisterForNewPropertyNotification")).forEach(x -> {
         try {
            x.call(void.class, newConsultantPropertyCallback);
         } catch (TRADEException e) {
            log.error("[shutdownComponent] error unregistering for new property notifications from consultants when shutting down",e);
         }
      });
   }

   @Override
   public Map<String,ChatTemplates> generateFocusedContextChatTemplates() {
      Map<String,ChatTemplates> chatTemplates = new HashMap<>();
      // Focused context toolsets:
      // Goal Scheduling
      List<Tool> scheduleGoalTools = new ArrayList<>();
      LinkedHashMap<String,ToolParameterV2> argMap = new LinkedHashMap<>();
      argMap.put("action",new ToolParameterV2("object", "The action to be performed. Please refer to the Action List in order to understand acceptable values and forms.", "nluextended"));
      argMap.put("start",new ToolParameterV2("string", "When to first execute the task. Valid forms are a relative time represented by start(<number>,<timeunit>) or an absolute time represented by start(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss). For example, the relative time of 2 hours from now == start(2,HOURS). The absolute time of noon == start(12-00-00). Being told to perform the first execution immediately == start(0,SECONDS)."));
      argMap.put("repeat",new ToolParameterV2("string","On what interval to repeat execution of the task. Valid forms are a relative time represented by repeat(<number>,<timeunit>). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. For example, the relative time of 30 minutes from now == repeat(30,MINUTES). If the task should only be performed once and not repeated, supply a value of repeat(nonrepeating)."));
      argMap.put("end",new ToolParameterV2("string", "When to stop repeating execution of the task. If this task is not repeating (meaning repeat is defined as repeat(nonrepeating)), you must supply the value end(nonrepeating). If end(indefinitely) is provided as the value, that means the task will continue repeating execution on the defined repeat until it is explicitly canceled by the user. Otherwise, valid forms are a relative time represented by end(<number>,<timeunit>) or an absolute time represented by end(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss). For example, the relative time of in a quarter of an hour == end(15,MINUTES). The absolute time of 5:30pm == end(17-30-00)."));
      scheduleGoalTools.add(new Tool(new ToolDefinition(new ToolFunction("submitScheduledTaskDefinition",
              "Submit the details of the fully defined scheduled task for execution. Call this when you believe you have all information about the scheduled task finalized and are ready to save.",
              argMap,
              new String[]{"action","start","repeat","end"})),
              UtteranceType.INSTRUCT
      ));
      LinkedHashMap<String,ToolParameterV2> respondToUserWithoutPerformingAnActionArgMap = new LinkedHashMap<>();
      respondToUserWithoutPerformingAnActionArgMap.put("response",new ToolParameterV2("string", "What you want to say to the user"));
      respondToUserWithoutPerformingAnActionArgMap.put("currentScheduledDefinition",new ToolParameterV2("object", "Your current understanding of the scheduled task being instructed to you.", argMap, new String[]{"action","start","repeat","end"}));
      Tool modifiedTextResponseTool1 = new Tool(new ToolDefinition(new ToolFunction("respondToUserWithoutPerformingAnAction",
                                                                   "You are still being taught new or modifying existing details of the scheduled task by the user. You will respond to" +
                                                                   "the user while following the Response Rules and Style Requirements.",
                                                                   respondToUserWithoutPerformingAnActionArgMap,
                                                new String[]{"response","currentScheduledDefinition"})),
                                                UtteranceType.INSTRUCT);
      scheduleGoalTools.add(modifiedTextResponseTool1);
      chatTemplates.put("startSchedulingTask", new ChatTemplates(Prompts.getPrompt("focused/scheduleGoal"), scheduleGoalTools));

      // Tour Teaching
      // Using custom "respondToUserWithoutPerformingAnAction" tool which also explicitly keeps track of the "current template parse" at
      //  each step. The idea is that writing it down each step provides something concretely to validate each step and
      //  better context for incremental LLM population. This could be first application/attempt of "general template
      //  population" structure, which we may want to use for at least general action learning if not every context.
      //List<Tool> learnTourTools = new ArrayList<>();
      //argMap = new LinkedHashMap<>();
      //LinkedHashMap<String,ToolParameterV2> innerArgMap = new LinkedHashMap<>();
      //innerArgMap.put("location", new ToolParameterV2("string","saved :location to visit when performing this step in the sequence", getLocations("self",true).toArray(new String[0]),false));
      //innerArgMap.put("action",new ToolParameterV2("object", "The action to be performed during this step in the sequence. Please refer to the Action List in order to understand acceptable values and forms.","nluextended"));
      //argMap.put("taskName", new ToolParameterV2("string","The name this task should be saved under and later instructed as"));
      //argMap.put("locationActionPairs", new ToolParameterV2("array","The list of action/location pairs defining this task sequence. Execution of this task will consist of iterating over the pairs in this list, visiting each location in sequence, and then performing the corresponding task at that location.",innerArgMap,new String[]{"location","action"}));
      //learnTourTools.add(new Tool(new ToolDefinition(new ToolFunction("submitTaskSequenceDefinition",
      //        "Submit the details of the fully defined task sequence sequence for storage and future use. Call this when you believe you have all information about the task finalized and are ready to save.",
      //        argMap,
      //        new String[]{"taskName","locationActionPairs"})),
      //        UtteranceType.INSTRUCT
      //));
      //respondToUserWithoutPerformingAnActionArgMap = new LinkedHashMap<>();
      //respondToUserWithoutPerformingAnActionArgMap.put("response",new ToolParameterV2("string", "What you want to say to the user. Generate this argument first."));
      //respondToUserWithoutPerformingAnActionArgMap.put("currentTaskSequenceDefinition", new ToolParameterV2("object", "Your current understanding of the steps of the task being taught to you", argMap, new String[]{"taskName","locationActionPairs"}));
      //Tool modifiedTextResponseTool2 = new Tool(new ToolDefinition(new ToolFunction("respondToUserWithoutPerformingAnAction",
      //        "You are still being taught new or modifying existing details of the task sequence by the user. You will respond to the user while following the Response Rules and Style Requirements.",
      //        respondToUserWithoutPerformingAnActionArgMap,
      //        new String[]{"response","currentTaskSequenceDefinition"})),
      //        UtteranceType.INSTRUCT);
      //learnTourTools.add(modifiedTextResponseTool2);
      //chatTemplates.put("learnTaskSequence", new ChatTemplates(Prompts.getPrompt("focused/learnTaskSequence"), learnTourTools));

      // More General Task Teaching Template
      List<Tool> learnTaskTools = new ArrayList<>();
      argMap = new LinkedHashMap<>();
      argMap.put("taskName", new ToolParameterV2("string","The name this task should be saved under and later instructed as"));
      argMap.put("actionList", new ToolParameterV2("array","The list of actions to be performed as substeps of the learned task. Execution of this task will consist of performing this list of tasks in order. Please refer to the Action List in order to understand acceptable values and forms.","nluextended"));
      learnTaskTools.add(new Tool(new ToolDefinition(new ToolFunction("submitTaskDefinition",
              "Submit the details of the fully defined task for storage and future use. Call this when you believe you have all information about the task finalized and are ready to save.",
              argMap,
              new String[]{"taskName","actionList"})),
              UtteranceType.INSTRUCT
      ));
      respondToUserWithoutPerformingAnActionArgMap = new LinkedHashMap<>();
      respondToUserWithoutPerformingAnActionArgMap.put("response",new ToolParameterV2("string", "What you want to say to the user. Generate this argument first."));
      respondToUserWithoutPerformingAnActionArgMap.put("currentTaskDefinition", new ToolParameterV2("object", "Your current understanding of the steps of the task being taught to you", argMap, new String[]{"taskName","actionList"}));
      Tool modifiedTextResponseTool2 = new Tool(new ToolDefinition(new ToolFunction("respondToUserWithoutPerformingAnAction",
              "You are still being taught new or modifying existing details of the task by the user. You will respond to the user while following the Response Rules and Style Requirements.",
              respondToUserWithoutPerformingAnActionArgMap,
              new String[]{"response","currentTaskDefinition"})),
              UtteranceType.INSTRUCT);
      learnTaskTools.add(modifiedTextResponseTool2);
      chatTemplates.put("startLearningNewTask", new ChatTemplates(Prompts.getPrompt("focused/learnTask"), learnTaskTools));

      // Goal status modification
      argMap = new LinkedHashMap<>();
      argMap.put("modification", new ToolParameterV2("string","The type of status modification to apply", new String[]{"cancel","suspend","resume"},true));
      argMap.put("taskType", new ToolParameterV2("string","Which task collection the desired task to modify is found in", new String[]{"current","queued","scheduled"}, true));
      argMap.put("taskIndex",new ToolParameterV2("array","The index(es) of desired task(s) to modify within the corresponding task collection. May be empty or [0] in the case of a 'current' taskType."));
      List<Tool> modifyExistingTaskStatusTools = new ArrayList<>();
      modifyExistingTaskStatusTools.add(new Tool(new ToolDefinition(new ToolFunction("submitTaskStatusModificationDefinition",
              "Submit the details of the fully defined task status modification for application.", 
              argMap,
              new String[]{"modification", "taskType", "taskIndex"})),
              UtteranceType.INSTRUCT
      ));
      respondToUserWithoutPerformingAnActionArgMap = new LinkedHashMap<>();
      respondToUserWithoutPerformingAnActionArgMap.put("response", new ToolParameterV2("string", "What you want to say to the user. Generate this argument first."));
      respondToUserWithoutPerformingAnActionArgMap.put("currentTaskStatusModificationDefinition", new ToolParameterV2("object", "Your current understanding of the task status modification being instructed to you.", argMap,new String[]{"modification", "taskType", "taskIndex"}));
      modifyExistingTaskStatusTools.add(new Tool(new ToolDefinition(new ToolFunction("respondToUserWithoutPerformingAnAction",
              "You are still being told new or modifying existing details of the task status modification by the user. You will respond to the user while following the Response Rules and Style Requirements.",
              respondToUserWithoutPerformingAnActionArgMap,
              new String[]{"response","currentTaskStatusModificationDefinition"})),
              UtteranceType.INSTRUCT));
      //TODO: when we have more time - modify the template to allow for these behaviors
      modifyExistingTaskStatusTools.add(new Tool(new ToolDefinition(new ToolFunction("cancelAllCurrentAndQueuedTasks",
              "Cancel the current task and all tasks in the queue. Only use this when the user wants to cancel all assigned tasks, as the standard taskStatusModification template is not flexible enough to represent this.")),
              UtteranceType.INSTRUCT
      ));
      chatTemplates.put("modifyExistingTaskStatus", new ChatTemplates(Prompts.getPrompt("focused/goalStatusManipulation"), modifyExistingTaskStatusTools));

      return chatTemplates;
   }

   /**
    * Initialize Prompt variables whose values are static during runtime or updated by event-based listeners
    */
   protected Map<String, String> initDomainPromptVars() {
      Map<String, String> vars = new LinkedHashMap<>();
      // TODO: If Ava components are ever extended to allow runtime saving of these entities, make corresponding updates
      //    to allow for dynamic updating of prompt vars (ideally through subscriber/listener setup)
      List<String> locations = getLocations("self", true);
      List<String> videos = getVideoNames();
      List<String> images = getImageNames();
      String locationList = " -- " + String.join("\n -- ", locations);
      if (!locationList.isEmpty()) {
         locationList += "\n";
      }
      String videoList = " -- " + String.join("\n -- ", videos);
      if (!videoList.isEmpty()) {
         videoList += "\n";
      }
      String imageList = " -- " + String.join("\n -- ", images);
      if (!videoList.isEmpty()) {
         imageList += "\n";
      }
      vars.put("%locationList", locationList);
      vars.put("%videoList", videoList);
      vars.put("%exampleVideo", videos.isEmpty() ? "example_video" : videos.get(0));
      vars.put("%exampleImage", images.isEmpty() ? "example_image" : images.get(0));
      vars.put("%imageList", imageList);
      vars.put("%exampleLocation", locations.isEmpty() ? "example_location:location" : locations.get(0));
      vars.put("%language", language);

      vars.put("%toolArgumentTypeDescription","- Any tool call parameter below which references a :location, :video, or :image can ONLY take values from the corresponding Location List, Video List, or Image List above. Values not present in the lists must be rejected\n-- For locations specifically, the values MUST include the :location suffix. For example, " + vars.get("%exampleLocation") + ".\n- If asked to go to a location which does not exist, call the \"getAssistanceOnUsingSaveLocationAndGoToLocationProperly\" tool to learn how to save new locations.\n- New videos and images can be added through the associated website.");
      return vars;
   }

   @Override
   protected Map<String,String> updateDomainPromptVars(Utterance utterance, boolean parsing) {
      Map<String,String> vars = new HashMap<>();
      vars.put("%batteryLevel", getBatteryLevel());
      vars.put("%currentLocation", getCurrentLocation());
      vars.put("%videoVolume", getVideoVolume());
      vars.put("%voiceVolume", getVoiceVolume());
      //Failed attempt at including context to be used in the base prompt itself
      ////TODO: based of EM listener
      //Symbol agent = parsing ? utterance.getAddressee() : utterance.getSpeaker();
      //Symbol user = parsing ? utterance.getSpeaker() : utterance.getAddressee();
      //vars.put("%goalSummaryCondensed", getCondensedGoalSummary(agent));
      ////TODO: once per chat cycle
      //List<ChatContext> extendedHistory = chatContextHistories.get(Pair.of(user.toUntyped(),agent.toUntyped()));
      //vars.put("%lastConvoSummary", (extendedHistory == null || extendedHistory.isEmpty()) ? "- <No past chat histories>" : getChatHistoryAsString(extendedHistory.get(0).history));
      return vars;
   }

   @Override
   protected String getBaseRoleDescription() {
      return "You are Ava, a social autonomous mobile robot with an associated back-end system.";
   }

   @Override
   protected String getBaseContextualInformation() {
      return //"- Current Location: %currentLocation\n" +
             // "- Current Battery Level: %batteryLevel\n" +
             // "- Current Video Volume: %videoVolume\n" +
             // "- Current Speaking Voice Volume: %voiceVolume\n" +
             // ////"- OUTSTANDING QUESTIONs the back-end is awaiting a response for:\n" +
             // ////"%questionList\n" +
              "- Location List:\n" +
              "%locationList\n" +
              "- Video List:\n" +
              "%videoList\n" +
              "- Image List\n" +
              "%imageList\n" +
             "- Use one of the following" + itkActionNames + " tool calls to get any other corresponding information if necessary.";
   }

   //Failed attempt at including context to be used in the base prompt itself
   @TRADEService
   public String getCondensedGoalSummary(Symbol agent) {
      List<Predicate> currentGoalPredicates = getCurrentGoals(agent);
      List<Predicate> pendingGoalPredicates = getPendingGoals(agent);
      List<Predicate> scheduledGoals = getScheduledGoals(agent);
      int numPending = pendingGoalPredicates.size();
      int numScheduled = scheduledGoals.size();


      StringBuilder stringBuilder = new StringBuilder();
//      stringBuilder.append("-- Current Task (the task you are focused on): ");
      stringBuilder.append("-- Current Task: ");
      if (!currentGoalPredicates.isEmpty()) {
         Predicate currentGoalPredicate = replaceReferencesInSemantics(currentGoalPredicates.get(0));
         if (currentGoalPredicate.hasArgs()) {
            String statusString = currentGoalPredicate.getName();
            stringBuilder.append(currentGoalPredicate.get(0));
            stringBuilder.append(". Status: ").append(statusString).append("\n");
         } else {
            stringBuilder.append(currentGoalPredicate).append("\n");
         }
      } else {
         stringBuilder.append(" <You have no current task>\n");
      }

      stringBuilder.append("-- Queued Tasks:\n");
      //stringBuilder.append("-- Queued Tasks (ordered list of tasks which will become the current task when the current task completes):\n");
      if (pendingGoalPredicates.isEmpty()) {
         stringBuilder.append(" --- <You have no pending tasks>\n");
      } else {
         for (int i = 0; i < numPending; i++) {
            stringBuilder.append(" --- ").append(i).append(": ").append(replaceReferencesInSemantics(pendingGoalPredicates.get(i))).append("\n");
         }
      }

      //stringBuilder.append("-- Scheduled Tasks (tasks which will be submitted to the Pending Task Queue at specific times in the future):\n");
      stringBuilder.append("-- Scheduled Tasks:\n");
      if (scheduledGoals.isEmpty()) {
         stringBuilder.append(" --- <You have no scheduled tasks>\n");
      } else {
         for (int i = 0; i < numScheduled; i++) {
            stringBuilder.append(" --- ").append(replaceReferencesInSemantics(scheduledGoals.get(i))).append("\n");
         }
      }

      return stringBuilder.toString();
   }

   @TRADEService
   public void newConsultantCallback(TRADEServiceInfo tsi) {
      TRADEServiceInfo newConsultantPropertyCallback = getMyService("newConsultantPropertyCallback");
      TRADE.getAvailableServices(new TRADEServiceConstraints().name("registerForNewPropertyNotification")).forEach( x -> {
                 try {
                    x.call(void.class, newConsultantPropertyCallback);
                 } catch (TRADEException e) {
                    log.error("[newConsultantCallback] error registering for new properties callback after a new consultant joined",e);
                 }
              }
      );
   }

   @TRADEService
   public void newConsultantPropertyCallback() {
      promptVars.putAll(initPromptVars());
      //TODO: incredibly inefficient. Figure out better way to update all tool definitions dealing with references
      //TODO: need similar notifications for images and videos
      List<String> locationList = getLocations("self",true);
      for (ChatTemplates templates : chatTemplates.values()) {
         for (ToolDefinition toolDefinition : templates.toolDefinitions) {
            for (ToolParameterV2 toolParameter : toolDefinition.function.parameters.getProperties().values()) {
               updatePossibleValues(locationList, toolParameter, "location");
            }
         }
      }
   }

   private void updatePossibleValues(List<String> possible_values, ToolParameterV2 param, String kbName) {
      Pattern pattern = Pattern.compile(".*:(.+)");
      Matcher matcher;
      if (!param.getProperties().isEmpty()) {
         for (ToolParameterV2 innerParam : param.getProperties().values()) {
            updatePossibleValues(possible_values, innerParam, kbName);
         }
      } else {
         if (!param.getPossible_values().isEmpty()) {
            matcher = pattern.matcher(param.getPossible_values().get(0));
            if (matcher.matches() && matcher.group(1) != null) {
               String type = matcher.group(1);
               if (type.equals(kbName)) {
                  param.setPossible_values(possible_values);
               }
            }
         }
      }
   }

   private String getBatteryLevel() {
      String batteryLevel = "unknown";
      try {
         Double batteryLevelDouble = TRADE.getAvailableService(new TRADEServiceConstraints().name("getBatteryLevel").argTypes()).call(Double.class);
         if (batteryLevelDouble != null) {
            batteryLevel = String.valueOf(batteryLevelDouble.longValue());
         }
      } catch (TRADEException e) {
         log.error("[getBatteryLevel] error getting Ava battery level");
      }
      return batteryLevel;
   }

   private String getCurrentLocation() {
      Predicate query = Factory.createPredicate("at","X","Y");
      List<Map<Variable, Symbol>> bindings;
      String robotLocationString = "unknown";
      try {
         bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class,query);
         TRADEServiceInfo tsi = getServiceForAgent("self", "getMapReferences");
         Map<Symbol, PoseReference> refs = new HashMap<>();
         if (tsi != null) {
            refs = tsi.call(Map.class);
         }
         if (refs != null && bindings != null) {
            for (Map<Variable, Symbol> binding : bindings) {
               Symbol y = binding.get(new Variable("Y"));
               //TODO: should be based on group
               PoseReference ref = refs.get(y);
               if (ref != null) {
                  if (!ref.properties.isEmpty()) {
                     robotLocationString = String.format("You were last at %s", ref.properties.get(0).getName());
                  }
               }
            }
         }
      } catch (TRADEException e) {
         log.error("[getCurrentLocation] ", e);
         //return "Nothing is known by the robot";
         return "Unknown";
      }

      String output = robotLocationString;
      if (output.isEmpty()) {
         log.warn("[getCurrentLocation] no matching beliefs found");
         //output = "<Nothing is known by the robot>";
         output = "Unknown";
      }
      return output;
   }

   //TODO: streaming TTS only - remove associated tool targets if not using streaming TTS
   private String getVoiceVolume() {
      String voiceVolume = "unknown";
      //TODO: check if streaming TTS to avoid error message spam
      try {
         Integer voiceVolumeInt = TRADE.getAvailableService(new TRADEServiceConstraints().name("getVoiceVolume").argTypes()).call(Integer.class);
         if (voiceVolumeInt != null) {
            voiceVolume = String.valueOf(voiceVolumeInt);
         }
      } catch (TRADEException e) {
         log.error("[getVoiceVolume] error getting Ava voice volume");
      }
      return voiceVolume;
   }

   private String getVideoVolume() {
      String videoVolume = "unknown";
      try {
         Integer videoVolumeInt = TRADE.getAvailableService(new TRADEServiceConstraints().name("getVideoVolume").argTypes()).call(Integer.class);
         if (videoVolumeInt != null) {
            videoVolume = String.valueOf(videoVolumeInt);
         }
      } catch (TRADEException e) {
         log.error("[getBatteryLevel] error getting Ava Display video volume");
      }
      return videoVolume;
   }

   @Override
   protected List<String> getRefTypes() {
      return List.of("location");
   }

   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //// Should be domain specific

   // TODO: rename this? Extends past just "correcting common mistakes", currently being used to convert from flexible LLM targets to actual underlying action
   // TODO: Revisit old error cases and explicitly handle if finding they are still possible
   // Modify semantics based on domain specific postprocessing
   // Return - Error String to add onto baseToolResponse to be sent when handling this utterance as an error.
   //          Empty String if no issues found
   protected String correctCommonNLUMistakes(Utterance utterance) {
      Predicate semantics = (Predicate) utterance.getSemantics();
      StringBuilder errorStringBuilder = new StringBuilder();
      Predicate transformed = correctCommonNLUMistakesHelper(semantics, utterance.getAddressee(), utterance.getSpeaker(), errorStringBuilder);
      if (transformed == null || !errorStringBuilder.isEmpty()) {
         return errorStringBuilder.toString();
      } else {
         utterance.setSemantics(transformed);
         return "";
      }
   }
   
   protected Predicate correctCommonNLUMistakesHelper(Predicate semantics, Symbol actor, Symbol instructor, StringBuilder errorStringBuilder) {
      try {
         String command = semantics.getName();
         if (command.equals("getActionSteps")) {
            //Not currently prompted to do this
            if (semantics.hasArgs()) {
               Symbol innerArg = semantics.get(1);
               List<Symbol> argsWithActor = new ArrayList<>();
               argsWithActor.add(actor);
               if (innerArg.isTerm()) {
                  innerArg = correctCommonNLUMistakesHelper((Predicate)innerArg, actor, instructor, errorStringBuilder);
                  if (innerArg != null) {
                     argsWithActor.addAll(((Term)innerArg).getArgs());
                  } else {
                     return null;
                  }
               }
               return Factory.createPredicate(command, actor, Factory.createPredicate(innerArg.getName(), argsWithActor));
            }
         } else if (command.equals("submitScheduledTaskDefinition")) {
            List<Symbol> args = new ArrayList<>();
            args.add(null);
            args.add(null);
            args.add(null);
            args.add(null);
            boolean hasStart = false;
            boolean hasPeriod = false;
            boolean hasEnd = false;
            for (Symbol arg : semantics.getArgs()) {
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
                  } else {
                     Predicate transformedArg = correctCommonNLUMistakesHelper((Predicate)arg, actor, instructor, errorStringBuilder);
                     if (transformedArg != null) {
                        args.add(0, transformedArg);
                     }
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
               return Factory.createPredicate("scheduled", args);
            } else {
               return null;
            }
         } else if (command.equals("submitTaskSequenceDefinition")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() != 3) {
               log.error("[correctCommonNLUMistakes] tourDefinition has unexpected number of arguments, this should have been caught in tool processing?");
            } else {
               String tourName = args.get(1).toString();
               if (!tourName.matches("[A-Za-z0-9_]+")) {
                  Symbol cleanedName = Factory.createSymbol(tourName.trim().replaceAll("\\s+", "_").replaceAll("[^a-zA-Z0-9_]", ""));
                  args.remove(1);
                  args.add(1, cleanedName);
               }

               Predicate andPred = (Predicate) args.get(2);
               List<Predicate> transformedPairs = new ArrayList<>();
               for (Symbol pair : andPred.getArgs()) {
                  Predicate pairPred = (Predicate)pair;
                  Predicate actionPred = (Predicate)pairPred.get(1);
                  Predicate transformedPred = correctCommonNLUMistakesHelper(actionPred, actor, instructor, errorStringBuilder);
                  transformedPairs.add(Factory.createPredicate("and",pairPred.get(0),transformedPred));
               }
               args.remove(2);
               args.add(Factory.createPredicate("and",transformedPairs));

               if (errorStringBuilder.isEmpty()) {
                  return Factory.createPredicate(semantics.getName(), args);
               } else {
                  return null;
               }
            }
         } else if (command.equals("submitTaskDefinition")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() != 3) {
               log.error("[correctCommonNLUMistakes] taught action definition has unexpected number of arguments, this should have been caught in tool processing?");
            } else {
               String taskName = args.get(1).toString();
               if (!taskName.matches("[A-Za-z0-9_]+")) {
                  Symbol cleanedName = Factory.createSymbol(taskName.trim().replaceAll("\\s+", "_").replaceAll("[^a-zA-Z0-9_]", ""));
                  args.remove(1);
                  args.add(1, cleanedName);
               }

               Predicate andPred = (Predicate) args.get(2);
               List<Predicate> transformedSteps = new ArrayList<>();
               for (Symbol step : andPred.getArgs()) {
                  Predicate stepPred = (Predicate)step;
                  Predicate transformedPred = correctCommonNLUMistakesHelper(stepPred, actor, instructor, errorStringBuilder);
                  transformedSteps.add(transformedPred);
               }
               args.remove(2);
               args.add(Factory.createPredicate("and",transformedSteps));

               if (errorStringBuilder.isEmpty()) {
                  return Factory.createPredicate(semantics.getName(), args);
               } else {
                  return null;
               }
            }
         } else if (command.equals("checkPastInteractionHistoryForMissingInformation")) {
            List<Symbol> args = semantics.getArgs();
            args.add(instructor);
            args.add(actor);
            return Factory.createPredicate(semantics.getName(), args);
         }
         else if (command.equals("escort")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() == 3) {
               return Factory.createPredicate("escortToLocation",args);
            } else if (args.size() == 4){
               return Factory.createPredicate("escortFromLocationToLocation",args);
            }
         } else if (command.equals("measureWifiSignal")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() == 1) {
               return Factory.createPredicate("measureWifiAtCurrentLocation",args);
            } else if (args.size() == 2) {
               if (args.get(1).isPredicate()) {
                  if (args.get(1).hasArgs()) {
                     return Factory.createPredicate("measureWifiAtLocations", args);
                  } else {
                     return Factory.createPredicate("measureWifiAtCurrentLocation",args.get(0));
                  }
               } else {
                  return Factory.createPredicate("measureWifiAtCurrentLocationAndReportAt",args);
               }
            } else if (args.size() == 3) {
               if (args.get(1).hasArgs()) {
                  return Factory.createPredicate("measureWifiAtLocationsAndReportAt", args);
               } else {
                  return Factory.createPredicate("measureWifiAtCurrentLocationAndReportAt",args.get(0), args.get(2));
               }
            }
         } else if (command.equals("measureTemperature")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() == 1) {
               return Factory.createPredicate("measureTemperatureAtCurrentLocation",args);
            } else if (args.size() == 2) {
               if (args.get(1).isPredicate()) {
                  if (args.get(1).hasArgs()) {
                     return Factory.createPredicate("measureTemperatureAtLocations", args);
                  } else {
                     return Factory.createPredicate("measureTemperatureAtCurrentLocation",args.get(0));
                  }
               } else {
                  return Factory.createPredicate("measureTemperatureAtCurrentLocationAndReportAt",args);
               }
            } else if (args.size() == 3) {
               if (args.get(1).hasArgs()) {
                  return Factory.createPredicate("measureTemperatureAtLocationsAndReportAt", args);
               } else {
                  return Factory.createPredicate("measureTemperatureAtCurrentLocationAndReportAt",args.get(0), args.get(2));
               }
            }
         } else if (command.equals("playVideo")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() == 1) {
               return Factory.createPredicate("playNextVideo",args);
            } else if (args.size() == 2) {
               if (args.get(1).hasType()) {
                  return Factory.createPredicate("playNextVideoAtLocation",args);
               } else {
                  return Factory.createPredicate("playSpecificVideo",args);
               }
            } else if (args.size() == 3) {
               return Factory.createPredicate("playSpecificVideoAtLocation",args);
            }
         } else if (command.equals("displayImage")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() == 2) {
               return Factory.createPredicate("displayNamedImage",args);
            } else if (args.size() == 3) {
               if (args.get(2).hasType()) {
                  return Factory.createPredicate("displayNamedImageAtLocation",args);
               } else {
                  return Factory.createPredicate("displayNamedImageForDuration",args);
               }
            } else if (args.size() == 4) {
               return Factory.createPredicate("displayNamedImageForDurationAtLocation",args);
            }
         } else if (command.equals("getPreviouslyMeasuredWifiAtLocationsString")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() == 1) {
               return Factory.createPredicate("getPreviouslyMeasuredWifiAtCurrentLocationString",args);
            } else if (args.size() == 2 && !args.get(1).hasArgs()) {
               return Factory.createPredicate("getPreviouslyMeasuredWifiAtCurrentLocationString",args.get(0));
            }
         } else if (command.equals("getPreviouslyMeasuredTemperatureAtLocationsString")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() == 1) {
               return Factory.createPredicate("getPreviouslyMeasuredTemperatureAtCurrentLocationString", args);
            } else if (args.size() == 2 && !args.get(1).hasArgs()) {
               return Factory.createPredicate("getPreviouslyMeasuredTemperatureAtCurrentLocationString", args.get(0));
            }
         } else if (command.equals("reportPreviouslyMeasuredWifiAtLocations")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() == 1) {
               return Factory.createPredicate("reportPreviouslyMeasuredWifiAtCurrentLocation",args);
            } else if (args.size() == 2 && !args.get(1).hasArgs()) {
               return Factory.createPredicate("reportPreviouslyMeasuredWifiAtCurrentLocation",args.get(0));
            }
         } else if (command.equals("reportPreviouslyMeasuredTemperatureAtLocations")) {
            List<Symbol> args = semantics.getArgs();
            if (args.size() == 1) {
               return Factory.createPredicate("reportPreviouslyMeasuredTemperatureAtCurrentLocation", args);
            } else if (args.size() == 2 && !args.get(1).hasArgs()) {
               return Factory.createPredicate("reportPreviouslyMeasuredTemperatureAtCurrentLocation", args.get(0));
            }
         } else if (command.equals("submitTaskStatusModificationDefinition")) {
            List<Symbol> args = semantics.getArgs();
            String modificationType = args.get(1).getName();
            String taskType = args.get(2).getName();
            Predicate taskIndicesPred = (Predicate)args.get(3);
            if (taskType.equals("current")) {
              switch (modificationType) {
                 case "suspend":
                    return Factory.createPredicate("suspendCurrentTask", args.get(0));
                 case "resume":
                    return Factory.createPredicate("resumeCurrentTask", args.get(0));
                 case "cancel":
                    return Factory.createPredicate("cancelCurrentTask", args.get(0));
              }
            } else {
               List<Long> taskIndices = new ArrayList<>();
               for (Symbol arg : taskIndicesPred.getArgs()) {
                  try {
                     Long index = (long) Double.parseDouble(arg.getName());
                     taskIndices.add(index);
                  } catch (NumberFormatException e) {
                     log.error("[correctCommonNLUMistakes] unexpected task index argument {}", arg, e);
                  }
               }
               if (taskIndices.isEmpty()) {
                  errorStringBuilder.append("- at least one taskIndex must be supplied for the task you are attempting to cancel");
               }
               if (!modificationType.equals("cancel")) {
                  errorStringBuilder.append("- the supplied type of tasks can only be canceled, not suspended or resumed");
               }

               //TODO: check if indices are in bounds here?
               //TODO: better NLG for this?
               if (taskType.equals("queued")) {
                  return Factory.createPredicate("cancelQueuedTasks", args.get(0),taskIndicesPred);
               } else if (taskType.equals("scheduled")) {
                  return Factory.createPredicate("cancelScheduledTasks",args.get(0),taskIndicesPred);
               }
            }
         } else if (command.equals("saveLocation")) {
            //TODO: type should be manually stripped and/or tool argument processing updated to only slot in VAR for actual existing entities to prevent LLM mistakes
            List<Symbol> args = semantics.getArgs();
            if (args.get(1).hasType()) {
               errorStringBuilder.append("- saveLocation takes in a new location label, not an existing location entity. Please call this tool again without the :location suffix. Do not involve the user in this operation.");
            }
         } else if (command.equals("urgentPriority") || command.equals("forgetTaughtAction")) {
            Predicate transformedArg = correctCommonNLUMistakesHelper((Predicate)semantics.get(1), actor, instructor, errorStringBuilder);
            if (transformedArg != null) {
               if (command.equals("forgetTaughtAction")) {
                  List<String> taughtActionNames = learnedActions.stream().map(DBEntry::getType).toList();
                  if (!taughtActionNames.contains(transformedArg.getName())) {
                     errorStringBuilder.append("- the supplied actionPredicate argument does not match an action previously taught by the user. Only user taught actions can be forgotten, not predefined ones; the current list of known taught action names are ").append(taughtActionNames);
                  }
               }
               return Factory.createPredicate(command,semantics.get(0),transformedArg);
            }
         }
      }
      catch (Exception e) {
         log.warn("[correctCommonNLUMistakes] unexpected semantics format {}", semantics);
         //TODO: explicitly fail with a specific message here? Or just return unmodified utterance?
      }
      return semantics;
   }

   private String getMatchingNameIgnoreCase(String query, List<String> targets) {
      for (String target : targets) {
         if (query.equalsIgnoreCase(target)) {
            return target;
         }
      }
      return "";
   }

   private List<String> getVideoNames() {
      try {
         List<String> names = TRADE.getAvailableService(new TRADEServiceConstraints().name("getVideoNames").argTypes(Boolean.class)).call(List.class,true);
         return names.stream().map(vid->vid.toLowerCase()).toList();
      } catch (TRADEException e) {
         log.error("[getVideoNames]", e);
      }
      return new ArrayList<>();
   }

   private List<String> getImageNames() {
      try {
         List<String> names = TRADE.getAvailableService(new TRADEServiceConstraints().name("getImageNames").argTypes(Boolean.class)).call(List.class,true);
         return names.stream().map(img->img.toLowerCase()).toList();
      } catch (TRADEException e) {
         log.error("[getImageNames]", e);
      }
      return new ArrayList<>();
   }

   //TODO: calls to this/getMapReferences wildly inefficient, fix
   private List<String> getLocations(String actorName, boolean english) {
      try {
         Map<Symbol, PoseReference> refs = new HashMap<>();
         //TODO: should be based on group
         TRADEServiceInfo tsi = getServiceForAgent(actorName, "getMapReferences");
         //TODO: resolver issue? Sometimes location list is empty?
         int maxAttempts = 3;
         for (int i=0; i<maxAttempts;i++) {
            if (tsi != null) {
               refs = tsi.call(Map.class);
            }
            if (refs != null && !refs.isEmpty()) {
               return refs.values().stream().map(ref -> {
                  if (!ref.properties.isEmpty()) {
                     return ref.properties.get(0).getName() + ":location";
                  } else {
                     return "";
                  }
               }).toList();
            }
         }
      } catch (TRADEException e) {
         log.error("[getLocations]", e);
      }

      return new ArrayList<>();
   }

   private String getWorldStateString(Symbol agent) {
      Predicate query = Factory.createPredicate("at","X","Y");
      List<Map<Variable, Symbol>> bindings;
      String robotLocationString = "";
      try {
         bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class,query);
         TRADEServiceInfo tsi = getServiceForAgent("self", "getMapReferences");
         Map<Symbol, PoseReference> refs = new HashMap<>();
         if (tsi != null) {
            refs = tsi.call(Map.class);
         }
         if (refs != null && bindings != null) {
            for (Map<Variable, Symbol> binding : bindings) {
               Symbol y = binding.get(new Variable("Y"));
               //TODO: should be based on group
               PoseReference ref = refs.get(y);
               if (ref != null) {
                  if (!ref.properties.isEmpty()) {
                     robotLocationString = String.format("You were last at the following location: %s.\n", ref.properties.get(0).getName());
                  }
               }
            }
         }
      } catch (TRADEException e) {
         log.error("[getWorldState] ", e);
         //return "Nothing is known by the robot";
         return "You do not know where you are";
      }

      String output = robotLocationString;
      if (output.isEmpty()) {
         log.warn("[getWorldState] no matching beliefs found for agent {}", agent);
         //output = "<Nothing is known by the robot>";
         output = "You do not know where you are";
      }
      return output;
   }

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
   //TODO: add example instruction fields to ToolDefinitions and conditionally add to output placed in prompt if desired
   //TODO: somehow clearly label this section of tools as 'Action tools' with an associated description in the overall prompt (hard to do because of current model side templating)
   public List<Tool> generateInstructTools(boolean xml) {
      List<Tool> instructTools = new ArrayList<>();
      ToolDefinition definition;
      String[] locationNames = getLocations("self", true).toArray(new String[0]);

      ////TODO: standardize concept of 'tools only advertised under specific contexts' not specifically related to focused contexts.
      ////      (Short term impl of this in SocialTRLLMServiceProvider, getContextDependentToolDefinitions)
      //definition = new ToolDefinition(new ToolFunction("stopFollowing",
      //        "You stop following the user",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));


      definition = new ToolDefinition(new ToolFunction("goToLocation",
              "You go to the supplied destination :location",
              new LinkedHashMap<>(Map.of("destination",new ToolParameterV2("string","The destination :location to go to. ",locationNames,false))),new String[]{"destination"}));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition(new ToolFunction("goHome",
              "You go to your default charging dock and charge"));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition(new ToolFunction("follow",
              "You follow the user"));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      LinkedHashMap<String,ToolParameterV2> argMap = new LinkedHashMap<>();
      argMap.put("name",new ToolParameterV2("string","Name of the entity to escort"));
      argMap.put("start",new ToolParameterV2("string","The start :location to escort from. Don't supply if already with the user",locationNames,false));
      argMap.put("destination",new ToolParameterV2("string","The destination :location to escort to",locationNames,false));
      definition = new ToolDefinition(new ToolFunction("escort",
              "You lead someone between two :locations",
              argMap,
              new String[]{"name","destination"}));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //TODO: Methods you cannot schedule or teach (this, maybe follow/stopFollowing, dontRespond, forgetTaughtAction, focused contexts, ...)
      definition = new ToolDefinition(new ToolFunction("saveLocation",
              "You save your current position under the supplied name. Use the 'getAssistanceOnUsingSaveLocationAndGoToLocationProperly' tool to understand how to use this function properly.",
              new LinkedHashMap<>(Map.of("name",new ToolParameterV2("string","The label to save the location as"))),new String[]{"name"}));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      argMap = new LinkedHashMap<>();
      argMap.put("locations",new ToolParameterV2("array","The ordered list of saved :locations to visit and measure wifi at. Empty if only measuring at your current position",locationNames,false));
      argMap.put("reportLocation",new ToolParameterV2("string","The saved :location to go to and give a summary of the results. Don't supply if not reporting a summary of the results",locationNames,false));
      definition = new ToolDefinition(new ToolFunction("measureWifiSignal",
              "You measure wifi signal strength at certain :locations",
              argMap));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      if (includeDemoTargets) {
         argMap = new LinkedHashMap<>();
         argMap.put("locations",new ToolParameterV2("array","The ordered list of saved :locations to visit and measure temperature at. Empty if only measuring at your current position",locationNames,false));
         argMap.put("reportLocation",new ToolParameterV2("string","The saved :location to go to and give a summary of the results. Don't supply if not reporting a summary of the results",locationNames,false));
         definition = new ToolDefinition(new ToolFunction("measureTemperature",
                 "You measure the temperature at certain :locations",
                 argMap));
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      }

      definition = new ToolDefinition(new ToolFunction("sit",
              "You lower your screen"));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition(new ToolFunction("stand",
              "You raise your screen"));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      argMap = new LinkedHashMap<>();
      argMap.put("imageName",new ToolParameterV2("string","The :image to display",getImageNames().toArray(new String[0]),false));
      argMap.put("duration",new ToolParameterV2("string","How long to display the image for in seconds. Don't supply if displaying indefinitely until told to stop."));
      argMap.put("location",new ToolParameterV2("string","The specific :location at which to display the image. Don't supply if displaying at your current location",locationNames,false));
      definition = new ToolDefinition(new ToolFunction("displayImage",
              "You display an :image from your saved collection on your built-in screen display at a :location.",
              argMap,
              new String[]{"imageName"}
      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      argMap = new LinkedHashMap<>();
      argMap.put("videoName",new ToolParameterV2("string","The specific :video to play. Don't supply if playing a random video",getVideoNames().toArray(new String[0]),false));
      argMap.put("location",new ToolParameterV2("string","The specific :location at which to play the video. Don't supply if playing at your current location",locationNames,false));
      definition = new ToolDefinition(new ToolFunction("playVideo",
              "You begin playback of a :video from your saved collection on your built-in screen display at a :location.",
              argMap
      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition(new ToolFunction("setVideoVolume",
              "You set the volume of videos played on your screen",
              new LinkedHashMap<>(Map.of("volume",new ToolParameterV2("integer","The volume to set the video media player to. Acceptable values are whole numbers between 0 and 100"))),new String[]{"volume"}));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      definition = new ToolDefinition(new ToolFunction("setVoiceVolume",
              "You set the volume of your own speaking voice",
              new LinkedHashMap<>(Map.of("volume",new ToolParameterV2("integer","The voice volume as a percentage. Acceptable values are whole numbers between 0 and 100"))),new String[]{"volume"}));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      ////TODO: should be general targets moved down to base class?
      definition = new ToolDefinition(new ToolFunction("waitForDuration",
              "You wait idly for a number of seconds",
              new LinkedHashMap<>(Map.of("seconds",new ToolParameterV2("integer","The amount of seconds to wait. Must be a positive whole number"))),new String[]{"seconds"}));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      definition = new ToolDefinition(new ToolFunction("dontRespond",
              "The user has asked you to not verbally respond"));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //TODO: forgetTaughtAction(s) focused context with action specification similar to that for goal manipulation
      //TODO: restrict list of acceptable tools to taught ones, not all nlu targets
      String exampleCallString = xml ?
              "<function=forgetTaughtAction><parameter=actionPredicate>{\"name\":\"goToLocation\",\"arguments\":{\"destination\":\""+promptVars.get("%exampleLocation")+"\"}}</parameter></function>" :
              "{\"name\":\"forgetTaughtAction\", \"arguments\":{\"actionPredicate\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\""+promptVars.get("%exampleLocation")+"\"}}}}";
      definition = new ToolDefinition(new ToolFunction("forgetTaughtAction",
              "You forget/delete a previously taught action",
              new LinkedHashMap<>(Map.of("actionPredicate",new ToolParameterV2("object","Populated JSON representing the tool call to forget. This json minimally needs \"name\" and \"argument\" fields, for example: " + exampleCallString, "nlu"))),
              new String[]{"actionPredicate"}));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition(new ToolFunction("forgetAllTaughtActions",
              "You forget all previously taught actions"));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //TODO: goalPriority(re)assignment focused context
      exampleCallString = xml ?
              "<function=urgentPriority><parameter=actionPredicate>{\"name\":\"goToLocation\",\"arguments\":{\"destination\":\""+promptVars.get("%exampleLocation")+"\"}}</parameter></function>" :
              "{\"name\":\"urgentPriority\", \"arguments\":{\"actionPredicate\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\""+promptVars.get("%exampleLocation")+"\"}}}}";
      definition = new ToolDefinition(new ToolFunction("urgentPriority",
              "You submit an action with urgent priority. This means that this action will interrupt execution of any other currently submitted actions and be immediately prioritized. \nExample Instruction: \"go to "+promptVars.get("%exampleLocation")+" right now.\" \nExample Call: " + exampleCallString,
              new LinkedHashMap<>(Map.of("actionPredicate",new ToolParameterV2("object","Populated JSON of another tool call representing the action to submit with urgent priority. This json minimally needs \"name\" and \"argument\" fields, like shown in the example.", "nlu"))),
              new String[]{"actionPredicate"}));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //TODO long term: make focused "configure kiosk mode" context
      //TODO short term: including any version of kiosk mode through language? <- think this is either something that needs to be externally explained or integrated deeply enough that it is a focused context.
      //                 Random targets for unexplained functionality cannot work well.
      //exampleCallString = xml ?
      //        "<function=setKioskModeEnabled><parameter=enabled>true</parameter></function>" :
      //        "{\"name\":\"setKioskModeEnabled\", \"arguments\":{\"enabled\":\"true\"}}";
      //definition = new ToolDefinition(new ToolFunction("setKioskModeEnabled",
      //        "You set whether or not kiosk mode is enabled. \nExample Instruction: \"turn on kiosk mode.\" \nExample Call: " + exampleCallString,
      //        new LinkedHashMap<>(Map.of("enabled",new ToolParameterV2("boolean","the truth value of kiosk mode being enabled",new ArrayList<>().toArray(new String[0]),false))),new String[]{"enabled"}));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition(new ToolFunction("setKioskTimeout",
      //        "You update the configured kiosk mode timeout",
      //        List.of(new ToolParameterHelper("duration", "integer", new ArrayList(), "The number of timeunits defining the timeout", true),
      //                new ToolParameterHelper("timeunit", "string", Arrays.stream(TimeUnit.values()).map(Objects::toString).toList(), "The timeunit of the duration to set ", true)
      //        ));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition(new ToolFunction("setKioskGoal",
      //        "You update the configured kiosk mode goal",
      //        List.of(new ToolParameterHelper("duration", "integer", new ArrayList(), "The number of timeunits defining the timeout", true),
      //                new ToolParameterHelper("timeunit", "string", Arrays.stream(TimeUnit.values()).map(Objects::toString).toList(), "The timeunit of the duration to set ", true)
      //        ));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //// Focused contexts
      definition = new ToolDefinition(new ToolFunction("startSchedulingTask",
              "You begin a more focused dialogue interaction with the user to schedule future or regularly repeating execution of a task. \nExample Instruction: direct like \"I want to schedule a goal\", or indirect like saying they want you to perform a certain action in the future or on an interval"));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition(new ToolFunction("learnTaskSequence",
      //        "You begin a more focused dialogue interaction with the user to learn how to perform a new \"task\", \"action\", or \"tour\" sequence. The learned task sequence will consist of navigating between a set of saved locations while performing actions at each location. \nExample Instruction: \"Let me tell you how to perform a new task\" or the user beginning to define an action sequence for you to save"));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition(new ToolFunction("startLearningNewTask",
              "You begin a more focused dialogue interaction with the user to learn how to perform a new \"task\" or \"action\" The learned task will consist of performing a specific sequence of actions in order. \nExample Instruction: \"Let me tell you how to perform a new task\" or the user beginning to define an action for you to save."));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition(new ToolFunction("modifyExistingTaskStatus",
              "You begin a dialogue interaction with the user to pause, cancel, or resume existing assigned tasks. Examples of when this is relevant could be when a user tells you to forget about, stop, or pick back up with what you are doing."));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //// Functionality temporarily removed whether due to lack of robust implementation or unsure if wanting to expose through flexible language
      //definition = new ToolDefinition(new ToolFunction("deliver(?objectName, ?location:location)",
      //        "You deliver ?objectName to location ?location:location",
      //        "bring the mug to "+promptVars.get("%exampleLocation")+"",
      //        "deliver(mug,"+promptVars.get("%exampleLocation")+")", 10);
      //toolDefinitions.add(definition);
      //    definition = new ToolDefinition(new ToolFunction("enableAdminMode",
      //            "Allow certain actions which require administrator privileges to be instructed through voice directly to the robot.. \nExample Instruction: \"enable admin mode.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition(new ToolFunction("disableAdminMode",
      //            "Reject certain actions which require administrator privileges from being instructed through voice directly to the robot.. \nExample Instruction: \"disable admin mode.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    TODO: this target should be something saying to refer to the website
      //    definition = new ToolDefinition(new ToolFunction("switchDisplayToTelepresence",
      //            "Change the output on your display to the telepresence stream. \nExample Instruction: \"start telepresence.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));


      ////Removed in favor of goal manipulation focused context attempt
      //definition = new ToolDefinition(new ToolFunction("cancelCurrentTask",
      //        "You cancel and fully discard your current goal, whether it is active or suspended.",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition(new ToolFunction("suspendCurrentTask",
      //        "You pause your current goal, which can be later resumed or canceled",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition(new ToolFunction("resumeCurrentTask",
      //        "You resume your current goal if it is suspended",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition(new ToolFunction("cancelSingleTaskInQueue",
      //        "You cancel and remove a specific goal in the queue specified by its index",
      //        new LinkedHashMap<>(Map.of("index",new ToolParameterV2("integer","The 0-based index of the task in the queue to remove"))),new String[]{"index"}));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition(new ToolFunction("cancelAllQueuedTasks",
      //        "You cancel all goals currently in the pending queue.",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition(new ToolFunction("cancelAllCurrentAndQueuedTasks",
      //        "You cancel both your current goal and all goals currently in the pending queue",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition(new ToolFunction("cancelSingleScheduledGoal",
      //        "You cancel a particular scheduled goal, meaning one tasks which has been scheduled to occur at a specific time in the future",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition(new ToolFunction("cancelAllScheduledGoals",
      //        "You cancel all scheduled goals, meaning all tasks which have been scheduled to occur at a specific time in the future",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

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
   //TODO: somehow clearly label this section of tools as ITK tools
   public List<Tool> generateITKTools() {
      List<Tool> itkTools = new ArrayList<>();
      ToolDefinition definition;

      //// Domain specific ITK
      definition = new ToolDefinition(new ToolFunction("getPreviouslyMeasuredWifiAtLocationsString",
              "Get previously measured wifi signal strengths of a set of supplied :locations",
              new LinkedHashMap<>(Map.of("locations",new ToolParameterV2("array","The list of saved :locations to report wifi signals for. Empty if the current location only",getLocations("self",true).toArray(new String[0]),false)))));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      if (includeDemoTargets) {
         definition = new ToolDefinition(new ToolFunction("getPreviouslyMeasuredTemperatureAtLocationsString",
                 "Get previously measured temperature readings of a set of supplied :locations",
                 new LinkedHashMap<>(Map.of("locations",new ToolParameterV2("array","The list of saved :locations to report the temperature for. Empty if the current location only",getLocations("self", true).toArray(new String[0]),false)))));
         itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      }

      // TODO: revisit which of these should actually be kept and work well.
      //// General "meta-level" ITK
      definition = new ToolDefinition(new ToolFunction("getInformationAroundTeachingNewActions",
              "Use this to get information from the system about how a user can start teaching you new tasks, the process of teaching a new task, and what actions can be taught as part of a new task. This information could be used to answer questions from the user. An example of when this may be useful to call is when a user asks things like \"how do I teach you a new task\", \"what information do I need to teach a new task\", \"how do I instruct something I've taught you\", or \"what can I tell you to do while teaching a new task\"."));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition(new ToolFunction("getInformationAroundHowToTeachATask",
      //        "Use this to get an explanation from the system on how a user can teach you new tasks, which you could use to answer questions from the user. If the supplied information is not relevant to the user's question, you do not need to address it. An example of when this may be useful to call is when a user asks \"how do I teach you a new task\".",
      //        new ArrayList<>());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition(new ToolFunction("getInformationAroundWhatALearnedTaskIs",
      //        "Use this to get an explanation from the system on what a taught task consists of, which you could use to answer questions from the user. If the supplied information is not relevant to the user's question, you do not need to address it. An example of when this may be useful to call is when a user asks \"what information do I need to teach a new task\" or \"how do I instruct something I've taught you\".",
      //        new ArrayList<>());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition(new ToolFunction("getInformationAroundWhatActionsCanBePerformedAlongALearnedTask",
      //        "Use this to get an explanation from the system on what actions you can be asked to perform as part of a taught task, which you could use to answer questions from the user. If the supplied information is not relevant to the user's question, you do not need to address it. An example of when this may be useful to call is when a user asks \"what can I tell you to do while teaching a new task\".",
      //        new ArrayList<>());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition(new ToolFunction("getAssistanceOnUsingSaveLocationAndGoToLocationProperly",
              "Use this to get information from the system about navigation, saved locations, the map, and how to learn new locations."));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));

      //TODO: move to base class
      //// Domain independent ITK
      definition = new ToolDefinition(new ToolFunction("getTaughtActionsList",
              "Use this to ask the system for a list of all of the actions which have been taught to you by a user."));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      String exampleCallString = "<function=getActionSteps><parameter=action>escort</parameter></function>";
      definition = new ToolDefinition(new ToolFunction("getActionSteps",
              "Use this to ask the system for a description of an action, in the form of the list the steps which would be performed when it is executed. ?action can be replaced by populated JSON representing another tool OR be just the name of another tool if no specific arguments are supplied. An example of when you may need this information is when a user asks \"How would you escort someone.\" \nExample Call: " + exampleCallString,
              new LinkedHashMap<>(Map.of("action",new ToolParameterV2("string","The tool whose execution steps to describe"))),new String[]{"action"}));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition(new ToolFunction("checkPastInteractionHistoryForMissingInformation",
              "Use this when you believe you are missing important context necessary to understand user input. For example, when the user explicitly references a past conversation or says something implying common ground you have no memory of."));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));


      //TODO: keep aligned with above whether including kiosk in language or not.
      //      Revisit when focused context implemented - include in system prompt vs only as focused vs as separate itk tool here
      definition = new ToolDefinition(new ToolFunction("getKioskGoalSettings",
                      "Use this to ask the system for the current configuration of kiosk mode; Whether it is enabled, the currently configured kiosk timeout, and the kiosk action. Kiosk mode refers to default tasks to perform after periods of inactivity."));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));

      //// Removed
      //TODO: make sure this distinction is possible to tease out from the LLM just from information in the prompt
      //definition = new ToolDefinition(new ToolFunction("getPredefinedActions",
      //        "Use this to ask the system for the list of actions a user is able to instruct you through natural language to perform. An example of when you may need this information is when a user asks \"what can you be instructed to do.\"",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION));

      //// Info moved to be included in system message automatically
      definition = new ToolDefinition(new ToolFunction("getBatteryLevelString",
              "Get the current battery level percent of the robot"));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition(new ToolFunction("getCurrentLocationString",
              "Get the current location you believe you are at"));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition(new ToolFunction("getAllKnownLocations",
              "Get a list of all of your currently saved locations"));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition(new ToolFunction("getVideoVolume",
              "Get the current volume videos are set to play at"));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition(new ToolFunction("getSpeakingVolume",
              "Get the current volume videos are set to play at"));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition(new ToolFunction("getSavedVideos",
              "Get the list of saved videos in your collection which you are able to play"));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition(new ToolFunction("getSavedImages",
              "Get the list of saved images in your collection which you are able to display"));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      return itkTools;
   }

   /**
    * Generate all possible REPLY UtteranceType tools available for this domain. Will be included in any LLM Completion
    * request dynamically based on matching expected response forms for current system questions.
    */
   public Map<String,Tool> generateReplyTools() {
      Map<String,Tool> replyTools = new HashMap();
      ToolDefinition definition;
      List<String> locationNames = new ArrayList<>(getLocations("self", true));
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