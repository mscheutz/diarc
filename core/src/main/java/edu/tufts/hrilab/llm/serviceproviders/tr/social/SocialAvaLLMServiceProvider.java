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
import edu.tufts.hrilab.llm.gson.tool.ToolParameter;
import edu.tufts.hrilab.llm.openai.ToolParameterHelper;
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
import java.util.stream.Collectors;

public class SocialAvaLLMServiceProvider extends SocialTRLLMServiceProvider {
   // Temperature measurements and reporting
   private boolean includeDemoTargets = false;

   // Revisit when making base/subclass distinction
   protected Prompt getNluSystemPrompt() {
      if (useCondensedPrompts) {
         return Prompts.getPrompt("socialava/condensed/nlu");
      } else {
         return Prompts.getPrompt("socialava/nlu");
         //    return Prompts.getPrompt("socialava/nlu_toolcalls_and_content");
      }
   }

   protected Prompt getNlgSystemPrompt() {
      if (useCondensedPrompts) {
         return Prompts.getPrompt("socialava/condensed/nlg");
      } else {
         return Prompts.getPrompt("socialava/nlg");
      }
   }

   protected Prompt getValidationNLGSystemPrompt() {
      if (useCondensedPrompts) {
         return Prompts.getPrompt("socialava/condensed/validation");
      } else {
         return Prompts.getPrompt("socialava/validation");
      }
   }

   protected Prompt getValidationNLUSystemPrompt() {
      if (useCondensedPrompts) {
         return Prompts.getPrompt("socialava/condensed/validationnlu");
      } else {
         return Prompts.getPrompt("socialava/validationnlu");
      }
   }

   @Override
   protected Prompt getGoalTranslationPrompt() {
      return Prompts.getPrompt("socialava/goalPredToString");
   }

   @Override
   public void init() {
      super.init();

      try {
         TRADE.requestNotification(this, "joined", new TRADEServiceConstraints().name("getKBName"),null, "newConsultantCallback");
      } catch (TRADEException e) {
         log.error("[init] error registering for TRADE notification for new consultant join");
      }

      //// init prompt vars
      // Generating string representations of toolset as reference for NLG and scheduleTask (want to present them, but not actually supply them as targets)
      // TODO: move into initPromptVars
      List<Tool> instructTools = generateInstructTools(false);
      //TODO: need general way to provide instruct versions of actions for teaching, scheduling, etc vs ITK versions of
      //  actions for general interaction to populate corresponding toolsets effectively
      if (!useCondensedTools) {
         ToolDefinition definition = new ToolDefinition("reportPreviouslyMeasuredWifiOfLocations",
                 "You report the wifi signal strength you had previously measured at each of the supplied locations in the list. Only returns a useful value for each value after wifi has been measured for that location. An example of when you may need this information is when a user asks \"how was the signal at %exampleLocation, example_location_2, and example_location_3?\"",
                 List.of(new ToolParameterHelper("locations", "array", getLocations("self", true), "The ordered list of saved locations to report the wifi signal for. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)));
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
         definition = new ToolDefinition("reportPreviouslyMeasuredWifiOfCurrentLocation",
                 "You report the the wifi signal strength of your current location. An example of when you may need this information is when a user asks \"how is the signal where you are now\"",
                 new ArrayList<>());
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
         if (includeDemoTargets) {
            definition = new ToolDefinition("reportPreviouslyMeasuredTemperatureOfLocations",
                    "You report the temperature (in degrees Fahrenheit) you previously measured at each of the supplied locations in the list. Only returns a useful value after measureTemperatureAtLocations has been run for those location. An example of when you may need this information is when a user asks \"how warm was it at %exampleLocation?, example_location_2, and example_location_3?\"",
                    List.of(new ToolParameterHelper("locations", "array", getLocations("self", true), "The ordered list of saved locations to report the temperature for. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)));
            instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
            definition = new ToolDefinition("reportPreviouslyMeasuredTemperatureOfCurrentLocation",
                    "You report the the temperature is at your current location. An example of when you may need this information is when a user asks \"what is the temperature where you are now\"",
                    new ArrayList<>());
            instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
         }
         definition = new ToolDefinition("sayLocally",
                 "The speaker on the robot will emit the supplied phrase. Do not use this when you want to respond to the user. This should be targeted when a user is explicitly instructing you to say something out loud in person, most commonly in the context of another task, to another person who is not the user, or at a later time. \n- Example instruction: \"say to everyone: welcome to the event!.\" \n- Example Call: <function=sayLocally><parameter=words>\"welcome to the event!\"</parameter></function>\"" ,
                 List.of(new ToolParameterHelper("words", "string", new ArrayList(), "The phrase to be said. This should be exactly what the user instructed you to say, not paraphrasing.", true)));
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
         definition = new ToolDefinition("stopFollowing",
                                         "You stop following the user. \nExample Instruction: \"stop following me.\"",
                                         new ArrayList());
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      } else {
         ToolDefinition definition = new ToolDefinition("reportPreviouslyMeasuredWifiOfLocations",
                 "You report the wifi signal strength you had previously measured at certain :locations",
                 List.of(new ToolParameterHelper("locations", "array", getLocations("self", true), "The list of :locations to report the wifi signal of. Empty if the current location at the time of this goal being performed", true)));
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
         if (includeDemoTargets) {
            definition = new ToolDefinition("reportPreviouslyMeasuredTemperatureOfLocations",
                    "You report the temperature you had previously measured at certain :locations",
                    List.of(new ToolParameterHelper("locations", "array", getLocations("self", true), "The list of :locations to report the temperature of. Empty if the current location at the time of this goal being performed", true)));
            instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
         }
         definition = new ToolDefinition("sayLocally",
                 "The speaker on the robot will emit the supplied phrase",
                 List.of(new ToolParameterHelper("words", "string", new ArrayList(), "The exact phrase instructed to be said", true)));
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
         definition = new ToolDefinition("stopFollowing",
                 "You stop following the user",
                 new ArrayList());
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      }

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
      //if (useCondensedTools) {
      //   scheduleGoalTools.add(new Tool(new ToolDefinition("submitScheduledTaskDefinition",
      //                                                     "Save a fully defined scheduled task.", List.of(
      //         new ToolParameterHelper("action", "object", new ArrayList<>(), "The action to be performed. Must be a JSON definition with name and arguments itself.", true, true, "nluextended"),
      //         new ToolParameterHelper("start", "string", new ArrayList<>(), "When to first execute the task. Valid forms are a relative time represented by start(<number>,<timeunit>) or an absolute time represented by start(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss).", true),
      //         new ToolParameterHelper("repeat", "string", new ArrayList<>(), "On what interval to repeat execution of the task. Valid forms are a relative time represented by repeat(<number>,<timeunit>). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. If the task should only be performed once and not repeated, supply a value of repeat(nonrepeating).", true),
      //         new ToolParameterHelper("end", "string", new ArrayList<>(), "When to stop repeating execution of the task. If this task is not repeating (meaning repeat is defined as repeat(nonrepeating)), you must supply the value end(nonrepeating). Otherwise, valid forms are a relative time represented by end(<number>,<timeunit>) or an absolute time represented by end(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss).", true)
      //   )),
      //                                  UtteranceType.INSTRUCT
      //   ));
      //   Tool modifiedTextResponseTool1 = new Tool(new ToolDefinition("stillGatheringInformation",
      //                                                                "You are still being taught new or modifying existing details of the scheduled task by the user. You will respond to" +
      //                                                                "the user to acknowledge reception of their most recent command, answer questions they have, or continue" +
      //                                                                "to gather missing information in order to define the scheduled task.",
      //                                                                List.of(new ToolParameterHelper("response", "string", new ArrayList<>(),
      //                                                                                                "What you want to say to the user", true),
      //                                                                        new ToolParameterHelper("currentScheduledDefinition", "object", new ArrayList<>(),
      //                                                                                                "Your current understanding of the scheduled task being instructed to you. The expected format is json matching the submitScheduledTaskDefinition function described above.", true)
      //                                                                )),
      //                                             UtteranceType.INSTRUCT);
      //   scheduleGoalTools.add(modifiedTextResponseTool1);
      //} else {
         scheduleGoalTools.add(new Tool(new ToolDefinition("submitScheduledTaskDefinition",
                                                           "Submit the details of the fully defined scheduled task for execution. Call this when you believe you have all information about the scheduled task finalized and are ready to save.", List.of(
               new ToolParameterHelper("action", "object", new ArrayList<>(), "The action to be performed. Please refer to the Action List in order to understand acceptable values and forms.", true, true, "nluextended"),
               new ToolParameterHelper("start", "string", new ArrayList<>(), "When to first execute the task. Valid forms are a relative time represented by start(<number>,<timeunit>) or an absolute time represented by start(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss). For example, the relative time of 2 hours from now == start(2,HOURS). The absolute time of noon == start(12-00-00). Being told to perform the first execution immediately == start(0,SECONDS).", true),
               new ToolParameterHelper("repeat", "string", new ArrayList<>(), "On what interval to repeat execution of the task. Valid forms are a relative time represented by repeat(<number>,<timeunit>). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. For example, the relative time of 30 minutes from now == repeat(30,MINUTES). If the task should only be performed once and not repeated, supply a value of repeat(nonrepeating).", true),
               new ToolParameterHelper("end", "string", new ArrayList<>(), "When to stop repeating execution of the task. If this task is not repeating (meaning repeat is defined as repeat(nonrepeating)), you must supply the value end(nonrepeating). If end(indefinitely) is provided as the value, that means the task will continue repeating execution on the defined repeat until it is explicitly canceled by the user. Otherwise, valid forms are a relative time represented by end(<number>,<timeunit>) or an absolute time represented by end(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss). For example, the relative time of in a quarter of an hour == end(15,MINUTES). The absolute time of 5:30pm == end(17-30-00).", true)
         )),
                                        UtteranceType.INSTRUCT
         ));
         Tool modifiedTextResponseTool1 = new Tool(new ToolDefinition("stillGatheringInformation",
                                                                      "You are still being taught new or modifying existing details of the scheduled task by the user. You will respond to" +
                                                                      "the user while following the Response Rules and Style Requirements.",
                                                                      List.of(new ToolParameterHelper("response", "string", new ArrayList<>(),
                                                                                                      "What you want to say to the user", true),
                                                                              new ToolParameterHelper("currentScheduledDefinition", "object", new ArrayList<>(),
                                                                                                      "Your current understanding of the scheduled task being instructed to you. The expected format is json matching the submitScheduledTaskDefinition function described above.", true)
                                                                      )),
                                                   UtteranceType.INSTRUCT);
         scheduleGoalTools.add(modifiedTextResponseTool1);
      //}
      if (useCondensedPrompts) {
         chatTemplates.put("scheduleTask", new ChatTemplates(Prompts.getPrompt("socialava/condensed/scheduleGoal"), scheduleGoalTools));
      } else {
         chatTemplates.put("scheduleTask", new ChatTemplates(Prompts.getPrompt("socialava/scheduleGoal"), scheduleGoalTools));
         //    chatTemplates.put("scheduleTask", new ChatTemplates(Prompts.getPrompt("socialava/schedulegoal_splitresponse"), scheduleGoalTools));
      }
      // Tour Teaching
      // Using custom "stillGatheringInformation" tool which also explicitly keeps track of the "current template parse" at
      //  each step. The idea is that writing it down each step provides something concretely to validate each step and
      //  better context for incremental LLM population. This could be first application/attempt of "general template
      //  population" structure, which we may want to use for at least general action learning if not every context.
      List<Tool> learnTourTools = new ArrayList<>();
      //if (useCondensedTools) {
      //   learnTourTools.add(new Tool(new ToolDefinition("submitTaskSequenceDefinition",
      //                                                  "Save a compelted task sequence definition", List.of(
      //         new ToolParameterHelper("taskName", "string", new ArrayList<>(), "Task name", true),
      //         new ToolParameterHelper("locationActionPairs", "array", new ArrayList<>(), "Ordered location-action pairs", true, true, "nluextended")
      //   )),
      //                               UtteranceType.INSTRUCT
      //   ));
      //   Tool modifiedTextResponseTool2 = new Tool(new ToolDefinition("stillGatheringInformation",
      //                                                                "Reply briefly while gathering missing information.",
      //                                                                List.of(new ToolParameterHelper("response", "string", new ArrayList<>(),
      //                                                                                                "Short spoken reply", true),
      //                                                                        new ToolParameterHelper("currentTaskSequenceDefinition", "object", new ArrayList<>(),
      //                                                                                                "Your current understanding of the steps of the task being taught to you, same format as locationActionPairs.", true)
      //                                                                )),
      //                                             UtteranceType.INSTRUCT);
      //   learnTourTools.add(modifiedTextResponseTool2);
      //} else {
         learnTourTools.add(new Tool(new ToolDefinition("submitTaskSequenceDefinition",
                                                        "Submit the details of the fully defined task sequence sequence for storage and future use. Call this when you believe you have all information about the task finalized and are ready to save.", List.of(
               new ToolParameterHelper("taskName", "string", new ArrayList<>(), "The name this task should be saved under and later instructed as", true),
               new ToolParameterHelper("locationActionPairs", "array", new ArrayList<>(), "The list of action/location pairs defining this task sequence. Execution of this task will consist of iterating over the pairs in this list, visiting each location in sequence, and then performing the corresponding task at that location. Each individual value in this list should be a json object following the form {\"location\":<location_name_here>, \"action\":<tool_call_definition_here>}, where the location field must contain a saved location and the action field must contain a valid tool call definition matching a form in the supplied actionList.", true, true, "nluextended")
         )),
                                     UtteranceType.INSTRUCT
         ));
         Tool modifiedTextResponseTool2 = new Tool(new ToolDefinition("stillGatheringInformation",
                                                                      "You are still being taught new or modifying existing details of the task sequence by the user. You will respond to" +
                                                                      "the user while following the Response Rules and Style Requirements.",
                                                                      List.of(new ToolParameterHelper("response", "string", new ArrayList<>(),
                                                                                                      "What you want to say to the user. Generate this argument first.", true),
                                                                              new ToolParameterHelper("currentTaskSequenceDefinition", "object", new ArrayList<>(),
                                                                                                      "Your current understanding of the steps of the task being taught to you. The expected format is json matching the submitTaskSequenceDefinition function described above. Generate this argument after your response.", true)
                                                                      )),
                                                   UtteranceType.INSTRUCT);
         learnTourTools.add(modifiedTextResponseTool2);
      //}
      if (useCondensedPrompts) {
         chatTemplates.put("learnTaskSequence", new ChatTemplates(Prompts.getPrompt("socialava/condensed/learnTour"), learnTourTools));
      } else {
         chatTemplates.put("learnTaskSequence", new ChatTemplates(Prompts.getPrompt("socialava/learnTour"), learnTourTools));
         //chatTemplates.put("learnTaskSequence", new ChatTemplates(Prompts.getPrompt("socialava/learntour_splitresponse"), learnTourTools));
      }

      // Goal status modification
      List<Tool> modifyExistingTaskStatusTools = new ArrayList<>();
      modifyExistingTaskStatusTools.add(new Tool(new ToolDefinition("submitTaskStatusModificationDefinition",
              "Submit the details of the fully defined task status modification for application.", List.of(
              new ToolParameterHelper("modification", "string", List.of("cancel","suspend","resume"), "The type of status modification to apply", true),
              new ToolParameterHelper("taskType", "string", List.of("current","queued","scheduled"), "Which task collection the desired task to modify is found in", true),
              new ToolParameterHelper("taskIndex", "array", new ArrayList<>(), "The index(es) of desired task(s) to modify within the corresponding task collection. May be empty or [0] in the case of a 'current' taskType.", true)
      )),
              UtteranceType.INSTRUCT
      ));
      modifyExistingTaskStatusTools.add(new Tool(new ToolDefinition("stillGatheringInformation",
              "You are still being told new or modifying existing details of the task status modification by the user. You will respond to" +
                      "the user while following the Response Rules and Style Requirements.",
              List.of(new ToolParameterHelper("response", "string", new ArrayList<>(),
                              "What you want to say to the user", true),
                      new ToolParameterHelper("currentTaskStatusModificationDefinition", "object", new ArrayList<>(),
                              "Your current understanding of the task status modification being instructed to you. The expected format is json matching the submitTaskStatusModificationDefinition function described above.", true)
              )),
              UtteranceType.INSTRUCT));
      //TODO: when we have more time - modify the template to allow for these behaviors
      modifyExistingTaskStatusTools.add(new Tool(new ToolDefinition("cancelAllCurrentAndQueuedTasks",
              "Cancel the current task and all tasks in the queue. Only use this when the user wants to cancel all assigned tasks, as the standard taskStatusModification template is not flexible enough to represent this.",
              new ArrayList<>()),
              UtteranceType.INSTRUCT
      ));
      chatTemplates.put("modifyExistingTaskStatus", new ChatTemplates(Prompts.getPrompt("socialava/condensed/focused/goalStatusManipulation"), modifyExistingTaskStatusTools));

      return chatTemplates;
   }

   /**
    * Initialize Prompt variables whose values are static during runtime or updated by event-based listeners
    */
   protected Map<String, String> initPromptVars() {
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
      vars.put("%exampleVideo", videos.isEmpty() ? "test_video" : videos.get(0));
      vars.put("%exampleImage", images.isEmpty() ? "test_image" : images.get(0));
      vars.put("%imageList", imageList);
      vars.put("%exampleLocation", locations.isEmpty() ? "test_location:location" : locations.get(0));
      vars.put("%language", language);

      if (useCondensedTools) {
         vars.put("%toolArgumentTypeDescription","- Any tool call parameter below which references a :location, :video, or :image can ONLY take values from the corresponding Location List, Video List, or Image List above. Values not present in the lists must be rejected\\n-- For locations specifically, the values MUST include the :location suffix. For example, " + vars.get("%exampleLocation") + ".\\n- If asked to go to a location which does not exist, call the \"getAssistanceOnUsingSaveLocationAndGoToLocationProperly\" tool to learn how to save new locations.\\n- New videos and images can be added through the associated website.");
      } else {
         vars.put("%toolArgumentTypeDescription","");
      }
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
   }

   private String getBatteryLevel() {
      String batteryLevel = "unknown";
      try {
         Double batteryLevelDouble = TRADE.getAvailableService(new TRADEServiceConstraints().name("getBatteryLevel").argTypes()).call(Double.class);
         if (batteryLevelDouble != null) {
            batteryLevel = String.valueOf(batteryLevelDouble);
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
                  argsWithActor.addAll(((Term) semantics.get(1)).getArgs());
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

   //TODO: remove
   @Override
   protected Symbol handleLocationActionPairs(String argumentName, Map<String,Object> toolDefinitionMap, ToolParameter argumentDefinition, ToolDefinition parentTool, Symbol actor, Utterance.Builder builder, Map<Symbol,Variable> varMap, StringBuilder errorMessage, boolean unpopulatedArgSafe) {
        String locationVal = (String)toolDefinitionMap.get("location");
        Object actionVal = toolDefinitionMap.get("action");
        Symbol locationSymbol = null;
        Symbol actionSymbol = null;
        if (locationVal == null) {
          log.warn("[processArgument] locationActionPair does not have location field present");
          errorMessage.append("- You supplied json for a \"").append(argumentName).append("\" element of the \"").append(parentTool.function.name).append("\" call, but it was missing a \"location\" field. Each locationPair needs a \"location\" field.\n");
        } else {
          locationSymbol = processSymbolArgument(argumentName, locationVal, getLocations("self",true),errorMessage,varMap,builder,false,unpopulatedArgSafe);
        }
        if (actionVal == null) {
          log.warn("[processArgument] locationActionPair does not have action field present");
          errorMessage.append("- You supplied json for a \"").append(argumentName).append("\" element of the \"").append(parentTool.function.name).append("\" call, but it was missing an \"action\" field. Each locationPair needs an \"action\" field.\n");
        } else {
          actionSymbol = processToolArgument("inner location action pair's action", actionVal, argumentDefinition, parentTool,actor, builder,varMap,errorMessage,unpopulatedArgSafe);
        }
        if (actionSymbol == null || locationSymbol == null) {
          return null;
        } else {
         return Factory.createPredicate("and",locationSymbol,actionSymbol);
        }
   }

   /**
    * Generate the base INSTRUCT UtteranceType toolset available for this domain
    */
   //TODO: Apply prompt variables dynamically (e.g. update location lists)
   //TODO: Think of a better way to define / gather this information
   //TODO: rephrase descriptions to align better with final "social" system prompt
   public List<Tool> generateInstructTools(boolean xml) {
      if (useCondensedTools) {
         return generateInstructToolsCondensed(xml);
      }

      List<Tool> instructTools = new ArrayList<>();
      ToolDefinition definition;
      List<String> locationNames = getLocations("self", true);

      String exampleCallString = xml ?
            "<function=goToLocation><parameter=destination>%exampleLocation</parameter></function>." :
            "{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}.";
      definition = new ToolDefinition("goToLocation",
                                      "You go to the supplied destination location.\n- Example instruction: \"please navigate to %exampleLocation\" \n- Example Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("destination", "string", locationNames, "The destination location to go to. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true))
      );
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("follow",
                                      "You follow the user. \n- Example instruction: \"follow me.\"",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //TODO: could also turn off validation for this? (only for local user?)
      exampleCallString = xml ?
            "<function=sayLocally><parameter=words>\"welcome to the event!\"</parameter></function>\"" :
            "{\"name\":\"say\", \"arguments\":{\"words\":\"welcome to the event\"}}";
      definition = new ToolDefinition("sayLocally",
                                      "The speaker on the robot will emit the supplied phrase. Do not use this when you want to respond to the user. This should be targeted when a user is explicitly instructing you to say something out loud in person, most commonly in the context of another task, to another person who is not the user, or at a later time. \n- Example instruction: \"say to everyone: welcome to the event!.\" \n- Example Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("words", "string", new ArrayList(), "The phrase to be said. This should be exactly what the user instructed you to say, not paraphrasing.", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("stopFollowing",
      //                                "You stop following the user. \nExample Instruction: \"stop following me.\"",
      //                                new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      exampleCallString = xml ?
            "<function=escortToLocation><parameter=name>owen</parameter><parameter=destination>%exampleLocation</parameter></function>" :
            "{\"name\":\"escortToLocation\", \"arguments\":{\"name\":\"owen\",\"destination\":\"%exampleLocation\"}}";
      definition = new ToolDefinition("escortToLocation",
                                      "You lead someone from your current location to a different one. \nExample Instruction: \"bring Owen to %exampleLocation.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("name", "string", new ArrayList(), "Name of the entity to escort", true),
                                              new ToolParameterHelper("destination", "string", locationNames, "The destination location to escort to. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)
                                      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      exampleCallString = xml ?
            "<function=escortFromLocationToLocation><parameter=name>guest</parameter><parameter=start>%exampleLocation</parameter><parameter=destination>dock:location</parameter></function>" :
            "{\"name\":\"escortFromLocationToLocation\", \"arguments\":{\"name\":\"guest\",\"start\":\"%exampleLocation\",\"destination\":\"dock:location\"}}";
      definition = new ToolDefinition("escortFromLocationToLocation",
                                      "You meet someone at the specific start location and then lead them to the destination location. \nExample Instruction: \"bring our guest from %exampleLocation to the dock.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("name", "string", new ArrayList(), "Name of the entity to escort", true),
                                              new ToolParameterHelper("start", "location", locationNames, "The start location to escort from. This should be where ?name can be found", true),
                                              new ToolParameterHelper("destination", "location", locationNames, "The destination location to escort to", true)
                                      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      exampleCallString = xml ?
            "<function=saveLocation><parameter=name>location one</parameter></function>":
            "{\"name\":\"saveLocation\", \"arguments\":{\"name\":\"location one\"}}";
      definition = new ToolDefinition("saveLocation",
                                      "You save your current location under the supplied name. \nExample Instruction: \"save this area as location one.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("name", "string", new ArrayList(), "the label for the location to save", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("goHome",
                                      "You go to your default charging dock and charge. \nExample Instruction: \"go back to charge.\"",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      exampleCallString = xml ?
            "<function=measureWifiAtCurrentLocation></function>}" :
            "{\"name\":\"measureWifiAtCurrentLocation\", \"arguments\":{}}";
      definition = new ToolDefinition("measureWifiAtCurrentLocation",
                                      "You measure the wifi signal strength at your current location when this task is performed. Use this when a user asks you to survey or check wifi without any additional specification. \nExample Instruction: \"check the wifi signal strength\" \nExample Call: " + exampleCallString,
                                      new ArrayList<>());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      exampleCallString = xml ?
            "<function=measureWifiAtLocations><parameter=locations>[%exampleLocation,dock:location]</parameter></function>" :
            "{\"name\":\"measureWifiAtLocations\", \"arguments\":{\"locations\":[\"%exampleLocation\",\"dock:location\"]}}";
      definition = new ToolDefinition("measureWifiAtLocations",
                                      "You drive to each of the supplied locations in order and measure wifi signal strength at each one. Use this when a user asks you to survey or check wifi at a set of places. For a single location, still supply a one-element list. \nExample Instruction: \"check the wifi at %exampleLocation and the dock\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("locations", "array", locationNames, "The ordered list of saved locations to visit and measure. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("measureWifiAtLocationsAndReportAt",
                                      "You drive to each of the supplied locations in order, measure wifi at each one, then drive to reportLocation and summarize the findings. Use this when the user asks to measure and report wifi at the same time.",
                                      List.of(new ToolParameterHelper("locations", "array", locationNames, "The ordered list of saved locations to visit and measure. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true),
                                              new ToolParameterHelper("reportLocation", "string", locationNames, "The saved location to return to before speaking the summary. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      // Hard to accurately target, biased to prefer measure rather than report
      // Bad for NLG
      //definition = new ToolDefinition("measureAndOrReportWifiStrengths",
      //        "You measure and/or report the wifi signal strength for any number of locations.\n" +
      //                "Possible variations -> Example:\n" +
      //                "- Measure the wifi strength at your current location -> <tool_call><function=measureAndOrReportWifiStrengths></function></tool_call>" +
      //                "- Measure the wifi strength at exampleLocation1 and exampleLocation2 -> <tool_call><function=measureAndOrReportWifiStrengths><parameter=measuredLocations>[exampleLocation1, exampleLocation2]</parameter></function></tool_call>\n" +
      //                "- Report previously measured wifi strengths of exampleLocation1 and exampleLocation3. This is done where you are located when executed. -> <tool_call><function=measureAndOrReportWifiStrengths><parameter=reportedLocations>[exampleLocation1, exampleLocation3]</parameter></function></tool_call>\n" +
      //                "- Measure wifi strength at exampleLocation2 and exampleLocation5. Then report both measured values where you are located when finished. -> <tool_call><function=measureAndOrReportWifiStrengths><parameter=measuredLocations>[exampleLocation2, exampleLocation5]</parameter><parameter=reportedLocations>[exampleLocation2, exampleLocation5]</parameter></function></tool_call>\n" +
      //                "- Measure wifi strength at exampleLocation2 and exampleLocation5. Then return to exampleLocation1 and report both measured values -> <tool_call><function=measureAndOrReportWifiStrengths><parameter=measuredLocations>[exampleLocation2, exampleLocation5]</parameter><parameter=reportedLocations>[exampleLocation2, exampleLocation5]</parameter><parameter=reportAtLocation>exampleLocation1</parameter></function></tool_call>\n" +
      //                "- Measure wifi strength at exampleLocation2 and exampleLocation5. Then return to exampleLocation1 and report both measured values -> <tool_call><function=measureAndOrReportWifiStrengths><parameter=measuredLocations>[exampleLocation2, exampleLocation5]</parameter><parameter=reportAtLocation>exampleLocation1</parameter></function></tool_call>\n" +
      //                "- Report previously measured wifi strengths of exampleLocation5 and exampleLocation6. This is dont at exampleLocation3 -> <tool_call><function=measureAndOrReportWifiStrengths><parameter=reportedLocations>[exampleLocation5, exampleLocation6]</parameter><parameter=reportAtLocation>exampleLocation3</parameter></function></tool_call>\n" +
      //                "",
      //        List.of(new ToolParameterHelper("measuredLocations", "array", new ArrayList<>(), "The ordered list of saved locations to visit and measure.", false),
      //                new ToolParameterHelper("reportedLocations", "string", new ArrayList<>(), "The ordered list of saved locations to report measured wifi for.", false),
      //                new ToolParameterHelper("reportAtLocation", "string", new ArrayList<>(), "Where to report all measured results.", false)));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //"- <tool_call><function=measureAndOrReportWifiStrengths><parameter=measuredLocations>[]</parameter></function></tool_call>",


      if (includeDemoTargets) {
         exampleCallString = xml ?
                 "<function=measureTemperatureAtCurrentLocation></function>" :
                 "{\"name\":\"measureTemperatureAtCurrentLocation\", \"arguments\":{}}";
         definition = new ToolDefinition("measureTemperatureAtCurrentLocation",
                 "You measure the temperature in degrees Fahrenheit at your current location when this task is performed. Use this when a user asks you to check temperature without any additional specification. \nExample Instruction: \"check the temperature\" \nExample Call: " + exampleCallString,
                 new ArrayList<>());
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
         exampleCallString = xml ?
                 "<function=measureTemperatureAtLocations><parameter=locations>[%exampleLocation]</parameter></function>" :
                 "{\"name\":\"measureTemperatureAtLocations\", \"arguments\":{\"locations\":[\"%exampleLocation\"]}}";
         definition = new ToolDefinition("measureTemperatureAtLocations",
                 "You drive to each of the supplied locations in order and measure temperature in degrees Fahrenheit at each one. Use this when a user asks you to survey or check temperature at a set of places. For a single location, still supply a one-element list. \nExample Instruction: \"take the temperature at %exampleLocation\" \nExample Call: " + exampleCallString,
                 List.of(new ToolParameterHelper("locations", "array", locationNames, "The ordered list of saved locations to visit and measure. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)));
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
         definition = new ToolDefinition("measureTemperatureAtLocationsAndReportAt",
                 "You drive to each of the supplied locations in order, measure temperature at each one, then drive to reportLocation and summarize te findings. Use this when the user wants to take and report temperature at the samge time",
                 List.of(new ToolParameterHelper("locations", "array", locationNames, "The ordered list of saved locations to visit and measure. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true),
                         new ToolParameterHelper("reportLocation", "string", locationNames, "The saved location to return to before speaking the summary. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)));
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      }

      //Temporarily removed in focused context attempt
      //definition = new ToolDefinition("cancelAllQueuedTasks",
      //                                "You cancel all goals currently in the pending queue. This does not include separately scheduled goals. \nExample Instruction: \"cancel all upcoming tasks.\"",
      //                                new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("cancelAllCurrentAndQueuedTasks",
      //                                "You cancel all currently assigned goals. This includes the current active goal and all pending goals which have yet to begin execution. \nExample Instruction: \"cancel all tasks.\"",
      //                                new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //exampleCallString = xml ?
      //      "<function=cancelSingleTaskInQueue><parameter=index>0</parameter></function>" :
      //      "{\"name\":\"cancelSingleTaskInQueue\", \"arguments\":{\"index\":0}}";
      //definition = new ToolDefinition("cancelSingleTaskInQueue",
      //                                "You cancel and remove a task in the task queue specified by its index. \nExample Instruction: \"Remove the first task from the queue.\" \nExample Call: " + exampleCallString,
      //                                List.of(new ToolParameterHelper("index", "integer", new ArrayList(), "The 0-based index of the task in the queue to remove", true)));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("cancelAllScheduledGoals",
      //                                "You cancel all scheduled goals, meaning all tasks which have been scheduled to occur at a specific time in the future. \nExample Instruction: \"cancel all scheduled tasks.\"",
      //                                new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("cancelCurrentTask",
      //        "You stop and fully discard your current task, whether active or suspended. \nExample Instruction: \"cancel\" or \"stop pursuing your current task\"",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("suspendCurrentTask",
      //                                "You pause your current task, which can be later resumed or canceled. \nExample Instruction: \"stop\" or \"pause what you're doing.\"",
      //                                new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("resumeCurrentTask",
      //                                "You resume your current task if it is paused. \nExample Instruction: \"resume\" or \"continue your previous task.\"",
      //                                new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      exampleCallString = xml ?
            "<function=forgetTaughtAction><parameter=actionPredicate>{\"name\":\"goToLocation\",\"arguments\":{\"destination\":\"%exampleLocation\"}}</parameter></function>" :
            "{\"name\":\"forgetTaughtAction\", \"arguments\":{\"actionPredicate\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}}}";
      definition = new ToolDefinition("forgetTaughtAction",
                                      "You forget/delete a previously taught action. \nExample Instruction: \"forget how to go to %exampleLocation.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "Populated JSON representing the tool call to forget. This json minimally needs \"name\" and \"argument\" fields, like shown in the example.", true, true, "nlu")));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      exampleCallString = xml ?
            "<function=setKioskTimeout><parameter=duration>30</parameter><parameter=timeunit>MINUTES</parameter></function>" :
            "{\"name\":\"setKioskTimeout\", \"arguments\":{\"duration\":\"30\",\"timeunit\":\"MINUTES\"}}";
      definition = new ToolDefinition("setKioskTimeout",
                                      "You update the configured kiosk timeout. \nExample Instruction: \"Set the kiosk timeout to 30 minutes.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("duration", "integer", new ArrayList(), "The number of timeunits defining the timeout", true),
                                              new ToolParameterHelper("timeunit", "string", Arrays.stream(TimeUnit.values()).map(Objects::toString).toList(), "The timeunit of the duration to set ", true)
                                      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      exampleCallString = xml ?
            "<function=setKioskModeEnabled><parameter=enabled>true</parameter></function>" :
            "{\"name\":\"setKioskModeEnabled\", \"arguments\":{\"enabled\":\"true\"}}";
      definition = new ToolDefinition("setKioskModeEnabled",
                                      "You set whether or not kiosk mode is enabled. \nExample Instruction: \"turn on kiosk mode.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("enabled", "boolean", new ArrayList<>(), "the truth value of kiosk mode being enabled", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      exampleCallString = xml ?
            "<function=setVideoVolume><parameter=volume>100</parameter></function>" :
            "{\"name\":\"setVideoVolume\", \"arguments\":{\"volume\":\"100\"}}";
      definition = new ToolDefinition("setVideoVolume",
                                      "You set the volume of videos played on your screen. \nExample Instruction: \"Set the video volume to max.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("volume", "integer", new ArrayList(), "The volume to set the video media player to. Acceptable values are whole numbers between 0 and 100", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("lowerVideoVolume",
                                      "You lower the volume of videos played on your screen by one step. \nExample Instruction: \"turn it down.\"",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("raiseVideoVolume",
                                      "You raise the volume of videos played on your screen by one step. \nExample Instruction: \"turn it up.\"",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      exampleCallString = xml ?
            "<function=setVoiceVolume><parameter=volume>50</parameter></function>" :
            "{\"name\":\"setVoiceVolume\", \"arguments\":{\"volume\":\"50\"}}";
      definition = new ToolDefinition("setVoiceVolume",
                                      "You set the volume of your own speaking voice. \nExample Instruction: \"Set your voice volume to half.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("volume", "integer", new ArrayList(), "The voice volume as a percentage. Acceptable values are whole numbers between 0 and 100", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("lowerVoiceVolume",
                                      "You lower the volume of your own speaking voice by one step. \nExample Instruction: \"speak more quietly.\"",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("raiseVoiceVolume",
                                      "You raise the volume of your own speaking voice by one step. \nExample Instruction: \"speak louder.\"",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      exampleCallString = xml ?
            "<function=waitForDuration><parameter=seconds>60</parameter></function>":
            "{\"name\":\"waitForDuration\", \"arguments\":{\"seconds\":\"60\"}}";
      definition = new ToolDefinition("waitForDuration",
                                      "You wait idly for a number of seconds. \nExample Instruction: \"Then wait for a minute.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("seconds", "integer", new ArrayList(), "The amount of seconds to wait. Must be a positive whole number", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //TODO: Test that postprocessing of all forms of this action works correctly
      ////Experimenting with new format for video/image actions
      //List<String> videoInstructions = new ArrayList<>();
      //List<String> videoTranslations = new ArrayList<>();
      //videoInstructions.add("play video");
      //videoTranslations.add("playVideo(default,default)");
      //videoInstructions.add("Show the %exampleVideo video");
      //videoTranslations.add("playVideo(%exampleVideo:video,default)");
      //videoInstructions.add("Play videos at %exampleLocation");
      //videoTranslations.add("playVideo(default,%exampleLocation)");
      //videoInstructions.add("Go to %exampleLocation then play the %exampleVideo video");
      //videoTranslations.add("playVideo(%exampleVideo:video,%exampleLocation)");
      definition = new ToolDefinition("playNextVideo",
                                      "You begin playback of the next video from your saved collection on your built-in screen display at your current location. An example of instructing this would be \"play any video\".",
                                      new ArrayList<>());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("playNextVideoAtLocation",
                                      "You begin playback of the next video from your saved collection on your built-in screen display at the specified location. An example of instructing this would be \"play the next video at %exampleLocation\".",
                                      List.of(new ToolParameterHelper("location", "string", new ArrayList<>(), "The specific location at which to play the video. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)
                                      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("playSpecificVideo",
                                      "You begin playback of a specific video from your saved collection on your built-in screen display at your current location. An example of instructing this would be \"play the %exampleVideo\".",
                                      List.of(new ToolParameterHelper("videoName", "string", getVideoNames(), "The specific video to play. Accepted values are only those found in the Video List", true)
                                      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("playSpecificVideoAtLocation",
                                      "You begin playback of a specific video from your saved collection on your built-in screen display at the specified location. An example of instructing this would be \"play the %exampleVideo at the %exampleLocation\".",
                                      List.of(new ToolParameterHelper("videoName", "string", getVideoNames(), "The specific video to play. Accepted values are only those found in the Video List", true),
                                              new ToolParameterHelper("location", "string", getLocations("self",true), "The specific location at which to play the video. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)
                                      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //TODO: Test that postprocessing of all forms of this action works correctly
      //List<String> imageInstructions = new ArrayList<>();
      //List<String> imageTranslations = new ArrayList<>();
      //imageInstructions.add("Display the %exampleImage image");
      //imageTranslations.add("displayImage(%exampleImage:image,default,default)");
      //imageInstructions.add("Display the %exampleImage image at the %exampleLocation location");
      //imageTranslations.add("displayImage(%exampleImage:image,%exampleLocation,default)");
      //imageInstructions.add("Display the %exampleImage image for a minute");
      //imageTranslations.add("displayImage(%exampleImage:image,default,60)");
      //imageInstructions.add("Display the %exampleImage image at the %exampleLocation location for 45 seconds");
      //imageTranslations.add("displayImage(%exampleImage:image,%exampleLocation,45)");
      definition = new ToolDefinition("displayNamedImage",
                                      "You display an image from your saved collection on your built-in display at your current location indefinitely.",
                                      List.of(new ToolParameterHelper("imageName", "string", getImageNames(), "The image to display. Accepted values are only those found in the Image List", true)
                                      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("displayNamedImageAtLocation",
                                      "You display an image from your saved collection on your built-in display at a specified location indefinitely.",
                                      List.of(new ToolParameterHelper("imageName", "string", getImageNames(), "The image to display. Accepted values are only those found in the Image List", true),
                                              new ToolParameterHelper("location", "string", locationNames, "The location at which to display the image. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)
                                      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("displayNamedImageForDuration",
                                      "You display an image from your saved collection on your built-in display at your current location for a certain duration.",
                                      List.of(new ToolParameterHelper("imageName", "string", getImageNames(), "The image to display. Accepted values are only those found in the Image List", true),
                                              new ToolParameterHelper("duration", "integer", new ArrayList<>(), "How long to display the image for in seconds.", true)
                                      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("displayNamedImageForDurationAtLocation",
                                      "You display an image from your saved collection on your built-in display at a specified location for a certain duration.",
                                      List.of(new ToolParameterHelper("imageName", "string", getImageNames(), "The image to display. Accepted values are only those found in the Image List", true),
                                              new ToolParameterHelper("duration", "integer", new ArrayList<>(), "How long to display the image for in seconds.", true),
                                              new ToolParameterHelper("location", "string", locationNames, "The location at which to display the image. Accepted values are only those found in the Location List. Location values must keep the :location suffix", true)
                                      ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      exampleCallString = xml ?
            "<function=urgentPriority><parameter=actionPredicate>{\"name\":\"goToLocation\",\"arguments\":{\"destination\":\"%exampleLocation\"}}</parameter></function>" :
            "{\"name\":\"urgentPriority\", \"arguments\":{\"actionPredicate\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}}}";
      definition = new ToolDefinition("urgentPriority",
                                      "You submit an action with urgent priority. This means that this action will interrupt execution of any other currently submitted actions and be immediately prioritized. \nExample Instruction: \"go to %exampleLocation right now.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "Populated JSON of another tool call representing the action to submit with urgent priority. This json minimally needs \"name\" and \"argument\" fields, like shown in the example.", true, true, "nlu")));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      definition = new ToolDefinition("dontRespond",
                                      "The user has asked you to not verbally respond. \nExample Instruction: \"shut up.\"",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      definition = new ToolDefinition("scheduleTask",
                                      "You begin a dialogue interaction with the user to schedule future or regularly repeating execution of a task. \nExample Instruction: direct like \"I want to schedule a goal\", or indirect like saying they want you to perform a certain action in the future or on an interval",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("learnTaskSequence",
                                      "You begin a dialogue interaction with the user to learn how to perform a new \"task\", \"action\", or \"tour\" sequence. The learned task sequence will consist of navigating between a set of saved locations while performing actions at each location. \nExample Instruction: \"Let me tell you how to perform a new task\" or the user beginning to define an action sequence for you to save",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("modifyExistingTaskStatus",
              "You begin a dialogue interaction with the user to pause, cancel, or resume existing assigned tasks.",
              new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      ////TODO: need to update this with new social extended interaction structure - Do we still even need this as an action? Or should be rephrased to repeatSystemQuestion
      //definition = new ToolDefinition("repeatLastQuestion",
      //        "You repeat the last question you asked, which the human still needs to answer. examples of instructing this could be \"could you repeat that\" or \"what is the question\"",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));


      //Temporarily removed or hardware specific
      //definition = new ToolDefinition("saveLocationsToFile()",
      //        "You backup all your currently saved locations",
      //        "backup your current locations",
      //        "saveLocationsToFile()", 10);
      //toolDefinitions.add(definition);
      definition = new ToolDefinition("sit",
                                      "You lower your screen. \nExample Instruction: \"sit down.\"",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("stand",
                                      "You raise your screen. \nExample Instruction: \"stand up.\"",
                                      new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("deliver(?objectName, ?location:location)",
      //        "You deliver ?objectName to location ?location:location",
      //        "bring the mug to %exampleLocation",
      //        "deliver(mug,%exampleLocation)", 10);
      //toolDefinitions.add(definition);
      //    definition = new ToolDefinition("enableAdminMode",
      //            "Allow certain actions which require administrator privileges to be instructed through voice directly to the robot.. \nExample Instruction: \"enable admin mode.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition("disableAdminMode",
      //            "Reject certain actions which require administrator privileges from being instructed through voice directly to the robot.. \nExample Instruction: \"disable admin mode.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition("switchDisplayToTelepresence",
      //            "Change the output on your display to the telepresence stream. \nExample Instruction: \"start telepresence.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition("switchDisplayToTRACS",
      //            "Change the output on your display to what is shown by TRACS, the default setting. \nExample Instruction: \"return to TRACS.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition("cancelScheduledGoal",
      //            "You begin a dialogue interaction with the user to cancel a particular scheduled goal. \nExample Instruction: \"I want to cancel a scheduled task.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition("learnNewTask",
      //            "You begin a dialogue interaction with the user to learn the steps of a new instructable action. \nExample Instruction: \"i will teach you a new task.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition("endActionLearning",
      //            "You conclude learning the steps of a new instructable action. Examples of instructing this could be \"those are the steps of the task\" or \"im done\".",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition("updateKioskTimeout",
      //            "You begin a dialogue interaction with the user to update the configured kiosk timeout. \nExample Instruction: \"I would like to update the kiosk timeout.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition("updateKioskAction",
      //            "You begin a dialogue interaction with the user to update the configured kiosk action. \nExample Instruction: \"I want to change the kiosk action.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition("modifyScheduledGoal",
      //            "You begin a dialogue interaction with the user to modify the timing of a future already scheduled task. \nExample Instruction: \"I want to change the timing of an action.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));


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
      if (useCondensedTools) {
         return generateITKToolsCondensed();
      }

      List<Tool> itkTools = new ArrayList<>();
      ToolDefinition definition;

      definition = new ToolDefinition("getBatteryLevelString",
                                      "Use this to ask the system what the current battery level of the robot as a percentage is. An example of when you may need this information is when a user asks \"how much charge do you have.\"",
                                      new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition("getCurrentLocationString",
                                      "Use this to ask the system what saved location on your map you believe you are currently at. You may not be localized and not know where you are. An example of when you may need this information is when a user asks \"what is your current location.\"",
                                      new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition("getAllKnownLocations",
                                      "Use this to ask the system for a list of all currently saved locations. An example of when you may need this information is when a user asks \"what are your saved locations.\"",
                                      new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition("getVideoVolume",
                                      "Use this to ask the system for the current volume videos are set to play at. An example of when you may need this information is when a user asks \"what is the video volume set to.\"",
                                      new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getIsVideoSaved",
      //        "Use this to ask the system whether you have a video saved under the supplied name. An example of when you may need this information is when a user asks \"Do you have the %exampleVideo video saved.\" \nExample Call: reportIsVideoSaved(?videoName)",
      //        List.of(new ToolParameterHelper("videoName", "string", new ArrayList<>(), "The video being queried about", true)));
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION));
      definition = new ToolDefinition("getSavedVideos",
                                      "Use this to ask the system to list out the collection of videos you currently have saved. An example of when you may need this information is when a user asks \"what videos can you play.\"",
                                      new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition("getSavedImages",
                                      "Use this to ask the system to list out the collection of images you currently have saved. An example of when you may need this information is when a user asks \"what images can you display.\"",
                                      new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition("getPreviouslyMeasuredWifiAtLocationsString",
                                      "Use this to ask the system what wifi signal strength you previously measured at each of the supplied locations in the list is. Only returns a useful value for each value after wifi has been measured for that location. An example of when you may need this information is when a user asks \"how was the signal at %exampleLocation, example_location_2, and example_location_3?\"",
                                      List.of(new ToolParameterHelper("locations", "array", getLocations("self",true), "The ordered list of saved locations to report the wifi signal for. Accepted values are only those found in the Location List. Location values must keep the :location suffix.", true)));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      if (includeDemoTargets) {
         definition = new ToolDefinition("getPreviouslyMeasuredTemperatureAtLocationsString",
                 "Use this to ask the system what temperature (in degrees Fahrenheit) you previously measured at each of the supplied locations in the list. Only returns a useful value after measureTemperatureAtLocations has been run for those location. An example of when you may need this information is when a user asks \"how warm was it at %exampleLocation?, example_location_2, and example_location_3?\"",
                 List.of(new ToolParameterHelper("locations", "array", getLocations("self", true), "The ordered list of saved locations to report the temperature for. Accepted values are only those found in the Location List. Location values must keep the :location suffix.", true)));
         itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
         definition = new ToolDefinition("getPreviouslyMeasuredWifiAtCurrentLocationString",
                 "Use this to ask the system what the wifi signal strength is at your current location. An example of when you may need this information is when a user asks \"how is the signal where you are now\"",
                 new ArrayList<>());
         itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
         definition = new ToolDefinition("getPreviouslyMeasuredTemperatureAtCurrentLocationString",
                 "Use this to ask the system what the temperature is at your current location. An example of when you may need this information is when a user asks \"what is the temperature where you are now\"",
                 new ArrayList<>());
         itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      }

      // General ITK
      definition = new ToolDefinition("getInformationAroundTeachingNewActions",
                                      "Use this to get information from the system about how a user can start teaching you new tasks, the process of teaching a new task, and what actions can be taught as part of a new task. This information could be used to answer questions from the user. An example of when this may be useful to call is when a user asks things like \"how do I teach you a new task\", \"what information do I need to teach a new task\", \"how do I instruct something I've taught you\", or \"what can I tell you to do while teaching a new task\".",
                                      new ArrayList<>());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));

      //definition = new ToolDefinition("getInformationAroundHowToTeachATask",
      //        "Use this to get an explanation from the system on how a user can teach you new tasks, which you could use to answer questions from the user. If the supplied information is not relevant to the user's question, you do not need to address it. An example of when this may be useful to call is when a user asks \"how do I teach you a new task\".",
      //        new ArrayList<>());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getInformationAroundWhatALearnedTaskIs",
      //        "Use this to get an explanation from the system on what a taught task consists of, which you could use to answer questions from the user. If the supplied information is not relevant to the user's question, you do not need to address it. An example of when this may be useful to call is when a user asks \"what information do I need to teach a new task\" or \"how do I instruct something I've taught you\".",
      //        new ArrayList<>());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getInformationAroundWhatActionsCanBePerformedAlongALearnedTask",
      //        "Use this to get an explanation from the system on what actions you can be asked to perform as part of a taught task, which you could use to answer questions from the user. If the supplied information is not relevant to the user's question, you do not need to address it. An example of when this may be useful to call is when a user asks \"what can I tell you to do while teaching a new task\".",
      //        new ArrayList<>());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //String exampleCallString = xml ?
      //      "<function=getActionSteps><parameter=action>escort</parameter></function>" :
      //      "{\"name\":\"getActionSteps\", \"arguments\":{\"action\":\"escort\"}}";
      String exampleCallString = "<function=getActionSteps><parameter=action>escort</parameter></function>";
      definition = new ToolDefinition("getActionSteps",
                                      "Use this to ask the system for a description of an action, in the form of the list the steps which would be performed when it is executed. ?action can be replaced by populated JSON representing another tool OR be just the name of another tool if no specific arguments are supplied. An example of when you may need this information is when a user asks \"How would you escort someone.\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("action", "string", new ArrayList(), "The tool whose execution steps to describe", true)));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition("getKioskSettings",
                                      "Use this to ask the system for the current configuration of kiosk mode; Whether it is enabled, the currently configured kiosk timeout, and the kiosk action. An example of when you may need this information is when a user asks \"what is your kiosk configuration.\"",
                                      new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition("getAssistanceOnUsingSaveLocationAndGoToLocationProperly",
                                      "Use this to get information from the system around the concepts of saved locations, navigating between them, and how to save new ones. The provided information may be helpful to answer questions from the user. If the supplied information is not relevant to the user's question, you do not need to address it. An example of when this may be useful to call is when a user asks \"Can I teach you how to go to X\".",
                                      new ArrayList<>());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition("checkPastInteractionHistoryForMissingInformation",
                                      "Use this when you believe you are missing important context necessary to understand user input. For example, when the user explicitly references a past conversation or says something implying common ground you have no memory of.",
                                      new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));

      //Prevent the LLM from just regurgitating the history directly
      //    definition = new ToolDefinition("checkPastInteractionHistoryForMissingInformation",
      //            "Use this to ask the system for the extended conversation history between you and the user. In other words, call this to get information about past interactions with the user in case it seems they are referring back to something they discussed with you which you do not have memory of.",
      //            new ArrayList());
      //    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));


      //// Removed ITK targets
      // Made irrelevant by instruct list present in basically all contexts' system prompts. Can reintroduce if we find we
      //  want a more formal representation or rigid relationship between itk tool responses and LLM text output. Seems to
      //  handle information directly from the prompt fairly well though
      //definition = new ToolDefinition("getIfCanPerform",
      //        "Use this to ask the system whether you know how to perform the action represented by the supplied actionPredicate. actionPredicate should be assigned a populated instance of another tool call. An example of when you may need this information is when a user asks \"do you know how to escort a patient from %exampleLocation to the dock.\" \nExample Call: getIfCanPerform(escortFromLocationToLocation(guest,%exampleLocation,dock:location))",
      //        List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "The populated tool call representing the action queried about", true)));
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION));

      //// Present in prompt
      //definition = new ToolDefinition("reportInstructableActions",
      //        "You report the list of actions a user is able to instruct you through natural language to perform. An example of when you may need this information is when a user asks \"what can you be instructed to do.\"",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION));
      //TODO: make sure this distinction is possible to tease out from the LLM just from information in the prompt
      definition = new ToolDefinition("getTaughtActionsList",
                                      "Use this to ask the system for a list of all of the actions which have been taught to you by a user. An example of when you may need this information is when a user asks \"What actions have you been taught.\"",
                                      new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getPredefinedActions",
      //        "Use this to ask the system for the list of actions a user is able to instruct you through natural language to perform. An example of when you may need this information is when a user asks \"what can you be instructed to do.\"",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION));
      return itkTools;
   }

   /**
    * Generate all possible REPLY UtteranceType tools available for this domain. Will be included in any LLM Completion
    * request dynamically based on matching expected response forms for current system questions.
    */
   public Map<String,Tool> generateReplyTools() {
      if (useCondensedTools) {
         return generateReplyToolsCondensed();
      }

      Map<String,Tool> replyTools = new HashMap();
      ToolDefinition definition;
      List<String> locationNames = new ArrayList<>(getLocations("self", true));
      locationNames.add("none");

      definition = new ToolDefinition("abort",
                                      "You inform the system that the user is explicitly abandoning answering its current OUTSTANDING QUESTION. Examples of instructing this could be saying \"nevermind\" immediately after a question has been asked.",
                                      List.of(new ToolParameterHelper("target","string",List.of("question"),"",true)));
      replyTools.put("abort",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("abort()"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("val",
                                      "You inform the system that the user has answered its current OUTSTANDING QUESTION with a yes/no answer.",
                                      List.of(new ToolParameterHelper("response","string",List.of("yes","no"),"Whether the user answered affirmatively or negatively",true)));
      replyTools.put("val",new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("validated",
                                      "You inform the system that the user has answered its current outstanding validation question.",
                                      List.of(new ToolParameterHelper("response","string",List.of("yes","no"),"The response value should be yes when the user has validated your interpretation of the their instruction, the response value should be no when the user denies your interpretation of their instruction or adds clarifying or additional details to your interpretation",true)));
      replyTools.put("validated",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("val(X)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("location",
                                      "You inform the system that the user has answered its current OUTSTANDING QUESTION by specifying a certain saved location",
                                      List.of(new ToolParameterHelper("name","string",locationNames,"The specified location. \"none\" is the one special value, indicating the user is done supplying location names.",true)));
      replyTools.put("location",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("location(X:location)"), new Tool(definition,UtteranceType.REPLY));
      //String exampleCallString = xml ?
      //      "<function=taskPredicate><parameter=toolCall><function=goToLocation><parameter=destination>%exampleLocation</parameter></function></parameter></function>" :
      //      "{\"name\":\"taskPredicate\", \"arguments\":{\"toolCall\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}}}";
      String exampleCallString = "<function=taskPredicate><parameter=toolCall><function=goToLocation><parameter=destination>%exampleLocation</parameter></function></parameter></function>";
      definition = new ToolDefinition("taskPredicate",
                                      "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of another tool call. \nExample Instruction: \"the task is to go to %exampleLocation\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("toolCall","string",new ArrayList<>(),"This argument should be replaced by definition of another tool call and populated arguments from the previous examples",true, true, "nlu")));
      replyTools.put("taskPredicate",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("taskPredicate(X)"), new Tool(definition,UtteranceType.REPLY));
      //exampleCallString = xml ?
      //      "<function=relativeTime><parameter=duration>2</parameter><parameter=timeunit>HOURS</parameter></function>" :
      //      "{\"name\":\"relativeTime\", \"arguments\":{\"duration\":\"2\",\"timeunit\":\"HOURS\"}}";
      exampleCallString = "<function=relativeTime><parameter=duration>2</parameter><parameter=timeunit>HOURS</parameter></function>";
      definition = new ToolDefinition("relativeTime",
                                      "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of a time relative to now. \nExample Instruction: \"in two hours\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("duration","number",List.of(),"an integer, the duration of the specified time",true),
                                              new ToolParameterHelper("timeUnit","string", Arrays.stream(TimeUnit.values()).map(TimeUnit::toString).toList(),"the duration's corresponding timeunit",true)
                                      ));
      replyTools.put("relativeTime",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("relativeTime(X,Y)"), new Tool(definition,UtteranceType.REPLY));
      //exampleCallString = xml ?
      //      "<function=absoluteTime><parameter=timeOfDay>12:30</parameter><parameter=partOfDay>pm</parameter></function>" :
      //      "{\"name\":\"absoluteTime\", \"arguments\":{\"timeOfDay\":\"12:30\",\"partOfDay\":\"pm\"}}";
      exampleCallString = "<function=absoluteTime><parameter=timeOfDay>12:30</parameter><parameter=partOfDay>pm</parameter></function>";
      definition = new ToolDefinition("absoluteTime",
                                      "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of an absolute time. \nExample Instruction: \"half past noon\" \nExample Call: " + exampleCallString,
                                      List.of(new ToolParameterHelper("timeOfDay","string",List.of(),"The time of day in the format hour:minute",true),
                                              new ToolParameterHelper("partOfDay","string", List.of("am","pm"),"",true)
                                      ));
      replyTools.put("absoluteTime",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("absoluteTime(X,Y)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("timingOf",
                                      "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of one of the aspects of execution timing of a repeated task. Example instructions and corresponding argument values could be \"when it first happens\" -> timingOf(execution) or \"the duration between when it happens\" -> timingOf(interval)",
                                      List.of(new ToolParameterHelper("timingType","string",List.of("execution","interval"),"execution references when something occurs and interval references the amount of time between something occurring ",true)));
      replyTools.put("timingOf",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("timingOf(X)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("taskName",
                                      "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of the name of a task to be learned",
                                      List.of(new ToolParameterHelper("name","string",List.of(),"",true)));
      replyTools.put("taskName",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("taskName(X)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("option",
                                      "You inform the system that the user has answered its current OUTSTANDING QUESTION through selection of a presented option.",
                                      List.of(new ToolParameterHelper("number","number",List.of(),"the number of the chosen option from the previously presented list",true)));
      replyTools.put("option",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("option(X)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("actionType",
                                      "You inform the system that the user has answered its current OUTSTANDING QUESTION through selection of a type of action. Example instructions and corresponding argument values could be \"the taught actions\" -> actionType(taught) or \"the ones you already knew\" -> actionType(predefined) or \"all of them\" -> actionType(both)",
                                      List.of(new ToolParameterHelper("type","string",List.of("taught","predefined","both"),"How the system came to know about the discussed action. Taught references actions tuaght by the user, predefined references actions known automatically by the system on startup, and both references all actions.",true)));
      replyTools.put("actionType",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("actionType(X)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("amount",
                                      "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of an amount.",
                                      List.of(new ToolParameterHelper("number","number",List.of(),"The only acceptable value is either an integer or \"all\". Where \"all\" is referring the maximum amount of something",true)));
      replyTools.put("amount",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("amount(X)"), new Tool(definition,UtteranceType.REPLY));

      return replyTools;
   }

   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /// "Condensed" versions of prompts
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   //TODO: add example instruction fields to ToolDefinitions and conditionally add to output placed in prompt if desired
   //TODO: somehow clearly label this section of tools as 'Action tools' with an associated description in the overall prompt (hard to do because of current model side templating)
   public List<Tool> generateInstructToolsCondensed(boolean xml) {
      List<Tool> instructTools = new ArrayList<>();
      ToolDefinition definition;
      List<String> locationNames = getLocations("self", true);

      ////TODO: standardize concept of 'tools only advertised under specific contexts' not specifically related to focused contexts.
      ////      (Short term impl of this in SocialTRLLMServiceProvider, getContextDependentToolDefinitions)
      //definition = new ToolDefinition("stopFollowing",
      //        "You stop following the user",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));


      definition = new ToolDefinition("goToLocation",
              "You go to the supplied destination :location",
              List.of(new ToolParameterHelper("destination", "string", locationNames, "The destination :location to go to. ", true))
      );
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("goHome",
              "You go to your default charging dock and charge",
              new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("follow",
              "You follow the user",
              new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("escort",
              "You lead someone between two :locations",
              List.of(new ToolParameterHelper("name", "string", new ArrayList(), "Name of the entity to escort", true),
                      new ToolParameterHelper("start", "string", locationNames, "The start :location to escort from. Don't supply if already with the user", false),
                      new ToolParameterHelper("destination", "string", locationNames, "The destination :location to escort to", true)
              ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //TODO: Methods you cannot schedule or teach (this, maybe follow/stopFollowing, dontRespond, forgetTaughtAction, focused contexts, ...)
      definition = new ToolDefinition("saveLocation",
              "You save your current position under the supplied name. Use the 'getAssistanceOnUsingSaveLocationAndGoToLocationProperly' tool to understand how to use this function properly.",
              List.of(new ToolParameterHelper("name", "string", new ArrayList(), "The label to save the location as", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));


      definition = new ToolDefinition("measureWifiSignal",
              "You measure wifi signal strength at certain :locations",
              List.of(new ToolParameterHelper("locations", "array", locationNames, "The ordered list of saved :locations to visit and measure wifi at. Empty if only measuring at your current position", false),
                      new ToolParameterHelper("reportLocation", "string", locationNames, "The saved :location to go to and give a summary of the results. Don't supply if not reporting a summary of the results", false)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      if (includeDemoTargets) {
         definition = new ToolDefinition("measureTemperature",
                 "You measure the temperature at certain :locations",
                 List.of(new ToolParameterHelper("locations", "array", locationNames, "The ordered list of saved :locations to visit and measure temperature at. Empty if only measuring at your current position", false),
                         new ToolParameterHelper("reportLocation", "string", locationNames, "The saved :location to return and give a summary of the results. Don't supply if not reporting a summary of the results", false)));
         instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      }

      definition = new ToolDefinition("sit",
              "You lower your screen",
              new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("stand",
              "You raise your screen",
              new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      definition = new ToolDefinition("displayImage",
              "You display an :image from your saved collection on your built-in screen display at a :location.",
              List.of(new ToolParameterHelper("imageName", "string", getImageNames(), "The image to display", true),
                      new ToolParameterHelper("duration", "integer", new ArrayList<>(), "How long to display the image for in seconds. Don't supply if displaying indefinitely until told to stop.", false),
                      new ToolParameterHelper("location", "string", locationNames, "The specific :location at which to display the image. Don't supply if displaying at your current location", false)
              ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("playVideo",
              "You begin playback of a :video from your saved collection on your built-in screen display at a :location.",
              List.of(new ToolParameterHelper("videoName", "string", getVideoNames(), "The specific video to play. Don't supply if playing a random video", false),
                      new ToolParameterHelper("location", "string", locationNames, "The specific :location at which to play the video. Don't supply if playing at your current location", false)
              ));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("setVideoVolume",
              "You set the volume of videos played on your screen",
              List.of(new ToolParameterHelper("volume", "integer", new ArrayList(), "The volume to set the video media player to. Acceptable values are whole numbers between 0 and 100", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      definition = new ToolDefinition("setVoiceVolume",
              "You set the volume of your own speaking voice",
              List.of(new ToolParameterHelper("volume", "integer", new ArrayList(), "The voice volume as a percentage. Acceptable values are whole numbers between 0 and 100", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      ////TODO: should be general targets moved down to base class?
      definition = new ToolDefinition("waitForDuration",
              "You wait idly for a number of seconds",
              List.of(new ToolParameterHelper("seconds", "integer", new ArrayList(), "The amount of seconds to wait. Must be a positive whole number", true)));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      definition = new ToolDefinition("dontRespond",
              "The user has asked you to not verbally respond",
              new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //TODO: forgetTaughtAction(s) focused context with action specification similar to that for goal manipulation
      //TODO: restrict list of acceptable tools to taught ones, not all nlu targets
      String exampleCallString = xml ?
              "<function=forgetTaughtAction><parameter=actionPredicate>{\"name\":\"goToLocation\",\"arguments\":{\"destination\":\"%exampleLocation\"}}</parameter></function>" :
              "{\"name\":\"forgetTaughtAction\", \"arguments\":{\"actionPredicate\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}}}";
      definition = new ToolDefinition("forgetTaughtAction",
              "You forget/delete a previously taught action",
              List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "Populated JSON representing the tool call to forget. This json minimally needs \"name\" and \"argument\" fields, for example: " + exampleCallString, true, true, "nlu")));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("forgetAllTaughtActions",
              "You forget all previously taught actions",
              new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //TODO: goalPriority(re)assignment focused context
      exampleCallString = xml ?
              "<function=urgentPriority><parameter=actionPredicate>{\"name\":\"goToLocation\",\"arguments\":{\"destination\":\"%exampleLocation\"}}</parameter></function>" :
              "{\"name\":\"urgentPriority\", \"arguments\":{\"actionPredicate\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}}}";
      definition = new ToolDefinition("urgentPriority",
              "You submit an action with urgent priority. This means that this action will interrupt execution of any other currently submitted actions and be immediately prioritized. \nExample Instruction: \"go to %exampleLocation right now.\" \nExample Call: " + exampleCallString,
              List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "Populated JSON of another tool call representing the action to submit with urgent priority. This json minimally needs \"name\" and \"argument\" fields, like shown in the example.", true, true, "nlu")));
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //TODO long term: make focused "configure kiosk mode" context
      //TODO short term: including any version of kiosk mode through language? <- think this is either something that needs to be externally explained or integrated deeply enough that it is a focused context.
      //                 Random targets for unexplained functionality cannot work well.
      //exampleCallString = xml ?
      //        "<function=setKioskModeEnabled><parameter=enabled>true</parameter></function>" :
      //        "{\"name\":\"setKioskModeEnabled\", \"arguments\":{\"enabled\":\"true\"}}";
      //definition = new ToolDefinition("setKioskModeEnabled",
      //        "You set whether or not kiosk mode is enabled. \nExample Instruction: \"turn on kiosk mode.\" \nExample Call: " + exampleCallString,
      //        List.of(new ToolParameterHelper("enabled", "boolean", new ArrayList<>(), "the truth value of kiosk mode being enabled", true)));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("setKioskTimeout",
      //        "You update the configured kiosk mode timeout",
      //        List.of(new ToolParameterHelper("duration", "integer", new ArrayList(), "The number of timeunits defining the timeout", true),
      //                new ToolParameterHelper("timeunit", "string", Arrays.stream(TimeUnit.values()).map(Objects::toString).toList(), "The timeunit of the duration to set ", true)
      //        ));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("setKioskGoal",
      //        "You update the configured kiosk mode goal",
      //        List.of(new ToolParameterHelper("duration", "integer", new ArrayList(), "The number of timeunits defining the timeout", true),
      //                new ToolParameterHelper("timeunit", "string", Arrays.stream(TimeUnit.values()).map(Objects::toString).toList(), "The timeunit of the duration to set ", true)
      //        ));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //// Focused contexts
      definition = new ToolDefinition("scheduleTask",
              "You begin a more focused dialogue interaction with the user to schedule future or regularly repeating execution of a task. \nExample Instruction: direct like \"I want to schedule a goal\", or indirect like saying they want you to perform a certain action in the future or on an interval",
              new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("learnTaskSequence",
              "You begin a more focused dialogue interaction with the user to learn how to perform a new \"task\", \"action\", or \"tour\" sequence. The learned task sequence will consist of navigating between a set of saved locations while performing actions at each location. \nExample Instruction: \"Let me tell you how to perform a new task\" or the user beginning to define an action sequence for you to save",
              new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      definition = new ToolDefinition("modifyExistingTaskStatus",
              "You begin a dialogue interaction with the user to pause, cancel, or resume existing assigned tasks. Examples of when this is relevant could be when a user tells you to forget about, stop, or pick back up with what you are doing.",
              new ArrayList());
      instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

      //// Functionality temporarily removed whether due to lack of robust implementation or unsure if wanting to expose through flexible language
      //definition = new ToolDefinition("deliver(?objectName, ?location:location)",
      //        "You deliver ?objectName to location ?location:location",
      //        "bring the mug to %exampleLocation",
      //        "deliver(mug,%exampleLocation)", 10);
      //toolDefinitions.add(definition);
      //    definition = new ToolDefinition("enableAdminMode",
      //            "Allow certain actions which require administrator privileges to be instructed through voice directly to the robot.. \nExample Instruction: \"enable admin mode.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    definition = new ToolDefinition("disableAdminMode",
      //            "Reject certain actions which require administrator privileges from being instructed through voice directly to the robot.. \nExample Instruction: \"disable admin mode.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //    TODO: this target should be something saying to refer to the website
      //    definition = new ToolDefinition("switchDisplayToTelepresence",
      //            "Change the output on your display to the telepresence stream. \nExample Instruction: \"start telepresence.\"",
      //            new ArrayList());
      //    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));


      ////Removed in favor of goal manipulation focused context attempt
      //definition = new ToolDefinition("cancelCurrentTask",
      //        "You cancel and fully discard your current goal, whether it is active or suspended.",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("suspendCurrentTask",
      //        "You pause your current goal, which can be later resumed or canceled",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("resumeCurrentTask",
      //        "You resume your current goal if it is suspended",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("cancelSingleTaskInQueue",
      //        "You cancel and remove a specific goal in the queue specified by its index",
      //        List.of(new ToolParameterHelper("index", "integer", new ArrayList(), "The 0-based index of the task in the queue to remove", true)));
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("cancelAllQueuedTasks",
      //        "You cancel all goals currently in the pending queue.",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("cancelAllCurrentAndQueuedTasks",
      //        "You cancel both your current goal and all goals currently in the pending queue",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("cancelSingleScheduledGoal",
      //        "You cancel a particular scheduled goal, meaning one tasks which has been scheduled to occur at a specific time in the future",
      //        new ArrayList());
      //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
      //definition = new ToolDefinition("cancelAllScheduledGoals",
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
   //TODO: need to force the LLM to call these tools to get an updated notion of dynamic information when relevant (rather than use old results)
   //TODO: somehow clearly label this section of tools as ITK tools
   public List<Tool> generateITKToolsCondensed() {
      List<Tool> itkTools = new ArrayList<>();
      ToolDefinition definition;

      //// Domain specific ITK
      definition = new ToolDefinition("getPreviouslyMeasuredWifiAtLocationsString",
              "Get previously measured wifi signal strengths of a set of supplied :locations",
              List.of(new ToolParameterHelper("locations", "array", getLocations("self",true), "The list of saved :locations to report wifi signals for. Empty if the current location only", false)));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      if (includeDemoTargets) {
         definition = new ToolDefinition("getPreviouslyMeasuredTemperatureAtLocationsString",
                 "Get previously measured temperature readings of a set of supplied :locations",
                 List.of(new ToolParameterHelper("locations", "array", getLocations("self", true), "The list of saved :locations to report the temperature for. Empty if the current location only", false)));
         itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      }

      // TODO: revisit which of these should actually be kept and work well.
      //// General "meta-level" ITK
      definition = new ToolDefinition("getInformationAroundTeachingNewActions",
              "Use this to get information from the system about how a user can start teaching you new tasks, the process of teaching a new task, and what actions can be taught as part of a new task. This information could be used to answer questions from the user. An example of when this may be useful to call is when a user asks things like \"how do I teach you a new task\", \"what information do I need to teach a new task\", \"how do I instruct something I've taught you\", or \"what can I tell you to do while teaching a new task\".",
              new ArrayList<>());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getInformationAroundHowToTeachATask",
      //        "Use this to get an explanation from the system on how a user can teach you new tasks, which you could use to answer questions from the user. If the supplied information is not relevant to the user's question, you do not need to address it. An example of when this may be useful to call is when a user asks \"how do I teach you a new task\".",
      //        new ArrayList<>());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getInformationAroundWhatALearnedTaskIs",
      //        "Use this to get an explanation from the system on what a taught task consists of, which you could use to answer questions from the user. If the supplied information is not relevant to the user's question, you do not need to address it. An example of when this may be useful to call is when a user asks \"what information do I need to teach a new task\" or \"how do I instruct something I've taught you\".",
      //        new ArrayList<>());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getInformationAroundWhatActionsCanBePerformedAlongALearnedTask",
      //        "Use this to get an explanation from the system on what actions you can be asked to perform as part of a taught task, which you could use to answer questions from the user. If the supplied information is not relevant to the user's question, you do not need to address it. An example of when this may be useful to call is when a user asks \"what can I tell you to do while teaching a new task\".",
      //        new ArrayList<>());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition("getAssistanceOnUsingSaveLocationAndGoToLocationProperly",
              "Use this to get information from the system about navigation, saved locations, the map, and how to learn new locations.",
              new ArrayList<>());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));

      //TODO: move to base class
      //// Domain independent ITK
      definition = new ToolDefinition("getTaughtActionsList",
              "Use this to ask the system for a list of all of the actions which have been taught to you by a user.",
              new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      String exampleCallString = "<function=getActionSteps><parameter=action>escort</parameter></function>";
      definition = new ToolDefinition("getActionSteps",
              "Use this to ask the system for a description of an action, in the form of the list the steps which would be performed when it is executed. ?action can be replaced by populated JSON representing another tool OR be just the name of another tool if no specific arguments are supplied. An example of when you may need this information is when a user asks \"How would you escort someone.\" \nExample Call: " + exampleCallString,
              List.of(new ToolParameterHelper("action", "string", new ArrayList(), "The tool whose execution steps to describe", true)));
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      definition = new ToolDefinition("checkPastInteractionHistoryForMissingInformation",
              "Use this when you believe you are missing important context necessary to understand user input. For example, when the user explicitly references a past conversation or says something implying common ground you have no memory of.",
              new ArrayList());
      itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));


      //TODO: keep aligned with above whether including kiosk in language or not.
      //      Revisit when focused context implemented - include in system prompt vs only as focused vs as separate itk tool here
      //definition = new ToolDefinition("getKioskSettings",
              //        "Use this to ask the system for the current configuration of kiosk mode; Whether it is enabled, the currently configured kiosk timeout, and the kiosk action. An example of when you may need this information is when a user asks \"what is your kiosk configuration.\"",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));

      //// Removed
      //TODO: make sure this distinction is possible to tease out from the LLM just from information in the prompt
      //definition = new ToolDefinition("getPredefinedActions",
      //        "Use this to ask the system for the list of actions a user is able to instruct you through natural language to perform. An example of when you may need this information is when a user asks \"what can you be instructed to do.\"",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION));

      //// Info moved to be included in system message automatically
      //definition = new ToolDefinition("getBatteryLevelString",
      //        "Get the current battery level percent of the robot",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getCurrentLocationString",
      //        "Get the current location you believe you are at",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getAllKnownLocations",
      //        "Get a list of all of your currently saved locations",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getVideoVolume",
      //        "Get the current volume videos are set to play at",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getSavedVideos",
      //        "Get the list of saved videos in your collection which you are able to play",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      //definition = new ToolDefinition("getSavedImages",
      //        "Get the list of saved images in your collection which you are able to display",
      //        new ArrayList());
      //itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
      return itkTools;
   }

   /**
    * Generate all possible REPLY UtteranceType tools available for this domain. Will be included in any LLM Completion
    * request dynamically based on matching expected response forms for current system questions.
    */
   public Map<String,Tool> generateReplyToolsCondensed() {
      Map<String,Tool> replyTools = new HashMap();
      ToolDefinition definition;
      List<String> locationNames = new ArrayList<>(getLocations("self", true));
      locationNames.add("none");

      //TODO: Move some general targets to base class
      definition = new ToolDefinition("abort",
              "You inform the system that the user is explicitly abandoning answering its current OUTSTANDING QUESTION",
              List.of(new ToolParameterHelper("target","string",List.of("question"),"",true)));
      replyTools.put("abort",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("abort()"), new Tool(definition,UtteranceType.REPLY));
      //TODO: should val still be included?
      definition = new ToolDefinition("val",
              "You inform the system that the user has answered its current OUTSTANDING QUESTION with a yes/no answer.",
              List.of(new ToolParameterHelper("response","string",List.of("yes","no"),"Whether the user answered affirmatively or negatively",true)));
      replyTools.put("val",new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("validated",
              "You inform the system that the user has answered its current outstanding validation question.",
              List.of(new ToolParameterHelper("response","string",List.of("yes","no"),"The response value should be yes when the user has validated your interpretation of the their instruction, the response value should be no when the user denies your interpretation of their instruction or adds clarifying or additional details to your interpretation",true)));
      replyTools.put("validated",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("val(X)"), new Tool(definition,UtteranceType.REPLY));

      //TODO: are any of these ever necessary? (doesn't matter too much because if no Q is asked with them as a possible response form, then they will never be inserted. But would be nice for cleanup)
      definition = new ToolDefinition("location",
              "You inform the system that the user has answered its current OUTSTANDING QUESTION by specifying a certain saved location",
              List.of(new ToolParameterHelper("name","string",locationNames,"The specified location. \"none\" is the one special value, indicating the user is done supplying location names.",true)));
      replyTools.put("location",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("location(X:location)"), new Tool(definition,UtteranceType.REPLY));
      //String exampleCallString = xml ?
      //      "<function=taskPredicate><parameter=toolCall><function=goToLocation><parameter=destination>%exampleLocation</parameter></function></parameter></function>" :
      //      "{\"name\":\"taskPredicate\", \"arguments\":{\"toolCall\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}}}";
      String exampleCallString = "<function=taskPredicate><parameter=toolCall><function=goToLocation><parameter=destination>%exampleLocation</parameter></function></parameter></function>";
      definition = new ToolDefinition("taskPredicate",
              "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of another tool call. \nExample Instruction: \"the task is to go to %exampleLocation\" \nExample Call: " + exampleCallString,
              List.of(new ToolParameterHelper("toolCall","string",new ArrayList<>(),"This argument should be replaced by definition of another tool call and populated arguments from the previous examples",true, true, "nlu")));
      replyTools.put("taskPredicate",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("taskPredicate(X)"), new Tool(definition,UtteranceType.REPLY));
      //exampleCallString = xml ?
      //      "<function=relativeTime><parameter=duration>2</parameter><parameter=timeunit>HOURS</parameter></function>" :
      //      "{\"name\":\"relativeTime\", \"arguments\":{\"duration\":\"2\",\"timeunit\":\"HOURS\"}}";
      exampleCallString = "<function=relativeTime><parameter=duration>2</parameter><parameter=timeunit>HOURS</parameter></function>";
      definition = new ToolDefinition("relativeTime",
              "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of a time relative to now. \nExample Instruction: \"in two hours\" \nExample Call: " + exampleCallString,
              List.of(new ToolParameterHelper("duration","number",List.of(),"an integer, the duration of the specified time",true),
                      new ToolParameterHelper("timeUnit","string", Arrays.stream(TimeUnit.values()).map(TimeUnit::toString).toList(),"the duration's corresponding timeunit",true)
              ));
      replyTools.put("relativeTime",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("relativeTime(X,Y)"), new Tool(definition,UtteranceType.REPLY));
      //exampleCallString = xml ?
      //      "<function=absoluteTime><parameter=timeOfDay>12:30</parameter><parameter=partOfDay>pm</parameter></function>" :
      //      "{\"name\":\"absoluteTime\", \"arguments\":{\"timeOfDay\":\"12:30\",\"partOfDay\":\"pm\"}}";
      exampleCallString = "<function=absoluteTime><parameter=timeOfDay>12:30</parameter><parameter=partOfDay>pm</parameter></function>";
      definition = new ToolDefinition("absoluteTime",
              "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of an absolute time. \nExample Instruction: \"half past noon\" \nExample Call: " + exampleCallString,
              List.of(new ToolParameterHelper("timeOfDay","string",List.of(),"The time of day in the format hour:minute",true),
                      new ToolParameterHelper("partOfDay","string", List.of("am","pm"),"",true)
              ));
      replyTools.put("absoluteTime",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("absoluteTime(X,Y)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("timingOf",
              "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of one of the aspects of execution timing of a repeated task. Example instructions and corresponding argument values could be \"when it first happens\" -> timingOf(execution) or \"the duration between when it happens\" -> timingOf(interval)",
              List.of(new ToolParameterHelper("timingType","string",List.of("execution","interval"),"execution references when something occurs and interval references the amount of time between something occurring ",true)));
      replyTools.put("timingOf",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("timingOf(X)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("taskName",
              "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of the name of a task to be learned",
              List.of(new ToolParameterHelper("name","string",List.of(),"",true)));
      replyTools.put("taskName",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("taskName(X)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("option",
              "You inform the system that the user has answered its current OUTSTANDING QUESTION through selection of a presented option.",
              List.of(new ToolParameterHelper("number","number",List.of(),"the number of the chosen option from the previously presented list",true)));
      replyTools.put("option",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("option(X)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("actionType",
              "You inform the system that the user has answered its current OUTSTANDING QUESTION through selection of a type of action. Example instructions and corresponding argument values could be \"the taught actions\" -> actionType(taught) or \"the ones you already knew\" -> actionType(predefined) or \"all of them\" -> actionType(both)",
              List.of(new ToolParameterHelper("type","string",List.of("taught","predefined","both"),"How the system came to know about the discussed action. Taught references actions tuaght by the user, predefined references actions known automatically by the system on startup, and both references all actions.",true)));
      replyTools.put("actionType",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("actionType(X)"), new Tool(definition,UtteranceType.REPLY));
      definition = new ToolDefinition("amount",
              "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of an amount.",
              List.of(new ToolParameterHelper("number","number",List.of(),"The only acceptable value is either an integer or \"all\". Where \"all\" is referring the maximum amount of something",true)));
      replyTools.put("amount",new Tool(definition,UtteranceType.REPLY));
      //replyTools.put(Factory.createPredicate("amount(X)"), new Tool(definition,UtteranceType.REPLY));

      return replyTools;
   }
}