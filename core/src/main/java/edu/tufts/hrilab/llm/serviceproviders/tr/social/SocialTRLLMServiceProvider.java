package edu.tufts.hrilab.llm.serviceproviders.tr.social;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.common.reflect.TypeToken;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonSyntaxException;
import com.google.gson.internal.LinkedTreeMap;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.LLMServiceProvider;
import edu.tufts.hrilab.llm.Message;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.llm.gson.tool.ToolDefinition;
import edu.tufts.hrilab.llm.gson.tool.ToolParameter;
import edu.tufts.hrilab.llm.gson.tool.ToolParameters;
import edu.tufts.hrilab.llm.gson.tool.response.ToolCall;
import edu.tufts.hrilab.llm.gson.tool.response.ToolFunction;
import edu.tufts.hrilab.llm.openai.ToolParameterHelper;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.dialogue.Question;
import org.apache.commons.lang3.tuple.Pair;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

//TODO: This is currently technically the "SocialAvaLLMServiceProvider" subclass of what should be an abstract
//      "SocialTRLLMServiceProvider" class. Extract domain specific logic from general extended dialogue service
//      provider logic when having time and/or more examples.
// (Otherwise, expansive higher level TODO list currently in personal notes)
public class SocialTRLLMServiceProvider extends LLMServiceProvider {
  /**
   * Maintained set of ChatCollections, split by user/AgentTeam pair
   * Contains all relevant conversation contexts and their managed histories between each pair
   */
  // TODO: extend to groups of interactors, similar to conversations
  protected final Map<Pair<Symbol,Symbol>,ChatCollection> currentChatCollections = new HashMap<>();

  /**
   * Extended History of chat contexts, stored for reference from ITK actions for the LLM to conditionally
   * retrieve information about past interactions without cluttering current chat context.
   */
  protected final Map<Pair<Symbol,Symbol>,List<ChatContext>> chatContextHistories = new HashMap<>();

  /**
   *  Prompt variables to be replaced in sent Completion requests. Refer to the Prompt class and apply method if this
   *   notion is unfamiliar to you.
   */
  //TODO: Refactor to align closer to how this method exists in the TRLLMServiceProvider
  //TODO: These are currently (re)populated incredibly inefficiently. Should largely maintain state and update on
  //      relevant event listeners.
  //TODO: At least elements of this should exist per user/AgentTeam pair - include parallel notion in ChatCollections for specific information
  protected Map<String,String> promptVars = new LinkedHashMap<>();
  //// Revisit when making base/subclass distinction
  //protected abstract Map<String,String> initPromptVars();
  //protected abstract Map<String,String> updatePromptVars(Utterance utterance);

  /**
   * List of types handled by consultants in the system - when encountering symbols with these types during
   *  NLG preprocessing/NLU postprocessing, replaces references with descriptors/typed descriptors with variables.
   */
  //TODO: pull from consultants automatically
  List<String> refTypes = List.of("location"); // subclass specific
  //protected abstract List<String> getRefTypes();

  /**
   * Collection of system prompts and Tool sets utilized in this domain. Ths set of prompts which must be defined are
   *  as follows:
   *    Required core, domain specific:
   *      "nlu" - used when making calls from parseUtteranceLLMService
   *      "nlg" - used when making calls from convertSemanticsToTextLLMService
   *      "validationnlg" - used when making calls from convertSemanticsToTextLLMService while validating a parse result
   *      "validationnlu" - used when making calls from parseUtteranceLLMService while validating a parse result
   *    Available core, general:
   *      "scheduleTask" - Social LLM alternative to templated turn-based scheduleGoal script
   *      TODO: learnAction, reorderGoals, handleHigherPriorityConflictingGoal, modifyScheduledGoal, ...
   *    Domain specific optional:
   *      Any "template targeting dialogue interactions" you wish to implement for your specific domain should be added
   *        as seen fit from a subclass. The current convention is that the key of the chatTemplates entry is the name
   *        of the corresponding action which calls the runContextSpecificChat asl action defined in nlu.asl (which ends
   *        up calling {@link SocialTRLLMServiceProvider#focusDialogueChat(Symbol, Symbol, String, long)}). See
   *        the action scheduleTask for an example. These sorts of chatTemplates allow you to temporarily modify
   *        prompting and tool targets in order to have a focused interaction to gather information to make a specific
   *        tool call (e.g. correctly fully populate more complex target semantics with complex parameterization).
   * TODO: ToolDefinitions per ChatTemplate should be updated with updated system information dynamically - add logic to
   *      use promptVars towards tools in addition to system/user Prompts
   * TODO: make these mostly hardcoded and only have subclass fill in certain fields (e.g. domain summary)
   */
  Map<String,ChatTemplates> chatTemplates;
  //protected abstract List<ToolDefinition> getBaseNLUToolDefinitions();
  //protected abstract void addContextSpecificChatTemplates();

  //// Revisit when making base/subclass distinction
  protected Prompt getNluSystemPrompt() {
    return Prompts.getPrompt("socialava/nlu");
  }
  protected Prompt getNlgSystemPrompt() {
    return Prompts.getPrompt("socialava/nlg");
  }
  protected Prompt getValidationNLGSystemPrompt() {
    return Prompts.getPrompt("socialava/validation");
  }
  protected Prompt getValidationNLUSystemPrompt() {
    return Prompts.getPrompt("socialava/validationnlu");
  }
  //protected abstract Prompt getNluSystemPrompt();
  //protected abstract Prompt getNlgSystemPrompt();
  //protected abstract Prompt getValidationSystemPrompt();
  private Prompt goalTranslationPrompt = Prompts.getPrompt("socialava/goalPredToString");

  /**
   * Explicitly holding REPLY Tool set separate because they will conditionally be added to outgoing parse requests as
   *  targets based on current system questions (explicitly represented by the DialogueComponent)
   */
  Map<String,Tool> replyToolset;

  /**
   * (More recent update with promising results)
   * Universal Tool target for the LLM to respond to a user without terminating the current conversation context. Was
   * having issues where the LLM would not target Tool calls when appropriate and just reported to the user that it had
   * performed the functionality afforded by a specific Tool call. Forcing the LLM to only ever target tool calls rather
   * than conditionally provide text has proven to make INSTRUCT Tool calling when appropriate much more consistent.
   */
  Tool textResponseTool;

  /**
   * Collection currently necessary to handle double side effects of calling a "focused" dialogue script action.
   * Utterances instructing something relevant to a focused dialogue context will always have two effects:
   *  1. The call itself to the method leading to runContextSpecificChat/focusDialogieChat (e.g. scheduleTask)
   *  2. Needing to call the completion again under that specific context/ChatTemplate for the same input and handling
   *     the result.
   *      a) generateResponse - The system also responding to the instruction (prompting the user for necessary information, etc.)
   *              (E.g. "I want to scheduled a task")
   *      b) Underlying tool call - If enough information was supplied upfront, another action call may be made from the focused tool set
   *              (E.g. "I want you to perform the introductory tour every hour starting in 20 minutes")
   *  It's really hard to handle two temporally disjoint side effects from the same input with the current dialogue
   *   pipeline, so this collection + a wrapper submission script associated with runContextSpecificChat is used to
   *   circumvent the current limitations.
   */
  Map<Long,Utterance> contextTransferResponseUtterances = new HashMap<>();

  private boolean registeredForQuestionAskingNotifications = false;
  private String language = "English";

  //Temporary list holding itk action names to detect in dialogue history callback whether UtteranceType Question
  //  was overwritten with INSTRUCT due to ITK / QueueEM incompatibility workarounds.
  List<String> itkActionNames = new ArrayList<>();

  private int externalErrorCount = 0;
  private boolean restrictedMode = false;

  @Override
  public void init() {
    super.init();
    chatTemplates = new HashMap<>();
    List<Tool> instructTools = generateInstructTools();
    List<Tool> itkTools = generateITKTools();
    for (Tool itkTool : itkTools) {
      itkActionNames.add(itkTool.toolDefinition.function.name);
    }
    replyToolset = generateReplyTools();

    //// Generate universal Tools ////
    Tool focusedEndConversationTool = new Tool(new ToolDefinition("endConversation",
            "End the conversation with the user. This is usually desired in two main cases. 1. The user no" +
                    " longer wants to perform your shared task. Or 2. The user has said goodbye or needs to leave.",
            new ArrayList<>()),
            UtteranceType.INSTRUCT
    );
    Tool defaultGoalInfoTool = new Tool(new ToolDefinition("getCurrentPendingScheduledGoalInformation",
            "Use this to ask the system for up to date information about your current list of tasks. Call this" +
                    "tool in order to get relevant information to help answer user questions about what you are currently" +
                    " doing, what you will be doing after your current task is complete, or what you are scheduled to" +
                    "perform in the future",
            new ArrayList<>()),
            UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY);
    itkActionNames.add("getCurrentPendingScheduledGoalInformation");

    //TODO: add current "best guess parse" field similar to what is done now in some "focused" template targeting contexts?
    textResponseTool = new Tool(new ToolDefinition("stillGatheringInformation",
            "You are still gathering information before being able to submit an accurate, fully defined " +
                    "instruction and need to speak with the user to gather missing information.",
            List.of(new ToolParameterHelper("response","string",new ArrayList<>(),
                    "What you want to say to the user", true))),
            UtteranceType.INSTRUCT);

    //// Gather Tool Sets and System Prompts ////
    // Base "nlu" tool set:
    //  - INSTRUCT targets
    //  - LLM<->Diarc ITK targets
    //  - (done dynamically at call-time) relevant REPLY targets
    //  - TODO: have base context history time out after T time with no interaction)
    List<Tool> baseNLUTools = new ArrayList<>(instructTools);
    baseNLUTools.addAll(itkTools);
    baseNLUTools.add(defaultGoalInfoTool);
    baseNLUTools.add(textResponseTool);
    chatTemplates.put("nlu", new ChatTemplates(getNluSystemPrompt(), baseNLUTools));

    // Universal "focused" tool set:
    //  - Template specific targets (gathered at call-time based on entry corresponding to root action name - should be added by subclass)
    //  - Base LLM<->Diarc ITK targets - TODO: temporarily removed - should these be available or not? Only a specific subset?
    //  - relevant REPLY targets - TODO: temporarily removed, should these be allowed? Should there be a mechanism to define REPLY targets per focused context?
    //  - suspend/cancel interaction targets
    List<Tool> universalFocusedTools = new ArrayList<>();
    universalFocusedTools.add(focusedEndConversationTool);
    universalFocusedTools.add(new Tool(new ToolDefinition("suspendCurrentInteraction",
            "You pause your current task, which can be later resumed. An example of instructing this could be \"pause what you're doing.\"" +
                    "This may be instructed directly as shown in the example, or could be intended when the user wants to temporarily have a separate conversation on a different topic.",
            new ArrayList<>()),
            UtteranceType.INSTRUCT, Tool.PostCallBehavior.KEEP_HISTORY));
    universalFocusedTools.add(new Tool(new ToolDefinition("cancelCurrentInteraction",
            "You stop and fully discard your current task. An example of instructing this could be \"stop what you're doing.\"",
            new ArrayList<>()),
            UtteranceType.INSTRUCT));
    universalFocusedTools.addAll(itkTools);

    // Validation tool set:
    //  - Just reply forms, which are added dynamically (yes,no,abort)
    // Q: anything else needed? ITK? Or just always force validation questions to be addressed immediately
    chatTemplates.put("validationnlu", new ChatTemplates(getValidationNLUSystemPrompt(), new ArrayList<>()));

    // NLG tool sets: none
    chatTemplates.put("nlg", new ChatTemplates(getNlgSystemPrompt(), new ArrayList<>()));
    chatTemplates.put("validation", new ChatTemplates(getValidationNLGSystemPrompt(), new ArrayList<>()));

    // Focused context toolsets:
    // Goal Scheduling
    List<Tool> scheduleGoalTools = new ArrayList<>();
    scheduleGoalTools.add(new Tool(new ToolDefinition("submitScheduledTaskDefinition",
            "Submit the details of the fully defined scheduled task for execution. Call this when you believe you have all information about the scheduled task finalized and are ready to save.", List.of(
                new ToolParameterHelper("action","string",new ArrayList<>(),"The action to be performed. Please refer to the Action List in order to understand acceptable values and forms.",true, true, "nlu"),
                new ToolParameterHelper("start","string",new ArrayList<>(),"When to first execute the task. Valid forms are a relative time represented by start(<number>,<timeunit>) or an absolute time represented by start(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss). For example, the relative time of 2 hours from now == start(2,HOURS). The absolute time of noon == start(12-00-00). Being told to perform the first execution immediately == start(0,SECONDS).",true),
                new ToolParameterHelper("repeat","string",new ArrayList<>(),"On what interval to repeat execution of the task. Valid forms are a relative time represented by repeat(<number>,<timeunit>). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. For example, the relative time of 30 minutes from now == repeat(30,MINUTES). If the task should only be performed once and not repeated, supply a value of repeat(nonrepeating).",true),
                new ToolParameterHelper("end","string",new ArrayList<>(),"When to stop repeating execution of the task. If this task is not repeating (meaning repeat is defined as repeat(nonrepeating)), you must supply the value end(nonrepeating). If end(indefinitely) is provided as the value, that means the task will continue repeating execution on the defined repeat until it is explicitly canceled by the user. Otherwise, valid forms are a relative time represented by end(<number>,<timeunit>) or an absolute time represented by end(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss). For example, the relative time of in a quarter of an hour == end(15,MINUTES). The absolute time of 5:30pm == end(17-30-00).",true)
            )),
                    UtteranceType.INSTRUCT
            ));
    scheduleGoalTools.addAll(universalFocusedTools);
    Tool modifiedTextResponseTool1 = new Tool(new ToolDefinition("stillGatheringInformation",
            "You are still being taught new or modifying existing details of the scheduled task by the user. You will respond to" +
                    "the user to acknowledge reception of their most recent command, answer questions they have, or continue" +
                    "to gather missing information in order to define the scheduled task.",
            List.of(new ToolParameterHelper("currentScheduledDefinition","object",new ArrayList<>(),
                            "Your current understanding of the scheduled task being instructed to you. The expected format is json matching the submitScheduledTaskDefinition function described above.", true),
                    new ToolParameterHelper("response","string",new ArrayList<>(),
                            "What you want to say to the user", true))),
            UtteranceType.INSTRUCT);
    scheduleGoalTools.add(modifiedTextResponseTool1);
    //scheduleGoalTools.add(textResponseTool);
    chatTemplates.put("scheduleTask", new ChatTemplates(Prompts.getPrompt("socialava/scheduleGoal"), scheduleGoalTools));
    // Tour Teaching
    // Using custom "stillGatheringInformation" tool which also explicitly keeps track of the "current template parse" at
    //  each step. The idea is that writing it down each step provides something concretely to validate each step and
    //  better context for incremental LLM population. This could be first application/attempt of "general template
    //  population" structure, which we may want to use for at least general action learning if not every context.
    List<Tool> learnTourTools = new ArrayList<>();
    learnTourTools.add(new Tool(new ToolDefinition("submitTaskSequenceDefinition",
            "Submit the details of the fully defined task sequence sequence for storage and future use. Call this when you believe you have all information about the task finalized and are ready to save.", List.of(
            new ToolParameterHelper("taskName","string",new ArrayList<>(),"The name this task should be saved under and later instructed as",true),
            new ToolParameterHelper("locationActionPairs","array",new ArrayList<>(),"The list of action/location pairs defining this task sequence. Execution of this task will consist of iterating over the pairs in this list, visiting each location in sequence, and then performing the corresponding task at that location. Each individual value in this list should be a json object following the form {\"location\":<location_name_here>, \"action\":<tool_call_definition_here>}, where the location field must contain a saved location and the action field must contain a valid tool call definition matching a form in the supplied actionList.",true, true, "nlu")
    )),
            UtteranceType.INSTRUCT
    ));
    learnTourTools.addAll(universalFocusedTools);
    Tool modifiedTextResponseTool2 = new Tool(new ToolDefinition("stillGatheringInformation",
            "You are still being taught new or modifying existing task steps by the user. You will respond to" +
                    "the user to acknowledge reception of their most recent command, answer questions they have, or continue" +
                    "to gather missing information in order to define the task sequence.",
            List.of(new ToolParameterHelper("currentTaskSequenceDefinition","object",new ArrayList<>(),
                    "Your current understanding of the steps of the task being taught to you. The expected format is json matching the submitTaskSequenceDefinition function described above.", true),
                    new ToolParameterHelper("response","string",new ArrayList<>(),
                    "What you want to say to the user", true))),
            UtteranceType.INSTRUCT);
    learnTourTools.add(modifiedTextResponseTool2);
    chatTemplates.put("learnTaskSequence", new ChatTemplates(Prompts.getPrompt("socialava/learnTour"), learnTourTools));

    //TODO: apply promptVars to Tool definitions/examples and most importantly possible_values sets as well
    //// init prompt vars
    // Generating string representations of toolset as reference for NLG and scheduleTask (want to present them, but not actually supply them as targets)
    // TODO: move into initPromptVars
    promptVars.put("%instructNLGList", instructTools.stream().map(t -> toolToDescString(t,false)).collect(Collectors.joining("\n#\n")));
    promptVars.put("%instructList", instructTools.stream().map(t -> toolToDescString(t,true)).collect(Collectors.joining("\n#\n")));
    promptVars.putAll(initPromptVars());

    trackDialogueHistory = true;
    trackLearnedActions = true;
  }

  @Override
  protected void executionLoop() {
    super.executionLoop();
    if (!registeredForQuestionAskingNotifications) {
      try {
        TRADEServiceInfo tsi = getMyService("questionAskingCallback", Symbol.class, Boolean.class, Question.class);
        TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForQuestionAskingNotifications").argTypes(TRADEServiceInfo.class)).call(void.class, tsi);
        registeredForQuestionAskingNotifications = true;
      } catch (TRADEException e) {
        log.error("[LLMServiceProvider] exception registering for question asking notifications, trying again", e);
      }
    }
  }

  /**
   * Initialize Prompt variables whose values are static during runtime or updated by event-based listeners
   */
  protected Map<String,String> initPromptVars() {
    Map<String,String> vars = new LinkedHashMap<>();
    // TODO: If Ava components are ever extended to allow runtime saving of these entities, make corresponding updates
    //    to allow for dynamic updating of prompt vars (ideally through subscriber/listener setup)
    List<String> locations = getLocations("self", true);
    List<String> videos = getVideoNames();
    List<String> images = getImageNames();
    String locationList = String.join(":location\n", locations);
    if (!locationList.isEmpty()) {
      locationList += ":location\n";
    }
    String videoList = String.join("\n", videos);
    if (!videoList.isEmpty()) {
      videoList += "\n";
    }
    String imageList = String.join("\n", images);
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
    return vars;
  }

  /**
   * Update prompt variables whose values need to be explicitly retrieved at call time
   * TODO: a lot of these can be updated to be held internally and updated on events
   */
  protected void updatePromptVars(Utterance utterance, boolean parsing) {
    Symbol agent = parsing ? utterance.getAddressee() : utterance.getSpeaker();
    Symbol user = parsing ? utterance.getSpeaker() : utterance.getAddressee();
    if (parsing) {
      //promptVars.put("%worldState", getWorldStateString(agent));
      promptVars.put("%questionList", getCurrentQuestionsString(agent));
      ChatCollection chatCollection = getChatCollection(user,agent);
      if (chatCollection != null) {
        ChatContext chatContext = chatCollection.currentChat;
        if (chatContext != null && chatContext.chatState == ChatState.VALIDATION) {
          //TODO: include tool information, not parsed semantics
          Map<Variable, Symbol> varMap = new HashMap<>();
          for (Term t : chatContext.utteranceUpForValidation.getSupplementalSemantics()) {
            varMap.put((Variable) t.get(0), Factory.createSymbol(t.getName(), t.getType()));
          }
          promptVars.put("%toolUpForValidationDescription",replaceVarsWithDescriptors((Predicate) chatContext.utteranceUpForValidation.getSemantics(), varMap).toString());
        }
      }
    }
    promptVars.put("%addressee", agent.getName());
  }

  ///////////////////////////////////
  //// NLU methods and utilities ////
  ///////////////////////////////////
  public Utterance parseUtteranceLLMService(Utterance utterance) {
    return parseUtteranceLLMService(utterance, false);
  }

  /**
   * Main NLU method used by LLMParserComponent
   * @param utterance Utterance with populated words to parse to semantics
   * @param reattemptedParse Whether this is a followup call after negative user validation (so we do not update the
   *                         Chat history with the same utterance more than once)
   * @return The utterance with populated semantics, or null if no valid parse is found
   */
  public Utterance parseUtteranceLLMService(Utterance utterance, boolean reattemptedParse) {
    //// Get existing chat collections or create new if none exists
    ChatCollection chatCollection;
    ChatContext chatContext;
    Chat chat;
    Symbol user = getUntypedSymbol(utterance.getSpeaker());
    Symbol agentTeam = getUntypedSymbol(utterance.getAddressee());
    synchronized (currentChatCollections) {
      chatCollection = getChatCollection(user, agentTeam);
      if (chatCollection == null) {
        chatCollection = new ChatCollection(new Chat());
        currentChatCollections.put(Pair.of(user.toUntyped(), agentTeam.toUntyped()), chatCollection);
      }
    }

    // New: Synchronization - need to wait for any previous chat inputs and corresponding side effects to finish being
    // processed before applying this one
    if (!reattemptedParse) {
      if (!chatCollection.grabInputLock(utterance)) {
        // Dialogue history update showed that external parser completed before reaching this point. Do not need to proceed.
        return utterance;
      }
    }

    List<ToolDefinition> tools;
    synchronized (currentChatCollections) {
      // Check for external parser notification between grabbing input lock and synchronized block
      if (!chatCollection.shouldProcess(utterance)) {
        chatCollection.signalInputProcessed(utterance); //Failsafe - revisit input lock grab
        return utterance;
      }

      // If previous input caused a context to be terminated, need to generate a new one. Otherwise, continue with
      // existing state
      chatContext = chatCollection.currentChat;
      if (chatContext == null) {
        chat = new Chat();
        chatContext = new ChatContext(chat);
        chatCollection.currentChat = chatContext;
      }

      //// Update chats based on current state
      Prompt systemTemplate;
      ChatTemplates chatTemplates = this.chatTemplates.get(chatContext.contextID);
      if (chatTemplates == null) {
        log.error("[parseUtteranceLLMService] could not find chat template for contextID {}", chatContext.contextID);
        return utterance;
      }
      // Pull correct template and update system message and tool targets based off current state
      if (chatContext.chatState == ChatState.VALIDATION) {
        systemTemplate = new Prompt(this.chatTemplates.get("validationnlu").systemMessageTemplate.getText());
        tools = new ArrayList<>();
        Tool focusedTextResponseTool = chatTemplates.tools.get(textResponseTool.toolDefinition.function.name);
        if (focusedTextResponseTool != null) {
          tools.add(focusedTextResponseTool.toolDefinition);
        } else {
          tools.add(textResponseTool.toolDefinition);
        }

        // ValidationNLU has chat history as part of system message. Otherwise, content in the history will be parsed as validation responses.
        //  Creating a new chat just for validation, but maintaining proper history in Chat stored in this ChatContext in case of negative validation and continued interaction
        chat = new Chat();
        promptVars.put("%chatHistoryAsString", getChatHistoryAsString(chatContext.history));
        chatContext.history.addUserMessage(utterance.getWordsAsString());
      } else {
        systemTemplate = new Prompt(chatTemplates.systemMessageTemplate.getText());
        tools = new ArrayList<>(chatTemplates.toolDefinitions);
        chat = chatContext.history;
      }

      // TODO: Had written this way, but is there a reason to explicitly hide these tools when in a focused context?
      if (!(chatContext.chatType == ChatType.FOCUSED && chatContext.chatState != ChatState.VALIDATION)) {
        // Add all matching system question response tools based off of expected reply semantics
        tools.addAll(getQuestionResponseToolDefinitions(agentTeam));
      }

      // Apply input as user message
      if (!reattemptedParse) {
        chat.addUserMessage(utterance.getWordsAsString());
      }

      // Update and apply prompt vars
      updatePromptVars(utterance, true);
      chat.setSystem(systemTemplate.apply(promptVars));
    }

    //// Send created chat to LLM completion
    Completion response = callChatCompletion(chat, tools);

     if (response == null) {
      log.error("[parseUtteranceLLMService] have null Completion response");
      handleErrorResponse(utterance, true, false);
      return utterance;
    } else {
      resetErrorCount(false, user, agentTeam);
    }

    log.info("[parseUtteranceLLMService] num tool calls: {}, text response: {}", response.toolCalls.size(), response.text);

    synchronized (currentChatCollections) {
      // Check for external parser notification between grabbing input lock and this synchronized block
      if (!chatCollection.shouldProcess(utterance)) {
        log.info("[parseUtteranceLLMService] external parser hit, returning without processing {}", response.toolCalls);
        chatCollection.signalInputProcessed(utterance); //Failsafe - revisit input lock grab
        return utterance;
      }

      StringBuilder errorResponseBuilder = new StringBuilder();

      //TODO: handling multiple tool calls present
      //TODO: handling text response and tool call(s) present
      //// Handle Responses
      Utterance output;
      if (response.toolCalls.isEmpty()) {
        // Handle text response - always handled the same regardless of state (superfluous dialogue, extended parse attempt, extended validation)
        //    Add robot message to chat and return generateResponse semantics with no validation required
        output = handleTextResponse(response, utterance, chatContext, errorResponseBuilder);
      } else {
        if (chatContext.chatState == ChatState.VALIDATION) {
          // Validation Tool Call - return appropriate reply semantics and modify chatCollection state as needed (possible
          //  to terminate and/or transfer between base and focused contexts in this case)
          output = handleValidationResponse(response, utterance, chatCollection, errorResponseBuilder);
        } else {
          // Tool Call - extended parse recovery on invalid parse, or enter validation of valid parse
          output = handleToolResponse(response, utterance, chatCollection, errorResponseBuilder);
        }
      }

      if (output == null) {
        output = runCompletionRetryLoop(response, tools, utterance, chatCollection, errorResponseBuilder);
      }
      return output;
    }
  }

  //TODO: revisit - only add calls to this if in synchronized block and understand what you're doing
  private Utterance runCompletionRetryLoop(Completion lastResponse, List<ToolDefinition> toolList, Utterance utterance, ChatCollection chatCollection, StringBuilder errorResponseBuilder) {
    Symbol user = utterance.getAddressee().toUntyped();
    Symbol agentTeam = utterance.getSpeaker().toUntyped();
    ChatContext chatContext = chatCollection.currentChat;
    int numRetries = 0;
    Utterance output;
    ToolCall lastToolCall = (lastResponse != null && !lastResponse.toolCalls.isEmpty()) ? lastResponse.toolCalls.get(0) : null;
    String tool_call_id;
    while (numRetries < 3) {
      // Experimenting with keeping tool calls explicitly in the history - previously had issues with the LLM calling
      //  tools which didn't exist anymore in different contexts but were present in the history. Have more safeguards
      //  in place now so that it should be safe
      if (!errorResponseBuilder.isEmpty()) {
        if (lastToolCall != null && lastToolCall.function.name.equals(textResponseTool.toolDefinition.function.name)) {
          // Need to add errant tool call so that added error message actually references something?
          chatContext.history.addRobotMessage("", List.of(lastToolCall));
        }
        String errorString = errorResponseBuilder.toString();
        log.info("[runCompletionRetryLoop] running extra chatCompletion request with additional error message: {}", errorString);
        tool_call_id = lastToolCall == null ? "" : lastToolCall.id;
        chatContext.history.addToolMessage(errorString, tool_call_id);
      }
      errorResponseBuilder = new StringBuilder();

      // Re-call completion for text response //TODO: pass in tool list? allow for more than just text response?
      Completion retryResponse = callChatCompletion(chatContext.history, toolList);

      if (retryResponse == null) {
        handleErrorResponse(utterance, true, false);
        return utterance;
      } else {
        resetErrorCount(false, user, agentTeam);
        if (retryResponse.toolCalls.isEmpty()) {
          lastToolCall = null;
          resetErrorCount(true, utterance.getSpeaker().toUntyped(), utterance.getAddressee().toUntyped());
          output = handleTextResponse(retryResponse, utterance, chatContext, errorResponseBuilder);
        } else {
          lastToolCall = retryResponse.toolCalls.get(0);
          output = handleToolResponse(retryResponse, utterance, chatCollection, errorResponseBuilder);
        }
        if (output != null) {
          return output;
        }
      }
      numRetries += 1;
      log.warn("[parseUtteranceLLMService] got null response, retrying completion request with added tool error message");
    }
    handleErrorResponse(utterance, true, true);
    return utterance;
  }

  /**
   * Method for handling Completion responses containing Tool calls, 3 main cases:
   *  - Base: Return an Utterance populated with the Tool's underlying semantics. In the case of an external cache hit,
   *          the context will be discarded and handleValidatedToolResponse will be called from the corresponding dialogue history update.
   *  - LLM ITK - will return semantics for goal without validation (Underlying semantics should match a SKIPPENDING priority goal which calls {@link #addToolResponse(Symbol, Symbol, String, String)})
   *  - Invalid parse - Add a system error message to the Chat, call the completion again, and return a text response to the user with the result
   *  - *NEW* Stand-in tool(s) for text responses - Experimenting targeting a specific tool to give text responses rather than using
   *      handleTextResponse above with promising initial success. Allowing the LLM to target text responses or tool calls was giving it a
   *      hard time calling tools rather than just saying it performed the underlying functionality.
   */
  public Utterance handleToolResponse(Completion llmResponse, Utterance inputUtterance, ChatCollection chatCollection, StringBuilder errorResponseBuilder) {
    if (llmResponse.toolCalls.size() > 1) {
      //TODO
      log.warn("[handleToolResponse] Multiple tool calls returned -> TODO handle both. Only using first...");
    }
    ToolCall toolCall = llmResponse.toolCalls.get(0);
    for (ToolCall tc: llmResponse.toolCalls) {
      log.info("[handleToolResponse] " + tc);
    }

    ChatContext chatContext = chatCollection.currentChat;

    //Experimental - adding tool calls back into history now that we have safeguards against them being called from
    // incorrect contexts when no longer available. Text response tool calls will be added from the dialogue history listener.
    if (!toolCall.function.name.equals(textResponseTool.toolDefinition.function.name)) {
      //TODO: make sure it is okay to not include text response calls here. Believe the only time it
      //    really matters would be on malformed/invalid target.
      chatContext.history.addRobotMessage("", List.of(toolCall));
    }

    //// Check that tool response is valid
    ToolFunction functionCall = toolCall.function;
    Tool correspondingTool = null;
    try {
      if (chatContext.chatType == ChatType.FOCUSED) {
        correspondingTool = chatTemplates.get(chatContext.contextID).tools.get(functionCall.name);
      } else {
        correspondingTool = chatTemplates.get("nlu").tools.get(functionCall.name);
        if (correspondingTool == null) {
          correspondingTool = replyToolset.get(functionCall.name);
        }
      }
    } catch (NullPointerException e) {
      log.error("[handleToolResponse] error finding Tool definition for called tool found in Completion response", e);
    }

    // Internal LLM error - should there be LLM specific error responses in this case?
    if (correspondingTool == null) {
      errorResponseBuilder.append("- Your chosen tool \"").append(toolCall.function.name).append("\" is no longer available for you to call in your current context. Please find an alternate specific tool call to use which is explicitly present in the tool list, or respond using the " + textResponseTool.toolDefinition.function.name + " tool");
      return null;
    }

    if (errorResponseBuilder.isEmpty()) {
      errorResponseBuilder.append("Your most recent tool call with id ").append(toolCall.id).append(" contains missing or invalid information. Please correct the following errors to supply" +
              " valid json to make the desired tool call. Solve these issues on your own where possible. If you must ask" +
              " the user for missing or clarifying information, precede your question by saying that your interpretation" +
              " of the instruction contained invalid information:\n");
    }


    // Process the tool call and supplied arguments - output utterance will be null and baseToolResponse will be updated
    //  with error details in case any invalid response was supplied by the LLM
    Utterance outputUtterance = processToolCall(functionCall, correspondingTool, inputUtterance, errorResponseBuilder);

    boolean error = false;
    if (outputUtterance != null && outputUtterance.getSemantics() != null) {
      //// Domain specific postprocessing to correct common mistakes
      String errorResponse = correctCommonNLUMistakes(outputUtterance);
      if (!errorResponse.isEmpty()) {
        error = true;
        errorResponseBuilder.append(errorResponse);
      }
    } else {
      error = true;
    }

    //// Handle processed response
    if (!error) {
      //Valid parse - proceed to validation

      // Experimental text response tool stand-in (described above)
      if (toolCall.function.name.equals(textResponseTool.toolDefinition.function.name)) {
        if (!outputUtterance.getSemantics().isPredicate() || ((Predicate)outputUtterance.getSemantics()).getArgs().size() < 2) {
          // Is this case even possible anymore?? Don't think so. Think would have to be developer error, otherwise would
          // have been caught in tool postprocessing.
          log.error("[handleToolResponse] malformed text response tool call **Believe this must be due to Developer Error**");
          errorResponseBuilder.append("- your " + textResponseTool.toolDefinition.function.name + " tool call needs to be properly formatted json including a response field");
          return null;
        }
        llmResponse.text = ((Predicate)outputUtterance.getSemantics()).get(1).toString();
        if(llmResponse.text.startsWith("\"") && llmResponse.text.endsWith("\"")) {
          llmResponse.text = llmResponse.text.substring(1, llmResponse.text.length() - 1);
        }
        return handleTextResponse(llmResponse,inputUtterance,chatContext, errorResponseBuilder);
      }

      //// Universal postprocessing for accepted cases
      // TODO: Reevaluate if these are in fact conventions we want to universally accept / how we want to do these things
      String toolName = outputUtterance.getSemantics().getName();
      chatContext.lastToolCallName = toolName;

      // Temporary special wrappers - revisit if ever reintroducing "offContext" functionality
      if (chatContext.chatType != ChatType.BASE) {
        if (toolName.equals("endConversation") || toolName.equals("cancelCurrentInteraction")) {
          //outputUtterance.setSemantics(Factory.createPredicate("cancelSystemGoals", inputUtterance.getAddressee()));
          toolName = "cancelSystemGoals";
        } else if (toolName.equals("suspendCurrentInteraction")) {
          //outputUtterance.setSemantics(Factory.createPredicate("suspendSystemGoalsBlocking", inputUtterance.getAddressee()));
          toolName = "suspendSystemGoalsBlocking";
        }
      }
      if (toolName.equals("cancelSystemGoals") || toolName.equals("suspendSystemGoalsBlocking")) {
        List<Long> gids = null;
        try {
          gids = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSystemGoalsIds").argTypes(Symbol.class)).call(List.class, inputUtterance.getAddressee());
        } catch (TRADEException e) {
          log.error("[handleToolResponse] exception getting current system goal ID for agentTeam: {}", inputUtterance.getAddressee(), e);
        }
        if (gids != null && !gids.isEmpty()) {
          Long gid = gids.get(0);
          if (toolName.equals("suspendSystemGoalsBlocking")) {
            outputUtterance.setSemantics(Factory.createPredicate("suspendGoal", inputUtterance.getAddressee(), Factory.createSymbol(String.valueOf(gid))));
          } else {
            outputUtterance.setSemantics(Factory.createPredicate("cancelGoal", inputUtterance.getAddressee(), Factory.createSymbol(String.valueOf(gid))));
          }
        }
      }

      // Proceed to validation if not architecture ITK call
      if (outputUtterance.getType() == UtteranceType.QUESTION) {
        outputUtterance.setType(UtteranceType.INSTRUCT);
        outputUtterance.setNeedsValidation(false);
      } else if (outputUtterance.getType() == UtteranceType.REPLY) {
        outputUtterance.setType(UtteranceType.REPLY);
        outputUtterance.setNeedsValidation(false);
      } else {
        if (outputUtterance.needsValidation()) {
          chatContext.chatState = ChatState.VALIDATION;
          chatContext.utteranceUpForValidation = outputUtterance; //copy?
          chatContext.toolCallIdUpForValidation = toolCall.id;
        } else {
          // noConfirmation case - deferred until dialogue history update in case of external parser hit
          //handleValidatedToolCall(outputUtterance, chatCollection);
        }
      }

      resetErrorCount(true, inputUtterance.getSpeaker().toUntyped(), inputUtterance.getAddressee().toUntyped());
      return outputUtterance;
    } else {
      // Invalid parse - engage in recovery in the form of an extended dialogue interaction
      return null;
    }
  }

  /**
   * Method for handling Completion responses containing Text responses. Add directly as assistant message to history
   *  and return generateResponseFromString semantics with needsValidation set to false
   */
  public Utterance handleTextResponse(Completion llmResponse, Utterance inputUtterance, ChatContext chatContext, StringBuilder errorResponseBuilder) {
    String responseString = llmResponse.getText().trim();

    if (responseString.contains("<tool_call>")) {
      //TODO: attempt to remove unmatched tags and process as a tool json string?
      log.warn("[handleTextResponse] detected uncaught tool call in completion text response");
      errorResponseBuilder.append("- Your initial response \"").append(responseString).append("\" includes a <tool_call> tag but was not picked up by the tool call parser. Please make sure your response is formatted correctly to be picked up by the tool parser. For example, prevent any white space between tags. If this tag is found elsewhere in your response, please use a different value as this is a reserved character sequence.");
      return null;
    } else if (responseString.startsWith("{") && responseString.endsWith("}")) {
      //TODO: attempt to process as a tool call?
      log.warn("[handleTextResponse] detected unexpected json in completion text response");
      //TODO: what if a json string was actually meant to be supplied as a response?
      errorResponseBuilder.append("- Your initial response \"").append(responseString).append("\" looks like json was supplied but was not picked up by the tool call parser. Please wrap your output in <tool_call> tags so it is recognized by the parser.");
      return null;
    }
    // TODO: need more postprocessing cases here for various special characters?

    chatContext.lastResponseString = responseString;

    if(!(responseString.startsWith("\"") && responseString.endsWith("\""))) {
      responseString = "\"" + responseString + "\"";
    }

    // Updating response from dialogue history now - in case this parse result is not used
    Predicate generateResponsePred = Factory.createPredicate("generateResponseFromString",inputUtterance.getAddressee().toString(), inputUtterance.getSpeaker().toString(), responseString);
    inputUtterance.setSemantics(generateResponsePred);
    inputUtterance.setType(UtteranceType.INSTRUCT);
    inputUtterance.setNeedsValidation(false);
    return inputUtterance;
  }

  // (Don't need to do any disjoint updates/checking for external cache hits because assuming we are already using LLM only by this point)
  /**
   * Method for handling Completion responses containing during validation.
   *
   * TODO: currently not validating validation responses - change if problematic
   */
  public Utterance handleValidationResponse(Completion llmResponse, Utterance inputUtterance, ChatCollection chatCollection, StringBuilder errorResponseBuilder) {
    if (llmResponse.toolCalls.size() > 1) {
      log.warn("[handleValidationResponse] Multiple tool calls returned -> TODO handle both. Currently only using first...");
    }
    ToolCall toolCall = llmResponse.toolCalls.get(0);
    for (ToolCall tc: llmResponse.toolCalls) {
      log.info("[handleValidationResponse] " + tc);
    }

    // Tool -> semantics. Yes/No/Abort
    //  Context specific prompt - no special cases. Set utterance semantics and terminate chat if validated or aborted
    //  Base prompt - special cases based on whether context specific prompt exists or not
    Utterance.Builder output = new Utterance.Builder(inputUtterance);
    ChatContext chatContext = chatCollection.currentChat;

    // Experimental text response tool stand-in (described above)
    if (toolCall.function.name.equals(textResponseTool.toolDefinition.function.name)) {
      return handleToolResponse(llmResponse, inputUtterance, chatCollection, errorResponseBuilder);
    }

    // Handle expected validation question yes/no response semantics
    if (toolCall.function.name.equals("val") || toolCall.function.name.equals("validated")) {
      String argument = toolCall.function.arguments.contains("yes") ? "yes" : "no";
      output.setSemantics(Factory.createPredicate("val",argument));
      output.setNeedsValidation(false);
      output.setUtteranceType(UtteranceType.REPLY);

      // If positively validated, terminate associated context and submit instruction
      if (argument.equals("yes")) {
        handleValidatedToolCall(chatContext.utteranceUpForValidation, chatCollection);
      } else {
        chatContext.lastToolCallName = "";
        // Negative validation - reset to PARSE state to continue interaction. Architecture messages and other
        //  side effects will be triggered from reattemptParse / the NLU asl pipeline
        chatContext.chatState = ChatState.PARSE;
      }
    } else if (toolCall.function.name.equals("abort")) {
      // User declined to answer the validation question - terminate context entirely and refocus base parse state
      output.setSemantics(Factory.createPredicate("abort(question)"));
      output.setNeedsValidation(false);
      output.setUtteranceType(UtteranceType.REPLY);
      chatCollection.currentChat.signalCompletion();
      saveContextHistory(inputUtterance.getSpeaker(), inputUtterance.getAddressee(), chatCollection.currentChat);
      chatCollection.currentChat = null;
    }
    return output.build();
  }

  private void handleValidatedToolCall(Utterance validatedUtterance, ChatCollection chatCollection) {
    Symbol user;
    Symbol agentTeam;
    if (validatedUtterance.isInputUtterance()) {
      user = validatedUtterance.getSpeaker();
      agentTeam = validatedUtterance.getAddressee();
    } else {
      user = validatedUtterance.getAddressee();
      agentTeam = validatedUtterance.getSpeaker();
    }
    handleValidatedToolCall(validatedUtterance.getSemantics().getName(), user, agentTeam, chatCollection);
  }

  /**
   * Method to perform relevant side effects for a validated (or noConfirmation) tool call
   */
  private void handleValidatedToolCall(String toolName, Symbol user, Symbol agentTeam, ChatCollection chatCollection) {
    ChatContext chatContext = chatCollection.currentChat;

    // If tool is targeting a 'focused dialogue action', store the current chat history for retrieval by that action
    if (chatTemplates.containsKey(toolName)) {
      // Kind of a hack? Can remove if extending synchronization around tentativeAccept/Submission dialogue or figuring
      //  out another solution for integrating NLG into relevant chats.
      // Remove potential excess system NLG for unrelated goals before storing this chat
      ChatContext copy = new ChatContext(chatContext);
      for (int i = copy.history.messages.size() - 1; i > 0; i--) {
        if (copy.history.messages.get(i).role.equals(copy.history.userName)) {
          break;
        }
        copy.history.messages.remove(i);
      }

      // Storing submission time so that action can conditionally add a disclaimer stating that this instruction
      //  was given T time ago (because the goal may end up being stuck in the queue for an arbitrary amount of time)
      copy.submissionTime = System.currentTimeMillis();
      chatCollection.pendingFocusedChats.put(toolName, copy);
    }

    Tool.PostCallBehavior postCallBehavior = Tool.PostCallBehavior.CLEAR_CONTEXT;
    ChatTemplates ct = chatTemplates.get(chatCollection.currentChat.contextID);
    if (ct != null) {
      Tool calledTool = ct.tools.get(toolName);
      if (calledTool == null) {
        calledTool = ct.tools.get(chatContext.lastToolCallName);
      }
      if (calledTool != null) {
        postCallBehavior = calledTool.postCallBehavior;
      }
    }

    switch (postCallBehavior) {
      case CLEAR_CONTEXT:
        // Wait on two-way lock so thread waiting on focused chat context to terminate can get any relevant result information
        chatCollection.currentChat.signalCompletion();
        saveContextHistory(user, agentTeam, chatCollection.currentChat);
        chatCollection.currentChat = null;
        break;
      case CLEAR_HISTORY:
        saveContextHistory(user, agentTeam, chatCollection.currentChat);
        //TODO: merge into "previous history" promptVar state? Clear "previous history" promptVar completely?
        chatCollection.currentChat.history.messages.clear();
      case KEEP_HISTORY:
        // Reset to parse state for future input
        chatCollection.currentChat.chatState = ChatState.PARSE;
        // Reset all "last tool call input" details
        chatCollection.currentChat.toolCallIdUpForValidation = "";
        chatCollection.currentChat.utteranceUpForValidation = null;
        chatCollection.currentChat.lastToolCallName = "";
    }
  }

  /**
   * Method for HybridParser/NLG components to prevent use of other parser/NLG components once entering an
   * extended interaction
   */
  public boolean inExtendedInteraction(Symbol user, Symbol agentTeam) {
    if (restrictedMode) {
      return false;
    }

    ChatCollection collection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
    // Without this, there could be an issue here if any TLDL greeting, reply, ack responses are present in the history
    //  but the LLM is failing calls to the endpoint. Would be returning true, but because of population from a tldl call
    if (collection != null) {
      ChatContext chatContext = collection.currentChat;
      if (chatContext != null) {
        return chatContext.containsLLMParse || chatContext.chatType != ChatType.BASE; // exclude error responses?
      }
    }
    return false;
  }

  // Revisit/remove
  @TRADEService
  public void usedLLMParse(Symbol user, Symbol agentTeam) {
    ChatCollection collection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
    if (collection != null) {
      ChatContext chatContext = collection.currentChat;
      if (chatContext != null) {
        chatContext.containsLLMParse = true;
      }
    }
  }

  /**
   * Method used in nlu.asl to understand on positive validation of a command whether the command was a first-time
   * instruction or part of an extended, contextual dialogue. Necessary to know what Utterance's are safe to cache.
   */
  @Override
  public int getLLMConversationLength(Symbol user, Symbol agentTeam) {
    ChatCollection collection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
    if (collection == null || collection.currentChat == null) {
      return 0;
    }
    return collection.currentChat.history.messages.size();
  }

  /**
   * Method called from nlu.asl after negative validation of an LLM parse. Adds a system message informing the LLM of
   * what occurred and calling the completion endpoint again with the user response. May result in an updated tool call
   * or a text response for the user.
   */
  @TRADEService
  public boolean reattemptParse(Utterance utterance) {
    ChatCollection chatCollection;
    ChatContext currentContext;
    synchronized (currentChatCollections) {
      Symbol user = utterance.getSpeaker().toUntyped();
      Symbol agentTeam = utterance.getAddressee().toUntyped();
      chatCollection = getChatCollection(user, agentTeam);
      currentContext = chatCollection.currentChat;
    }
    if (currentContext == null || currentContext.chatState != ChatState.PARSE) {
      log.error("[reattemptParse] called with incompatible parse state");
      // Should we reset state here? Fail loudly? Currently would fail silently when used in the intended pipeline.
      Predicate generateResponsePred = Factory.createPredicate("generateResponseFromString",utterance.getAddressee().toString(), utterance.getSpeaker().toString(), "\"Sorry, an internal error has occurred. Could you repeat that?\"");
      utterance.setSemantics(generateResponsePred);
      utterance.setType(UtteranceType.INSTRUCT);
      utterance.setNeedsValidation(true);
      utterance.setIsValidated(false);
      return false;
    }

    Symbol semantics = utterance.getSemantics().clone();
    if (semantics.isPredicate() && utterance.getSupplementalSemantics() != null) {
      ////Note: if including tool calls in the history, this message needs to include matching tool_call_id
      // Add tool response stating what was negatively validated by the user
      Map<Variable, Symbol> varMap = new HashMap<>();
      for (Term t : utterance.getSupplementalSemantics()) {
        varMap.put((Variable) t.get(0), Factory.createSymbol(t.getName(), t.getType()));
      }
      semantics = replaceVarsWithDescriptors((Predicate)semantics, varMap);
    }
    currentContext.history.addToolMessage("The user has just informed you that your previous interpretation of their command " +
            "which resulted in your tool call with id " + currentContext.toolCallIdUpForValidation + " represented by " + semantics + " was not what they intended to instruct. Use the last " +
            "user message to determine one of two ways to proceed: 1. If the user's message includes additional information clarifying " +
            "their original intention, then you may update your definition with the new information and attempt another updated tool submission call. " +
            "2. Otherwise, please continue with your original task to resolve the misunderstanding. In this case, only " +
            "ask the user how or what to correct in one short sentence.", currentContext.toolCallIdUpForValidation);

    // Call completion with user response and tool message
    Utterance tempUtterance = new Utterance.Builder().setSpeaker(utterance.getSpeaker()).setAddressee(utterance.getAddressee()).setWords(utterance.getWords()).build();
    tempUtterance.setNeedsValidation(true);
    tempUtterance = parseUtteranceLLMService(tempUtterance, true);

    if (tempUtterance.getSemantics() == null) {
      log.warn("[reattemptParse] original utterance returned from inner parse method - indicates exception occurred");
      tempUtterance.setSemantics(Factory.createPredicate("generateResponseFromString",utterance.getAddressee().toString(), utterance.getSpeaker().toString(), "\"Sorry, an internal error has occurred. What did you mean to instruct me to do?\""));
      tempUtterance.setNeedsValidation(false);
      tempUtterance.setType(UtteranceType.INSTRUCT);
    } else if (tempUtterance.getType() == UtteranceType.UNKNOWN) {
      tempUtterance.setType(UtteranceType.INSTRUCT);
    }

    //TODO: better way to do this/prevent doing this - needs to be updated by reference as of now
    utterance.setSemantics(tempUtterance.getSemantics());
    utterance.setBindings(tempUtterance.getBindings());
    utterance.setSupplementalSemantics(tempUtterance.getSupplementalSemantics());
    utterance.setIndirectSemantics(tempUtterance.getIndirectSemantics());
    utterance.setTierAssignments(tempUtterance.getTierAssignments());
    utterance.setNeedsValidation(tempUtterance.needsValidation());
    utterance.setType(tempUtterance.getType());

    chatCollection.utteranceBeingProcessed = null;
    boolean ret = utterance.needsValidation();
    if (!ret) {
      // This is kind of weird, but we need to set isValidated to true even for things normally not needing validation
      //  (e.g. LLM text responses) so that outer loop in NLU.asl doesn't discard the utterance.
      utterance.setIsValidated(true);
    }
    return ret;
  }

  ///////////////////////////////////
  //// NLG methods and utilities ////
  ///////////////////////////////////
  public Utterance convertSemanticsToTextLLMService(Utterance utterance) {
    return convertSemanticsToTextLLMService(utterance,true);
  }

  /**
   * Main NLG method used by LLMNLGComponent
   * @param utterance Utterance with populated semantics to transform to words
   * @param addRobotMessage Whether the generated text result should be added to the corresponding ChatContext as an
   *                        assistant message. (i.e. currently true if occurring as part of a dialogue interaction between a user
   *                        and robot, false if part of a system-LLM ITK message generation).
   * @return The utterance with populated words, or null if no generation could be formed
   */
  @TRADEService
  public Utterance convertSemanticsToTextLLMService(Utterance utterance, boolean addRobotMessage) {
    ////// Get existing chats
    //TODO: revisit ChatTemplates and PromptVars relationship with NLG side as opposed to NLU side to reduce inefficiency
    Symbol user = utterance.getAddressee().toUntyped();
    Symbol agentTeam = utterance.getSpeaker().toUntyped();

    updatePromptVars(utterance, false);

    Symbol semantics;
    List<Term> boundSemantics = utterance.getBoundSemantics();
    if (boundSemantics.isEmpty()) {
      semantics = utterance.getSemantics();
    } else if (boundSemantics.size() == 1) {
      semantics = boundSemantics.get(0);
    } else {
      semantics = Factory.createPredicate("and", boundSemantics);
    }
    if (semantics == null) {
      log.error("[convertSemanticsToTextLLMService] supplied utterance has null semantics");
      return utterance;
    }

    ChatTemplates chatTemplates = null;
    ChatCollection chatCollection = getChatCollection(user, agentTeam);
    if (chatCollection != null) {
      ChatContext chatContext = chatCollection.currentChat;
      if (chatContext != null) {
        promptVars.put("%chatHistoryAsString",getChatHistoryAsString(chatContext.history));
        if (chatContext.chatState == ChatState.VALIDATION) {
          chatTemplates = this.chatTemplates.get("validation");
        }
      }
    } else {
      promptVars.put("%chatHistoryAsString","<There is no relevant chat history>");
    }
    promptVars.put("%user", user.getName());
    if (chatTemplates == null) {
      chatTemplates = this.chatTemplates.get("nlg");
    }

    semantics = preprocessNLGSemantics(semantics);

    // Supplying history in the system prompt itself as to not confuse the LLM - need to supply history for context, but
    //    the NLU history would contradict the stated task in this system prompt.
    // Create a new chat as to not destroy history
    Chat genChat = new Chat();
    utterance = new Utterance.Builder(utterance).build();
    if (semantics.isPredicate()) {
      semantics = replaceReferencesInSemantics((Predicate) semantics);
    }
    utterance.setSemantics(semantics);
    genChat.addUserMessage(utterance.toString());
    genChat.setSystem(chatTemplates.systemMessageTemplate.apply(promptVars));
    Completion llmResponse = callChatCompletion(genChat, null);

    if (llmResponse == null) {
      log.error("[convertSemanticsToTextLLMService] received null completion response");
      handleErrorResponse(utterance, false, false);
      return utterance;
    } else {
      resetErrorCount(false, user, agentTeam);
    }

    log.info(llmResponse.getText());
    String responseString = llmResponse.getText().trim();

    //TODO: Still need this output format? Or should it just be direct?
    Pattern outputPattern = Pattern.compile(".*\"?translation\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL);
    Matcher m = outputPattern.matcher(responseString);
    if (!m.matches()) {
      log.error("[parseNLGCompletionResponse] cannot find Output field in LLM response");
      return null;
    } else {
      String output = m.group(m.groupCount());
      utterance.setWords(List.of(output.split(" ")));
    }

    return utterance;
  }

  private Symbol preprocessNLGSemantics(Symbol input) {
    if (input != null) {
      String inputString = input.toString();
      for (String patternString : List.of("(cancelGoal)\\((.*),(\\d+)\\)", "(suspendGoal)\\((.*),(\\d+)\\)")) {
        Pattern pattern = Pattern.compile(patternString);
        Matcher matcher = pattern.matcher(inputString);
        if (matcher.find()) {
          String command = matcher.group(1);
          String actor = matcher.group(2);
          String gid = matcher.group(3);
          try {
            GoalInfo goalInfo = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalInfo").argTypes(Long.class)).call(GoalInfo.class, Long.valueOf(gid));
            Predicate goalPred = replaceReferencesInSemantics(goalInfo.goal);
            String updatedString = inputString.replaceAll(patternString, command+"(" + actor + "," + goalPred.toString() + ")");
            Symbol updatedSymbol = Factory.createSymbol(updatedString);
            if (updatedSymbol != null) {
              return updatedSymbol;
            }
          } catch (Exception e) {
            log.warn("[preprocessNLGSemantics] exceptions while replacing goalId with corresponding predicate");
          }
        }
      }
    }
    return input;
  }

  /////////////////////////////////////////////////
  //// 'Focused context' methods and utilities ////
  /////////////////////////////////////////////////
  /**
   * Transfer to a focused context chat. Only called through runContextSpecificChat in nlu.asl - specific assumptions
   * and logic made around this method, do not call this method from elsewhere without fully understanding what you are
   * doing.
   */
  @TRADEService
  public Utterance focusDialogueChat(Symbol user, Symbol agentTeam, String contextIdentifier, long cid) {
    ChatContext chatContext;
    ChatCollection collection;
    Symbol speaker;
    Symbol addressee;
    // Get System Prompt Template and Tool set for focused context - convention is for a ChatTemplate to be added
    //  on init with the entry key == the name of the action causing this call
    ChatTemplates contextSpecificTemplate = chatTemplates.get(contextIdentifier);
    if (contextSpecificTemplate == null) {
      log.error("[focusDialogueChat] could not find chat templates matching supplied context identifier {}", contextIdentifier);
      return null;
    }
    Chat chat;
    Utterance originalUtterance = getOriginalContextSpecificUtterance(user, agentTeam, contextIdentifier);
    if (originalUtterance == null) {
      log.error("[focusDialogueChat] could not find utterance triggering call for context identifier {}", contextIdentifier);
      return null;
    }

    //TODO: Update what actually happens here. Conceptually we should wait until the user is done with their last
    //  interaction or proactively interrupt it in order to engage in this. Issue is it is hard to detect when an
    //  interaction is "done" purely through language, and we probably want to override some interactions and not
    //  others. Currently interrupting everything that is not in a validation state after the current input is processed
    while (true) {
      // Grab current chat state
      synchronized (currentChatCollections) {
        speaker = getUntypedSymbol(user);
        addressee = getUntypedSymbol(agentTeam);
        collection = getChatCollection(speaker, addressee);
        if (collection == null) {
          log.error("[runContextSpecificChat] supplied user/agent pair {}/{} does not have an existing Chat collection", speaker, addressee);
          return null;
        }
      }

      // Wait for current pending input(s) to be fully processed and then grab input lock until this call's side effect terminates.
      //  Will be unlocked in dialogue history update resulting from "focusedContextResponseSubmissionWrapper" goal submitted in outer script.
      collection.grabInputLock(originalUtterance);

      // After input is processed, check if we are in validation state. If so, loop to top and wait for it to terminate.
      // Otherwise, overwrite and remove the existing chat with the focused context.
      synchronized (currentChatCollections) {
        if (collection.currentChat != null && collection.currentChat.chatState == ChatState.VALIDATION) {
          // In validation state, release hold on input lock and loop to top
          log.warn("[focusDialogueChat] existing chat context is in validation, allowing it to terminate before entering focused context");
          collection.signalInputProcessed(originalUtterance);
          chatContext = collection.currentChat;
        } else {
          // Set focused context to current
          log.warn("[focusDialogueChat] discarding existing chat context. Should only be system responses / failure messages");

          // Grab corresponding chat history which was cached in pending contexts
          //TODO: This should theoretically be a list per contextIdentifier; probably shouldn't, but could have multiple
          //      suspended conversations of the same type at the same time. (could use ActionContext id)
          chatContext = collection.pendingFocusedChats.remove(contextIdentifier);
          if (chatContext == null) {
            log.error("[runContextSpecificChat] could not find stored chat state for user/agent pair {}/{}", speaker, addressee);
            collection.signalInputProcessed(originalUtterance);
            return null;
          }

          // Remove previous tool calls, replace system prompt, add previous Chat history to new system prompt
          chatContext.chatState = ChatState.PARSE;
          collection.signalInputProcessed();
          chatContext.contextID = contextIdentifier;
          collection.currentChat = chatContext;
          chat = chatContext.history;
          // Found issues with changing prompt but maintaining history in Message list - LLM interprets it as future input
          //   regardless of explanations in system prompt and will attempt to call tools based on that history. Including
          //  in system prompt itself and resetting message list
          String chatHistoryAsString = getChatHistoryAsString(chat);
          //promptVars.put("%focusedChatHistoryAsString", chatHistoryAsString);
          chat.setMessages(new ArrayList<>());
          // Qwen3.5 chat template requires a user message
          String templateArgumentName = "target template";
          Tool focusedTextResponseTool = contextSpecificTemplate.tools.get(textResponseTool.toolDefinition.function.name);
          if (focusedTextResponseTool != null) {
            //TODO: this isn't the most principled way to get this value
            Set<String> argNames = focusedTextResponseTool.toolDefinition.function.parameters.getProperties().keySet();
            argNames.remove("response");
            if (!argNames.isEmpty()) {
              templateArgumentName = argNames.iterator().next();
            }
          }
          chat.addUserMessage("The previous conversation history leading up to this point is as follows:\n" + chatHistoryAsString + "\nExtract relevant information from this conversation history to populate corresponding details of the \"" + templateArgumentName + "\" argument of your initial " + textResponseTool.toolDefinition.function.name + " reply. Then generate the response accordingly");
          chat.setSystem(contextSpecificTemplate.systemMessageTemplate.apply(promptVars));

          // Update chat status
          chatContext.chatType = ChatType.FOCUSED;
          // Check time elapsed since storage, if greater than N, add architecture message
          long timeSinceSubmission = (System.currentTimeMillis() - chatContext.submissionTime) / 1000;
          //TODO: what amount of time here?
          if (timeSinceSubmission > 15) {
            chatContext.history.addMessage("tool", String.format("%d seconds have passed since the user gave you the previous instruction. In your next response, unless you are calling a function, please briefly remind the user that they previously instructed you to perform the current task.", timeSinceSubmission));
          }
          chatContext.cid = cid;

          //TODO: can likely refactor code to prevent this and parseUtteranceLLMService from having the same control flow duplicated
          Utterance output;
          int numRetries = 0;
          ToolCall lastToolCall = null;
          StringBuilder errorResponseBuilder = new StringBuilder();
          String tool_call_id;
          while (numRetries < 3) {
            if (!errorResponseBuilder.isEmpty()) {
              if (lastToolCall != null && lastToolCall.function.name.equals(textResponseTool.toolDefinition.function.name)) {
                // Need to add errant tool call so that added error message actually references something?
                chatContext.history.addRobotMessage("", List.of(lastToolCall));
              }
              String errorString = errorResponseBuilder.toString();
              log.info("[runCompletionRetryLoop] running extra chatCompletion request with additional error message: {}", errorString);
              tool_call_id = lastToolCall == null ? "" : lastToolCall.id;
              chatContext.history.addToolMessage(errorString, tool_call_id);
            }
            errorResponseBuilder = new StringBuilder();
            // Call chat completion again with updated prompt and collection state - return Utterance with updated semantics
            Completion response = callChatCompletion(chat, contextSpecificTemplate.toolDefinitions);

            if (response == null) {
              log.error("[focusDialogueChat] received null Completion response");
              Utterance failureUtterance = (new Utterance.Builder()).setSpeaker(user).setAddressee(agentTeam).build();
              handleErrorResponse(failureUtterance, true, false);
              return failureUtterance;
            } else {
              resetErrorCount(false, user, agentTeam);
            }

            if (response.toolCalls.isEmpty()) {
              // Text response, presumably querying for information towards new template target
              lastToolCall = null;
              output = handleTextResponse(response, originalUtterance, collection.currentChat, errorResponseBuilder);
            } else {
              // Direct tool call on transfer
              lastToolCall = response.toolCalls.get(0);
              output = handleToolResponse(response, originalUtterance, collection, errorResponseBuilder);
              //(Old note - not sure if still valid) if this call returns the utterance unmodified, needs to be an error case
            }

            if (output != null) {
              // Need to store utterance whose semantics we need to update in order for proper side effect to occur in the NLU pipeline
              // Reset validated state - don't want to carry over from originalUtterance
              output.setIsValidated(false);
              contextTransferResponseUtterances.put(cid, output);
              return output;
            }
            numRetries += 1;
          }
          handleErrorResponse(originalUtterance, true, true);
          originalUtterance.setType(UtteranceType.UNKNOWN); //Revisit whole commit - decide on error semantics or figure out better way to handle this
          return originalUtterance;
        }
      }
      chatContext.waitForCompletion();
    }
  }

  /**
   * Interruption method for {@link #waitOnFocusedChat(Symbol, Symbol, long)}
   * Unlocks current chatContext for user/agentTeam pair if matching supplied Context ID
   */
  //TODO: should this method be modified to be specific just to a corresponding tsc waiting on the chat, rather than
  // unlocking the entire chat?
  @TRADEService
  public boolean interruptWaitOnFocusedChat(Symbol user, Symbol agentTeam, long cid) {
    log.info("[interruptWaitOnFocusedChat] in method {}/{} {}", user, agentTeam, cid);

    Symbol speaker = getUntypedSymbol(user);
    Symbol addressee = getUntypedSymbol(agentTeam);
    ChatContext chatContext;
    synchronized (currentChatCollections) {
      ChatCollection collection = getChatCollection(speaker, addressee);
      if (collection == null) {
        log.error("[interruptWaitOnFocusedChat] supplied user/agent pair {}/{} does not have an existing Chat history", speaker, addressee);
        return false;
      } else if (collection.currentChat == null || collection.currentChat.cid != cid) {
        log.warn("[interruptWaitOnFocusedChat] no matching chat for pair and id {}/{} {}", speaker, addressee, collection);
        return false;
      }
      chatContext = collection.currentChat;
    }
    chatContext.signalCompletion();

    return true;
  }

  /**
   * Method to wait for termination of a focused ChatContext; currently only called from runContextSpecificChat in nlu.asl.
   * @param user external interactor who is a part of this conversation
   * @param agentTeam DIARC AgentTeam which is a part of this conversation
   * @param cid unique identifier for this focused chat context ("context ID" as in DIARC Context class ID)
   */
  @OnInterrupt(onCancelServiceCall = "interruptWaitOnFocusedChat(?user,?agentTeam,?cid)", onSuspendServiceCall = "interruptWaitOnFocusedChat(?user,?agentTeam,?cid)")
  @Action
  @TRADEService
  public void waitOnFocusedChat(Symbol user, Symbol agentTeam, long cid) {
    log.info("[waitOnFocusedChat] in method {}/{} {}", user, agentTeam, cid);

    // Get Chat for supplied user/agentTeam pair
    Symbol speaker = getUntypedSymbol(user);
    Symbol addressee = getUntypedSymbol(agentTeam);
    ChatCollection collection;
    ChatContext chatContext;
    synchronized (currentChatCollections) {
      collection = getChatCollection(speaker, addressee);
      if (collection == null) {
        log.error("[waitOnFocusedChat] supplied user/agent pair {}/{} does not have an existing Chat history", speaker, addressee);
        return;
      } else if (collection.currentChat == null || collection.currentChat.cid != cid) {
        log.warn("[waitOnFocusedChat] no matching chat for pair and id {}/{} {}. This should only be possible if the initial instruction contained all relevant information", speaker, addressee, collection);
        return;
      }
      chatContext = collection.currentChat;
    }

    // Wait for focused context to terminate
    //Predicate resultOfFocusedContext = chatContext.waitForCompletion();
    chatContext.waitForCompletion();

    Utterance utt = contextTransferResponseUtterances.remove(cid);

    log.info("[waitOnFocusedChat] out method {}/{} {}", user, agentTeam, cid);
  }

  /**
   * Utility to get second LLM response on transfer to a focused context (see comments above
   * contextTransferResponseUtterance for an explanation)
   */
  @TRADEService
  public Utterance getContextTransferResponseUtterance(long cid) {
    Utterance out = contextTransferResponseUtterances.get(cid);
    if (out == null) {
      log.error("[getContextTransferResponseUtterance] no entry found for cid {}", cid);
    }
    return out;
  }

  /**
   * Retrieve the utterance which originally targeted the tool instructing the action causing the transfer to a focused
   * context. Overwriting that utterance's semantics above in order to be able to perform the secondary side effect
   * of the input under the focused context
   */
  //TODO: does this need to check for resume?
  //TODO: Revisit impl of this - should this be using conversation history or ChatContext? Also, do we now just
  //        explicitly store something holding this information?
  private Utterance getOriginalContextSpecificUtterance(Symbol user, Symbol agentTeam, String contextIdentifier) {
    user = getUntypedSymbol(user);
    agentTeam = getUntypedSymbol(agentTeam);
    List<Utterance> conversationHistory = getConversationHistory(user, agentTeam);
    if (!conversationHistory.isEmpty()) {
      for (int i=conversationHistory.size()-1;i>=0;i--) {
        Utterance u = conversationHistory.get(i);
        if (u != null && u.getType() == UtteranceType.INSTRUCT && u.getSemantics() != null && u.getSemantics().isPredicate()) {
          Predicate semantics = (Predicate) u.getSemantics();
          Symbol instructedAction = semantics.get(1);
          if (instructedAction != null && instructedAction.getName().equals(contextIdentifier)) {
            return u;
          }
        }
      }
    }

    return null;
  }

  //// General Completion / ChatCollection Utilities ////

  private Completion callChatCompletion(Chat chat, List<ToolDefinition> tools) {
    ExecutorService endpointCallExecutor = Executors.newSingleThreadExecutor();
    Future<Completion> future = endpointCallExecutor.submit(() -> {
      try {
        //TODO: Add this generally to pipeline
        if (tools != null) {
          return TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Chat.class, List.class)).call(Completion.class, chat, tools);
        } else {
          return TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Chat.class)).call(Completion.class, chat);
        }
      } catch (TRADEException e) {
        log.error("[llmParserService] Error performing chat completion", e);
        return null;
      }
    });
    try {
      // What amount of time here?
      return future.get(10, TimeUnit.SECONDS);
    } catch (Exception e) {
      log.error("[callChatCompletion] exception or timeout waiting on endpoint completion request", e);
      return null;
    }
  }

  @TRADEService
  public void streamingChunkHandler(OpenaiChatCompletionResponse lastChunk, OpenaiChatCompletionResponse currentResponse) {
    log.info("[streamingChunkHandler] in method");
  }

  private Completion callChatCompletionStreaming(Chat chat, List<ToolDefinition> tools) {
    TRADEServiceInfo tsi = getMyService("streamingChunkHandler", OpenaiChatCompletionResponse.class, OpenaiChatCompletionResponse.class);
    ExecutorService endpointCallExecutor = Executors.newSingleThreadExecutor();
    Future<Completion> future = endpointCallExecutor.submit(() -> {
      try {
        return TRADE.getAvailableService(new TRADEServiceConstraints().name("streamingChatCompletion").argTypes(Chat.class, List.class, TRADEServiceInfo.class)).call(Completion.class, chat, tools, tsi);
      } catch (TRADEException e) {
        log.error("[callChatCompletionStreaming] Error performing chat completion", e);
        return null;
      }
    });
    try {
      // What amount of time here?
      return future.get(10, TimeUnit.SECONDS);
    } catch (Exception e) {
      log.error("[callChatCompletionStreaming] exception or timeout waiting on endpoint completion request", e);
      return null;
    }
  }

  /**
   * Method to discard the current chat context. Currently only used when validation questions expire (should be used
   * for general nonactivity timeout as well)
   */
  @TRADEService
  public void endChatContext(Symbol user, Symbol agentTeam) {
    synchronized (currentChatCollections) {
      ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
      if (chatCollection != null) {
        if (chatCollection.currentChat != null && chatCollection.currentChat.chatState == ChatState.VALIDATION) {
          log.warn("[endChatContext] called during validation, not currently equipped to cancel validation question and clean up side effects. Just clearing history...");
          chatCollection.currentChat.history = new Chat();
        } else {
          chatCollection.releaseLocks();
          // This method is only currently called during error cases - make sure all relevant error messages make it
          //  into the appropriate Chat stored in the extended history
          saveContextHistory(user, agentTeam, chatCollection.currentChat);
          chatCollection.currentChat = null;
        }
      }
    }
  }

  // Need to be careful about when to copy vs pass by ref. (overall synchronization)
  public ChatCollection getChatCollection(Symbol user, Symbol agentTeam) {
    if (user == null || agentTeam == null) {
      log.error("[getChatCollection] supplied user/agent pair contains null value {}/{}", user, agentTeam);
      return null;
    }
    ChatCollection collection = currentChatCollections.get(Pair.of(getUntypedSymbol(user), getUntypedSymbol(agentTeam)));
    if (collection == null) {
      log.error("[getChatCollection] no chat collection found for {}/{} pair", user, agentTeam);
    }
    return collection;
  }

  @TRADEService
  public String callChatCompletionForTextResponse(Symbol user, Symbol agentTeam, long id) {
    // Get chat context
    ChatCollection collection;
    synchronized (currentChatCollections) {
      Symbol speaker = getUntypedSymbol(user);
      Symbol addressee = getUntypedSymbol(agentTeam);
      collection = getChatCollection(speaker, addressee);
      if (collection == null) {
        log.error("[callChatCompletionForTextResponse] supplied user/agent pair {}/{} does not have an existing Chat history", speaker, addressee);
        return "";
      } else if (collection.currentChat == null || collection.currentChat.cid != id) {
        log.warn("[callChatCompletionForTextResponse] no current chat for pair with id {}/{} {}", speaker, addressee, id);
        return "";
      }

      Utterance dummy = new Utterance.Builder().setSpeaker(user).setAddressee(agentTeam).build();
      Utterance out = runCompletionRetryLoop(null, List.of(textResponseTool.toolDefinition), dummy, collection, new StringBuilder());
      if (out.getSemantics() != null && out.getSemantics().getName().equals("generateResponseFromString") && out.getSemantics().isPredicate() && ((Predicate)out.getSemantics()).getArgs().size() > 2) {
        return ((Predicate)out.getSemantics()).get(2).toString();
      } else {
        log.error("[callChatCompletionForTextResponse] got unexpected output {}", out);
        return "";
      }
    }
  }

  //TODO: Should ITK calls be fully synchronized between instruction to this call here? Ideally, yes, but it would be
  //  very very hard to detect any exception in the external itk method which would cause this method to not be
  //  reached. For now, if a user submits another input which terminates the chat context before this method has a
  //  chance to be called, we just discard the response.
  /**
   * Method used by "social LLM" itk behaviors (llm <-> diarc itk) to add DIARC's response to an ITK request and then
   * call the Completion endpoint with the updated Chat for a text response.
   */
  @TRADEService
  public String addToolResponse(Symbol user, Symbol agentTeam, String toolResponse, String toolCallName) {
    Symbol speaker = getUntypedSymbol(user);
    Symbol addressee = getUntypedSymbol(agentTeam);
    ChatCollection collection;
    ChatContext chatContext;
    synchronized (currentChatCollections) {
      collection = getChatCollection(speaker, addressee);
      if (collection == null) {
        log.error("[callChatCompletionForTextResponse] supplied user/agent pair {}/{} does not have an existing Chat history", speaker, addressee);
        return "";
      } else if (collection.currentChat == null) {
        log.warn("[callChatCompletionForTextResponse] no current chat for pair with id {}/{}", speaker, addressee);
        return "";
      } else {
        chatContext = collection.currentChat;
        int size = chatContext.history.messages.size();
        String toolCallId = "";
        for (int i=size-1;i>0;i--) {
          Message msg = chatContext.history.messages.get(i);
          if (msg.tool_calls != null && msg.tool_calls.length>0) {
            if (msg.tool_calls[0].function.name.equals(toolCallName)) {
              toolCallId = msg.tool_calls[0].id;
              break;
            }
          }
        }
        if (toolCallId.isEmpty()) {
          log.warn("[addToolResponse] couldn't find matching original tool call {} prompting response {}", toolCallName, toolResponse);
          chatContext.history.addMessage("tool", toolResponse);
        } else {
          chatContext.history.addToolMessage(toolResponse, toolCallId);
        }
      }

      Utterance dummy = new Utterance.Builder().setSpeaker(user).setAddressee(agentTeam).build();
      Utterance out = runCompletionRetryLoop(null, List.of(textResponseTool.toolDefinition), dummy, collection, new StringBuilder());
      if (out.getSemantics() != null && out.getSemantics().getName().equals("generateResponseFromString") && out.getSemantics().isPredicate() && ((Predicate)out.getSemantics()).getArgs().size() > 2) {
        return ((Predicate)out.getSemantics()).get(2).toString();
      } else {
        log.error("[callChatCompletionForTextResponse] got unexpected output {}", out);
        return "";
      }
    }
  }

  //// Inner suspend/resume methods for runContextSpecificChat ////
  @TRADEService
  public Justification suspendLLMChatContext(Symbol user, Symbol agentTeam, long cid) {
    synchronized (currentChatCollections) {
      ChatCollection collection = getChatCollection(user, agentTeam);
      if (collection == null) {
        return new ConditionJustification(false);
      }
      ChatContext chatContext = collection.currentChat;
      if (chatContext == null) {
        log.error("[suspendLLMChatContext] {}/{} chat collection has no active chat", user, agentTeam);
        return new ConditionJustification(false);
      }
      if (chatContext.cid != cid) {
        log.error("[suspendLLMChatContext] active chat for {}/{} does match supplied cid. {}, {}", user, agentTeam, chatContext.cid, cid);
        return new ConditionJustification(false);
      }
      if (chatContext.chatType != ChatType.FOCUSED) {
        log.error("[suspendLLMChatContext] active chat for {}/{} does not have ChatType FOCUSED, has {}", user, agentTeam, chatContext.chatType);
        return new ConditionJustification(false);
      }

      collection.suspendedChats.add(0, chatContext);
      collection.currentChat = null;
    }

    return new ConditionJustification(true);
  }

  @TRADEService
  public Justification resumeLLMChatContext(Symbol user, Symbol agentTeam, long cid) {
    synchronized (currentChatCollections) {
      ChatCollection collection = getChatCollection(user, agentTeam);
      if (collection == null) {
        return new ConditionJustification(false);
      }
      if (collection.currentChat != null) {
        log.error("[resumeLLMChatContext] active chat for {}/{} already exists.", user, agentTeam);
        return new ConditionJustification(false);
      }

      int index = -1;
      for (int i = 0; i < collection.suspendedChats.size(); i++) {
        if (collection.suspendedChats.get(i).cid == cid) {
          index = i;
          break;
        }
      }

      if (index == -1) {
        log.error("[resumeLLMChatContext] Could not find suspended chat for pair {}/{} and cid {}.", user, agentTeam, cid);
        return new ConditionJustification(false);
      }

      //TODO: this message needs to be tuned
      ChatContext chatContext = collection.suspendedChats.remove(index);
      //TODO: relate this response id to the suspend tool call?
      chatContext.history.addMessage("tool", "Your interaction with the user was paused for some amount of time while you both performed other actions. It has now been resumed by the system. Please refamiliarize the user with the task you are performing and the current state of progress.");
      boolean userMessageRequired = true;
      for (Message message : chatContext.history.messages) {
        if (message.role.equals("user")) {
          userMessageRequired = false;
          break;
        }
      }
      if (userMessageRequired) {
        // Think it may be theoretically possible, but don't think it should usually be case that you won't have a user
        //    message at this point.
        // Qwen3.5 chat template requires a user message
        chatContext.history.addUserMessage("follow the previous instructions");
      }
      collection.currentChat = chatContext;
    }

    return new ConditionJustification(true);
  }

  @TRADEService
  public Justification cancelLLMChatContext(Symbol user, Symbol agentTeam, long cid) {
    synchronized (currentChatCollections) {
      ChatCollection collection = getChatCollection(user, agentTeam);
      if (collection == null) {
        return new ConditionJustification(false);
      }

      if (collection.currentChat != null && collection.currentChat.cid == cid) {
        ChatContext chatContext = collection.currentChat;
        if (chatContext.chatType != ChatType.FOCUSED) {
          log.error("[cancelLLMChatContext] active chat for {}/{} does not have ChatType FOCUSED, has {}", user, agentTeam, collection.currentChat.chatType);
          return new ConditionJustification(false);
        }
        //TODO: add explicit message indicating chat was canceled? Or indication should probably already exist
        saveContextHistory(user, agentTeam, collection.currentChat);
        collection.currentChat = null;
      } else {
        int index = -1;
        for (int i = 0; i < collection.suspendedChats.size(); i++) {
          if (collection.suspendedChats.get(i).cid == cid) {
            index = i;
            break;
          }
        }
        if (index == -1) {
          log.error("[cancelLLMChatContext] could not find matching chat for pair and id {}/{}, {}", user, agentTeam, cid);
          return new ConditionJustification(false);
        } else {
          ChatContext chatContext = collection.suspendedChats.remove(index);
        }
      }
    }

    return new ConditionJustification(true);
  }

  //// General Utilities ////

  //TODO: This form of storage/retrieval only matches on name, should match on signature form (without specific variable name choices)
  private List<ToolDefinition> getQuestionResponseToolDefinitions(Symbol agent) {
    List<ToolDefinition> responseFormTools = new ArrayList<>();
    try {
      List<Term> expectedResponseForms = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestionResponseForm").argTypes(Symbol.class)).call(List.class,agent);
      if (expectedResponseForms != null) {
        for (Term responseForm : expectedResponseForms) {
          Tool tool = replyToolset.get(responseForm.getName());
          if (tool != null) {
            responseFormTools.add(tool.toolDefinition);
          } else {
            log.error("[getQuestionResponseToolDefinitions] found no corresponding ToolDefinition for expected response form {}", responseForm);
            //TODO: create default definition from semantics
          }
        }
      }
    } catch (TRADEException e) {
      log.error("[getCurrentQuestionsString] error getting last question for agent");
    }
    if (!responseFormTools.isEmpty()) {
      responseFormTools.add(replyToolset.get("abort").toolDefinition);
    }
    return responseFormTools;
  }

  //TODO: Extend to include pending and scheduled goals + all current goals - not just most recent
  protected List<Predicate> getCurrentGoals(Symbol agent) {
    try {
      List<Predicate> currentGoalPredicate = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSystemGoalsPredicates").argTypes(Symbol.class)).call(List.class,agent);
      return currentGoalPredicate;
    } catch (TRADEException e) {
      log.error("[getCurrentGoals] error getting current goal for agent", e);
    }
    return new ArrayList<>();
  }

  protected List<Predicate> getPendingGoals(Symbol agent) {
    try {
      List<Predicate> pendingGoalPredicate = TRADE.getAvailableService(new TRADEServiceConstraints().name("getPendingGoals").argTypes(Symbol.class)).call(List.class,agent);
      return pendingGoalPredicate;
    } catch (TRADEException e) {
      log.error("[getPendingGoals] error getting pending goals for agent", e);
    }
    return new ArrayList<>();
  }

  protected List<Predicate> getScheduledGoals(Symbol agent) {
    try {
      List<Predicate> scheduledGoals = TRADE.getAvailableService(new TRADEServiceConstraints().name("getScheduledGoals").argTypes(Symbol.class)).call(List.class,agent);
      return scheduledGoals;
    } catch (TRADEException e) {
      log.error("[getScheduledGoals] error getting scheduled goals for agent", e);
    }
    return new ArrayList<>();
  }

  protected String getCurrentGoalsSummary(Symbol agent) {
    List<Predicate> currentGoalPredicate = getCurrentGoals(agent);
    if (!currentGoalPredicate.isEmpty()) {
      return currentGoalPredicate.get(0).toString() + "\n";
    } else {
      return "<The robot has no current task>\n";
    }
  }

  ////TODO: don't want to generate each time?
  @TRADEService
  public String getFullGoalsSummary(Symbol agent) {
    List<Predicate> currentGoalPredicates = getCurrentGoals(agent);
    List<Predicate> pendingGoalPredicates = getPendingGoals(agent);
    List<Predicate> scheduledGoals = getScheduledGoals(agent);
    //TODO: call endpoint to transform all of the predicates to String descs
    int numCurrent = currentGoalPredicates.size();
    numCurrent = numCurrent == 0 ? 0 : 1; //TODO: handle multiple current if not using queue
    int numPending = pendingGoalPredicates.size();
    int numScheduled = scheduledGoals.size();
    List<String> translatedGoalList = new ArrayList<>();
    if (numCurrent + numPending + numScheduled != 0) {
      Chat goalGenChat = new Chat(goalTranslationPrompt.apply(promptVars));
      StringBuilder goalListString = new StringBuilder();

      if (numCurrent != 0) {
        String goalString;
        if (currentGoalPredicates.get(0).hasArgs()) {
          goalString = currentGoalPredicates.get(0).get(0).isPredicate() ?
                  replaceReferencesInSemantics((Predicate) currentGoalPredicates.get(0).get(0)).toString() :
                  currentGoalPredicates.get(0).get(0).toString();
        } else {
          goalString = replaceReferencesInSemantics(currentGoalPredicates.get(0)).toString();
        }
        goalListString.append("\nInput 1: ").append(goalString);
      }

      if (numPending != 0) {
        for (int i = 0; i < numPending; i++) {
          goalListString.append("\nInput ").append(i+2).append(": ").append(replaceReferencesInSemantics(pendingGoalPredicates.get(i)));
        }
      }

      if (numScheduled != 0) {
        int buffer = numCurrent == 0 ? 1 : 2 + numPending;
        for (int i = 0; i < numScheduled; i++) {
          goalListString.append("\nInput ").append(i+buffer).append(": ").append(replaceReferencesInSemantics(scheduledGoals.get(i)));
        }
      }

      //TODO: revisit this prompt - output isn't exactly correctly phrased
      goalGenChat.addUserMessage(goalListString.toString());
      Completion response = callChatEndpoint(goalGenChat);
      if (response == null) {
        log.error("[getFullGoalsSummary] received null completion response");
        return "There was an error with losing connection while making this request. Please try to make this call again.";
      }

      String responseString = response.getText().trim();
      Pattern outputPattern = Pattern.compile(".*?\"?translation\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*?", Pattern.DOTALL);
      Matcher m = outputPattern.matcher(responseString);
      while (m.find()) {
        translatedGoalList.add(m.group(1));
      }
      if (translatedGoalList.isEmpty()) {
        //TODO: better recovery
        log.error("[getFullGoalsSummary] unable to parse completion response");
        return "There was an error retrieving your task list";
      } else if (translatedGoalList.size() != numPending + numScheduled + numCurrent) {
        //TODO: better recovery
        log.error("[getFullGoalsSummary] unexpected number of goal translations parsed from completion response");
        return "There was an error retrieving your task list";
      }
    }

    StringBuilder stringBuilder = new StringBuilder("At any point in time, you may have been instructed to perform various" +
            " tasks by different users, but cannot do everything all at once. As a result, you keep track of the list of" +
            " things you need to do and order them according to a priority value. By default, this is the order in which the tasks" +
            " are submitted. You do not currently have the capability to change priority of submitted tasks, but will be able" +
            " to soon in a future update. Because you cannot perform multiple tasks at once, there will be one task which" +
            " you are actively focused on. This task can be referred to as the \"active task\", \"suspended task\", \"current task\"," +
            " or simply \"what you are currently doing\". As stated, this task can be temporarily \"suspended\". In this state, that task" +
            " will remain as your current focus, and you will wait until it is resumed or canceled to perform any other tasks, even if" +
            " others are present. The rest of the list of things you need to do can be referred to as your \"pending tasks\", " +
            "\"upcoming tasks\", or simply \"what you still need to do\" after your current task. There is no limit to the amount of" +
            " pending tasks you can have at once. Finally, there are also \"scheduled tasks\". These are tasks which are scheduled" +
            " to occur at some specific point in the future, optionally on a repeating interval, but are not currently on" +
            " the active or pending list of tasks to perform. When the scheduled time arrives, the associated task will be added to the" +
            " list of things you must do.\n-------\n");

    stringBuilder.append("Your \"current task\"/\"active task\"/\"what you are currently doing\" is:\n");
    if (!currentGoalPredicates.isEmpty()) {
        if (currentGoalPredicates.get(0).hasArgs()) {
          String statusString = currentGoalPredicates.get(0).getName();
          stringBuilder.append(" - Status: ").append(statusString).append(", Description: ").append(translatedGoalList.get(0)).append("\n");
        } else {
          stringBuilder.append("- Description: ").append(translatedGoalList.get(0)).append("\n");
        }
    } else {
      stringBuilder.append(" - <You have no current task>\n");
    }

    stringBuilder.append("Your list of \"pending tasks\"/\"upcoming tasks\"/\"next tasks\" are:\n");
    if (pendingGoalPredicates.isEmpty()) {
      stringBuilder.append(" - <You have no pending tasks>\n");
    } else {
      for (int i = 0; i < numPending; i++) {
        stringBuilder.append(" - ").append(translatedGoalList.get(i+1)).append("\n");
      }
    }

    stringBuilder.append("Your list of \"scheduled tasks\" are:\n");
    if (scheduledGoals.isEmpty()) {
      stringBuilder.append(" - <You have no scheduled tasks>\n");
    } else {
      for (int i = 0; i < numScheduled; i++) {
        stringBuilder.append(" - ").append(translatedGoalList.get(i+numCurrent+numPending)).append("\n");
      }
    }
    //Doesn't seem to do anything - may need to just split calls for retrieving current vs pending vs scheduled vs all tasks
    stringBuilder.append("-------\nOnly respond to the user with information relevant to what they actually asked about " +
            "which caused you to trigger the getCurrentPendingScheduledGoalInformation tool call. For example: asking what" +
            "you are doing should cause you to only say what your current task is, being asked how many scheduled goals you have" +
            "should cause you to only answer with the number of scheduled goals, and being asked what you have to do after your" +
            "current task should cause you to only begin listing your pending tasks. Do not list more than 5 tasks at once" +
            "without being asked to list more.");
    return stringBuilder.toString();
  }

  private void saveContextHistory(Symbol user, Symbol agentTeam, ChatContext chatContext) {
    List<ChatContext> extendedHistory = chatContextHistories.get(Pair.of(user.toUntyped(),agentTeam.toUntyped()));
    if (extendedHistory == null) {
      extendedHistory = new ArrayList<>();
      chatContextHistories.put(Pair.of(user.toUntyped(),agentTeam.toUntyped()),extendedHistory);
    } else if (extendedHistory.size() > 20) {
      //What should cap size be ^? Should this be different than what is retrieved by itk? (first case where the answer
      //  may be yes is in order to retrieve full history for tests)
      extendedHistory.remove(extendedHistory.size()-1);
    }

    extendedHistory.add(0,chatContext);
  }

  public List<ChatContext> getExtendedConversationHistories(Symbol user, Symbol agentTeam) {
    return chatContextHistories.getOrDefault(Pair.of(user.toUntyped(),agentTeam.toUntyped()), new ArrayList<>());
  }

  @TRADEService
  public String getExtendedConversationHistoryString(Symbol user, Symbol agentTeam) {
    List<ChatContext> extendedHistory = chatContextHistories.get(Pair.of(user.toUntyped(),agentTeam.toUntyped()));

    StringBuilder extendedHistoryString = new StringBuilder("The extended interaction history between you and this user is copied below:\n#\n");

    if (extendedHistory == null) {
      extendedHistoryString.append("<There are no stored past histories found>\n\n");
    } else {
      // What should cap size be? Should we allow iteratively longer history retrieval?
      int numHistories = Math.min(extendedHistory.size(), 5);
      for (int i=0; i<numHistories;i++) {
        ChatContext cc = extendedHistory.get(i);
        extendedHistoryString.append("Past conversation context # ").append(i).append(":\n");
        extendedHistoryString.append(getChatHistoryAsString(cc.history)).append("\n\n");
      }
      ChatCollection collection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
      if (collection != null && !collection.suspendedChats.isEmpty()) {
        extendedHistoryString.append("You also have some interrupted conversations which you may still eventually return to and complete. They are as follows:\n");
        List<ChatContext> suspendedContexts = collection.suspendedChats;
        for (int i=0; i<suspendedContexts.size();i++) {
          ChatContext cc = suspendedContexts.get(i);
          extendedHistoryString.append("Suspended conversation context # ").append(i).append(":\n");
          extendedHistoryString.append(getChatHistoryAsString(cc.history)).append("\n\n");
        }
      }
    }

    extendedHistoryString.append("\n#\nPlease use this information in order to help fulfill the user's request, whether it is to answer " +
            "a question directly about what you have discussed in the past or to simply understand something from the past" +
            "that the user is referring to. Do not relay this information back to the user unless they have directly asked about it, in" +
            "which case you may summarize relevant conversations. If the user has asked about something which you cannot find" +
            "in either these histories or your current one, you must tell the user that you are sorry and cannot remember" +
            "what they are referring to.");
    return extendedHistoryString.toString();
  }

  //TODO: does this need to be extended to agent pairs?
  protected String getCurrentQuestionsString(Symbol agent) {
    String output = "";
    try {
      String questionString = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestion").argTypes(Symbol.class)).call(String.class,agent);
      List<Term> expectedResponseForms = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestionResponseForm").argTypes(Symbol.class)).call(List.class,agent);
      if (expectedResponseForms != null && !expectedResponseForms.isEmpty()) {
        output = "Through previous interaction between the user and the back end system, the system has came up with its own question(s) for the user. The question(s) and their corresponding expected reply forms are presented below.";
        output += questionString + "\nEXPECTED REPLY FORM:\n" + String.join("\n",expectedResponseForms.stream().map(Term::toString).toList());
        output += "\nThere should be function entries with name and arguments matching each of the expected reply forms. It is very likely that the most recent user input should be interpreted as a response to this question(s) as opposed to a separate instruction in the case where it may reasonably be interpreted as either.";
      }
    } catch (TRADEException e) {
      log.error("[getCurrentQuestionsString] error getting last question for agent");
    }
    return output;
  }

  private String getConversationHistoryString(List<Utterance> conversationHistory) {
    StringBuilder builder = new StringBuilder();
    if (conversationHistory.isEmpty()) {
      builder.append("<there is no conversation history>");
    } else {
      for (Utterance pastUtterance : conversationHistory) {
        builder.append(pastUtterance.getSpeaker()).append(": ").append(pastUtterance.getWordsAsString()).append("\n");
      }
    }
    return builder.toString();
  }

  //TODO: all taught action support
  @Override
  protected void initLearnedActions() {
    super.initLearnedActions();
    synchronized (currentChatCollections) {
      for (ActionDBEntry entry : learnedActions) {
        addLearnedActionToCollections(entry);
      }
    }
  }

  @Override
  public void handleDatabaseAddition(ActionDBEntry entry) throws TRADEException {
    super.handleDatabaseAddition(entry);
    if (entry.isScript() && entry.getDBFile() == null && !entry.getName().equals("planned")) {
      log.info("[handleDatabaseAddition] have learned action {}", entry.getName());
      addLearnedActionToCollections(entry);
    }
  }

  @Override
  public void handleDatabaseRemoval(ActionDBEntry entry) throws TRADEException {
    super.handleDatabaseRemoval(entry);
    removeLearnedActionFromCollections(entry.getName());

  }

  private void addLearnedActionToCollections(ActionDBEntry dbEntry) {
    Predicate signature = dbEntry.getSignature(false);
    Tool correspondingTool = new Tool(new ToolDefinition(signature.getName(), "This is an action which was previously taught to you by a human user. It was given the name \"" + signature.getName() + "\" during instruction. You may use the getActionSteps function to get a description of the steps contained within this action if desired.",new ToolParameters(new ArrayList<>())),UtteranceType.INSTRUCT);
    String nluString = toolToDescString(correspondingTool, true);
    String nlgString = toolToDescString(correspondingTool, false);

    // If an action with same name already exists, overwrite it and fill in with new details
    //  (TODO: If teaching parameterized/overloaded actions, would need to update representation)
    synchronized (currentChatCollections) {
      if (chatTemplates.get("nlu").tools.get(signature.getName()) != null) {
        log.error("[addLearnedActionToCollections] have multiple taught actions with the same name {}. Overwriting existing entry.", signature.getName());
        removeLearnedActionFromCollections(signature.getName());
      }
      // Add to "instructable actions" list for focused contexts
      promptVars.put("%instructNLGList", promptVars.get("%instructNLGList") + String.format("\n%s\n#", nlgString));
      promptVars.put("%instructList", promptVars.get("%instructList") + String.format("\n%s\n#", nluString));
      // Add as tool target in base context
      chatTemplates.get("nlu").toolDefinitions.add(correspondingTool.toolDefinition);
      chatTemplates.get("nlu").tools.put(signature.getName(), correspondingTool);
    }
  }

  private void removeLearnedActionFromCollections(String name) {
    synchronized (currentChatCollections) {
      Tool correspondingTool = chatTemplates.get("nlu").tools.remove(name);
      if (correspondingTool == null) {
        log.error("[removeLearnedActionFromCollections] removing action not found in base instructable tool set");
        return;
      }
      String nluString = toolToDescString(correspondingTool, true);
      String nlgString = toolToDescString(correspondingTool, false);
      // As of now is the same because all are parameterless
      //String NLGString = toolToDescString(correspondingTool, false);
      //Remove from "instructable actions list" for focused contexts
      promptVars.put("%instructNLGList", promptVars.get("%instructNLGList").replace(String.format("\n%s\n#", nlgString), ""));
      promptVars.put("%instructList", promptVars.get("%instructList").replace(String.format("\n%s\n#", nluString), ""));
      // Remove as tool target in base context
      chatTemplates.get("nlu").toolDefinitions.remove(correspondingTool.toolDefinition);
      chatTemplates.get("nlu").tools.remove(name);
    }
  }

  //TODO: Able to move at least some of this to the base class?
  @TRADEService
  public List<String> getInstructableActions() {
    return new ArrayList<>(chatTemplates.get("nlu").tools.keySet());
  }

  /**
   * Experimenting adding messages to ChatContexts from dialogue history updates (**currently NLG only) in case
   *  output from external parsers or generators is chosen over this one.
   *  (If intending to do the same for NLU, need to first stop adding messages directly in parseUtterance)
   */
  @Override
  public void updateLLMConversationHistory(Utterance utterance, boolean newUtterance) {
    super.updateLLMConversationHistory(utterance, newUtterance);

    if (utterance == null) {
      return;
    }

    synchronized (currentChatCollections) {
      if (utterance.isInputUtterance()) {
        //NLU
         if (utterance.getSemantics() == null || !utterance.getSemantics().isPredicate()) {
           return;
         }
        Symbol agentTeam = utterance.getAddressee().toUntyped();
        Symbol user = utterance.getSpeaker().toUntyped();
        ChatCollection chatCollection = getChatCollection(user, agentTeam);
        Predicate semantics = (Predicate) utterance.getSemantics();

        // Create new collection in case another parser hit completes and triggers a history update
        //  before the input even reaches this parser. In this case, we want to short-circuit the parse call for this
        //  utterance to prevent any unncessary side effects.
        if (chatCollection == null) {
          chatCollection = new ChatCollection();
          currentChatCollections.put(Pair.of(user.toUntyped(), agentTeam.toUntyped()), chatCollection);
        }

        if (chatCollection.currentChat != null) {
          ChatContext chatContext = chatCollection.currentChat;

          // Reset validation Question timeout on any user input
          if (chatContext.chatState == ChatState.VALIDATION) {
            try {
              List<Term> responseForms = List.of(Factory.createPredicate("val(X)"), Factory.createPredicate("validated(X)"));
              TRADE.getAvailableService(new TRADEServiceConstraints().name("resetQuestionTimeout").argTypes(List.class, Symbol.class, Symbol.class)).call(void.class, responseForms, agentTeam, user);
            } catch (TRADEException e) {
              log.error("[updateLLMConversationHistory] unable to reset timeout for validation questions", e);
            }
          }

          // If negative validation response, don't release inputLock until reattemptParse completes
          if ((semantics.getName().equals("val") || semantics.getName().equals("validated")) && semantics.get(0).getName().equals("no")) {
            return;
          }

          // Perform necessary side effects to current chat. 2 cases handled collectively here:
          //   External parser hit for an action instruction - discard chat context holding any side effects from unused LLM parse
          //   LLM noConfirmation case - handle side effects as if a validated instruction just occurred
          //   +1
          if (utterance.getType() == UtteranceType.INSTRUCT && !utterance.needsValidation()) {
            String command = chatContext.lastToolCallName;
            chatContext.lastToolCallName = "";
            if (utterance.getSemantics() != null && utterance.getSemantics().isPredicate()) {
              if (semantics.getArgs().size() > 1) {
                if (command.isEmpty() || semantics.get(1).getName().equals("generateResponseFromString")) {
                  command = semantics.get(1).getName();
                }
              }
            }
            if (!command.equals("generateResponseFromString")) {
              handleValidatedToolCall(command, utterance.getSpeaker(), utterance.getAddressee(), chatCollection);
            }
          }
        }

        // TODO: do we want to extend holding the lock until around tentative accept response or failure response?
        // Side effects complete - Unlock input lock for this user/agent pair
        chatCollection.signalInputProcessed(utterance);
      } else {
        //NLG - add to current context if exists
        Symbol user = utterance.getAddressee().toUntyped();
        Symbol agentTeam = utterance.getSpeaker().toUntyped();
        ChatCollection chatCollection = getChatCollection(user, agentTeam);
        if (chatCollection != null) {
          ChatContext chatContext = chatCollection.currentChat;
          if (chatContext != null) {
            //// Add as assistant message "content" string
            //chatContext.history.addRobotMessage(utterance.getWordsAsString());

            //// Add as assistant message text response tool call
            if (chatContext.chatState == ChatState.VALIDATION) {
              // TODO: keep an eye on this, make sure the LLM doesn't try to target this fabricated tool in the future
              // (Not sure how to best represent this; Balancing act of accurately conveying what is actually happening
              //  vs preventing LLM from targeting tool calls which aren't exposed to it / doing validation on its own)
              ToolCall toolCall = new ToolCall();
              toolCall.id = "";
              toolCall.type = "function";
              ToolFunction toolFunction = new ToolFunction();
              toolFunction.name = "validatingToolSubmission";
              Map<String, Object> responseArguments = new HashMap<>();
              responseArguments.put("response", utterance.getWordsAsString());
              toolFunction.arguments = new Gson().toJson(responseArguments);
              toolCall.function = toolFunction;
              chatContext.history.addRobotMessage("", List.of(toolCall));
            } else {
              // TODO: Keep an eye on this, make sure it's okay that these seemingly unprompted assistant messages are present.
              Tool textResponseTool = chatTemplates.get(chatContext.contextID).tools.get(this.textResponseTool.toolDefinition.function.name);
              if (textResponseTool != null) {
                ToolCall toolCall = new ToolCall();
                toolCall.id = "";
                toolCall.type = "function";
                ToolFunction toolFunction = new ToolFunction();
                toolFunction.name = this.textResponseTool.toolDefinition.function.name;
                Map<String, Object> responseArguments = new LinkedTreeMap<>();
                if (!chatContext.lastTemplateName.isEmpty() && textResponseTool.toolDefinition.function.parameters.getProperties().containsKey(chatContext.lastTemplateName)) {
                  responseArguments.put(chatContext.lastTemplateName, chatContext.lastTemplateValue);
                }
                responseArguments.put("response", utterance.getWordsAsString());
                toolFunction.arguments = new Gson().toJson(responseArguments);
                toolCall.function = toolFunction;
                chatContext.history.addRobotMessage("", List.of(toolCall));
              }
            }
          }
          //TODO: Should this be added to a new history/context if none currently exists? Hard question to answer because
          //  sometimes it may be relevant and sometimes it may throw off future parses significantly. Best case is to
          //  have an "extended dialogue history" itk target which is handled well
        }
      }
    }
  }

  /**
   * Question asking notification callback - throw away the current chat context if the associated system validation
   *  question timed out without a response.
   */
  @TRADEService
  public void questionAskingCallback(Symbol agent, boolean active, Question question) {
    // Only care when question is terminated
    if (active) {
      return;
    }

    // Only care if validation question
    if (!question.hasMatchingResponseForms(List.of(Factory.createPredicate("val(X)"), Factory.createPredicate("validated(X)")))) {
      return;
    }

    Symbol user = question.getAddressee().toUntyped();
    Symbol agentTeam = agent.toUntyped();
    // Validation questions from socialLLMnlu.asl should always have a specific target
    if (user.getName().equals("any")) {
      return;
    }

    if (!question.wasAnswered()) {
      // If validation question expired but chat context is still in validation state, we know the validation question expired.
      //  So update chat state accordingly.
      synchronized (currentChatCollections) {
        ChatCollection chatCollection = getChatCollection(user, agentTeam);
        if (chatCollection != null) {
          ChatContext chatContext = chatCollection.currentChat;
          if (chatContext != null && chatContext.chatState == ChatState.VALIDATION) {
            chatCollection.signalInputProcessed();
            if (chatContext.chatType == ChatType.FOCUSED) {
              chatContext.chatState = ChatState.PARSE;
              chatContext.history.addToolMessage("Your last validation question was not answered after a long duration and timed out from the perspective of the system. You may need to remind the user of your task if they seem unaware of your current task of \""+chatContext.contextID+"\" and will need to call your target tool call again if they try answering your question affirmatively.",chatContext.toolCallIdUpForValidation);
              chatContext.toolCallIdUpForValidation = "";
              chatContext.utteranceUpForValidation = null;
            } else {
              //TODO: add message indicating timeout occurred?
              saveContextHistory(user, agentTeam, chatCollection.currentChat);
              chatCollection.currentChat = null;
            }
          }
        }
      }
    }
  }

  //TODO: should postprocessing errors here increment "internalErrorCount" even if returning their own specialized error message?
  /**
   * Process an LLM Tool call response by matching it against existing Tool Definitions.
   * @param functionCall Completion response Tool call
   * @param tool Matching Tool definition containing intended structure and constraints to validate against
   * @param inputUtterance input Utterance whose semantics to update
   * @param errorMessage Stringbuilder to populate with all relevant errors caught during postprocessing to feed back to the LLM
   * Return:
   *   Fully populated utterance with information extracted from functionCall
   *   Null on any error case - errorMessage should be updated in this case
   */
  private Utterance processToolCall(ToolFunction functionCall, Tool tool, Utterance inputUtterance, StringBuilder errorMessage) {
    Utterance.Builder builder = new Utterance.Builder(inputUtterance);
    Symbol actor = inputUtterance.getAddressee();
    Map<Symbol,Variable> varMap = new HashMap<>();
    Symbol semantics = processToolArgument(functionCall.name, functionCall.arguments, tool, actor, builder, varMap, errorMessage, false);
    if (semantics != null) {
      builder.setSemantics(semantics);
      builder.setUtteranceType(tool.utteranceType);
      return builder.build();
    } else {
      return null;
    }
  }
  // Process an arbitrary inner argument found in a Tool response - convert to a Symbol
  //  List -> Converts each element into a Symbol independently and wraps in an "and" predicate
  //  Map or json string -> Corresponding Tool parameter must have processAsTool flag and associated toolChatTemplateContextId set. Will be processed as a Tool call, converted into the corresponding action predicate
  //  String/Number -> Factory.createFoL(arg)
  private Symbol processArgument(String parameterName, Object parameterValue, ToolParameter parameterDefinition, ToolDefinition parentTool, Symbol actor, Utterance.Builder builder, Map<Symbol,Variable> varMap, StringBuilder errorMessage, boolean unpopulatedArgSafe) {
    Symbol processedArg = null;
    boolean argValid = true;
    unpopulatedArgSafe = unpopulatedArgSafe || parentTool.function.name.equals("getIfCanPerform"); //TODO; revisit this and associated cases
    if (parameterValue instanceof List<?>) {
      //// Handle List
        List<Object> argObjectList = (List<Object>) parameterValue; //Can you actually have a ClassCastException here? Catch and handle if so
        List<Symbol> innerArgs = new ArrayList<>();
        for (Object argValue : argObjectList) {
          Symbol innerArg = processArgument(parameterName, argValue, parameterDefinition, parentTool, actor, builder, varMap, errorMessage, unpopulatedArgSafe);
          if (innerArg == null) {
            argValid = false;
            continue;
          }
          innerArgs.add(innerArg);
        }
        processedArg = Factory.createPredicate("and",innerArgs);
    } else if (parameterValue instanceof Map<?,?>) {
      //// Handle Map (Tool)
      if (!parameterDefinition.getProcessAsTool()) {
        log.error("[processArgument] have argument value json without flag set to process as tool: {}", parameterValue);
        //TODO: is this just developer error or can it actually also be LLM error?
        errorMessage.append("- You supplied a JSON object for the \"").append(parameterName).append("\" argument of the \"").append(parentTool.function.name).append("\" call, but it was not signaled to be parsed as a tool call. Please adapt this argument to a string or number. Do not speak to the user about this.\n");
        return null;
      }
      processedArg = processToolArgument(parameterName, parameterValue, parameterDefinition, parentTool, actor, builder, varMap, errorMessage, unpopulatedArgSafe);
    } else if (parameterValue instanceof String) {
      if (parameterDefinition.getProcessAsTool()) {
        //// Handle Tool Json String
        processedArg = processToolArgument(parameterName, parameterValue, parameterDefinition, parentTool, actor, builder, varMap, errorMessage, unpopulatedArgSafe);
      } else {
        //// Handle General String (Symbol/Predicate)
        String stringValue = (String) parameterValue;
        if (parentTool.function.name.equals(textResponseTool.toolDefinition.function.name) || parentTool.function.name.equals("say")) {
          if (!(stringValue.startsWith("\"") && stringValue.endsWith("\""))) {
            stringValue = "\"" + stringValue + "\"";
          }
        }
        processedArg = processSymbolArgument(parameterName, stringValue, parameterDefinition.getPossible_values(), errorMessage, varMap, builder, false, unpopulatedArgSafe);
      }
    } else {
      //// Handle Number - anything else possible?
      String stringValue = String.valueOf(parameterValue);
      processedArg = processSymbolArgument(parameterName, stringValue, parameterDefinition.getPossible_values(), errorMessage, varMap, builder, false, unpopulatedArgSafe);
    }

    if (argValid) {
      return processedArg;
    } else {
      return null;
    }
  }
  // Process an inner argument which has the flag set to be processed as a tool call
  private Symbol processToolArgument(String argumentName, Object argumentValue, ToolParameter argumentDefinition, ToolDefinition parentTool, Symbol actor, Utterance.Builder builder, Map<Symbol,Variable> varMap, StringBuilder errorMessage, boolean unpopulatedArgSafe) {
    Map<String,Object> toolDefinitionMap;
    boolean validToolDefinition = true;
    if (argumentValue instanceof String) {
      try {
        toolDefinitionMap = new Gson().fromJson((String)argumentValue, new TypeToken<LinkedTreeMap<String, Object>>() {
        }.getType());
        toolDefinitionMap.values().removeIf(Objects::isNull);
      } catch (JsonSyntaxException e) {
        //TODO: added this 4/28, definitely definitely duplicating a ton of cases at this point
        try {
          log.info("[processToolArgument] hit array as string case, trying to handle as list");
          String temp = (String)argumentValue;
          if (temp.startsWith("[") && temp.endsWith("]")) {
            while (temp.startsWith("[")) {
              temp = temp.substring(1);
            }
            while (temp.endsWith("]")) {
              temp = temp.substring(0,temp.length()-1);
            }
            argumentValue = "[" + temp + "]";
          }
          List<Object> argObjectList = new GsonBuilder().setLenient().create().fromJson((String)argumentValue, new TypeToken<ArrayList<Object>>() {
          }.getType());
          List<Symbol> innerArgs = new ArrayList<>();
          for (Object argValue : argObjectList) {
            Symbol innerArg = processToolArgument(argumentName, argValue, argumentDefinition, parentTool, actor, builder, varMap, errorMessage, unpopulatedArgSafe);
            if (innerArg == null) {
              return null;
            }
            innerArgs.add(innerArg);
          }
          return Factory.createPredicate("and", innerArgs);
        } catch (JsonSyntaxException e2) {
          //TODO: this error message doesn't actually cause the LLM to fix itself all the time - manually inspect what type of object it is and try to process as such manually?
          errorMessage.append("- The supplied tool call definition for the \"").append(argumentName).append("\" parameter of the \"").append(parentTool.function.name).append("\" call could not be parsed as a valid json string. Please reformat your call to supply a valid json string or supply an object directly. If you are supplying an array, do not supply it as a string; supply the object directly. You do not need to speak with the user about this. \n");
          log.error("[processToolCall] malformed json present in functionCall", e);
          return null;
        }
      }
    } else if (argumentValue instanceof Map<?,?>) {
      toolDefinitionMap = (Map<String, Object>) argumentValue; //Catch and handle ClassCastException if ever encountered
    } else if (argumentValue instanceof List<?>) {
      //TODO: added this 4/27, definitely duplicating a ton of cases at this point
      //// Handle List
      List<Object> argObjectList = (List<Object>) argumentValue; //Can you actually have a ClassCastException here? Catch and handle if so
      List<Symbol> innerArgs = new ArrayList<>();
      for (Object argValue : argObjectList) {
        Symbol innerArg = processToolArgument(argumentName, argValue, argumentDefinition, parentTool, actor, builder, varMap, errorMessage, unpopulatedArgSafe);
        if (innerArg == null) {
          return null;
        }
        innerArgs.add(innerArg);
      }
      return Factory.createPredicate("and", innerArgs);
    } else {
      errorMessage.append("- The supplied tool call definition for the \"").append(argumentName).append("\" parameter of the \"").append(parentTool.function.name).append("\" call could not be parsed as json. Please reformat your call to supply a valid json string or object. You do not need to speak with the user about this.\n");
      log.error("[processToolArgument] unhandled {}", argumentValue);
      return null;
    }

    String chatTemplateContextId = argumentDefinition.getToolChatTemplateContextId();
    if (chatTemplateContextId == null || chatTemplateContextId.isEmpty()) {
      log.error("[processArgument] inner tool call has processAsTool flag set but no associated ChatTemplateContextId. **Developer Error - please check your LLM subclass implementation** Cannot recover from this...: {}", argumentName);
      //TODO: definitely developer error at this point - what do we do?
      return null;
    }
    ChatTemplates innerToolChatTemplates = chatTemplates.get(chatTemplateContextId);
    if (innerToolChatTemplates == null) {
      log.error("[processArgument] inner tool call's ChatTemplateContextId does not match an existing ChatTemplates entry: {}. **Developer Error - please check your LLM subclass implementation** Cannot recover from this...", argumentName);
      //TODO: also definitely developer error - what do we do?
      return null;
    }
    toolDefinitionMap.values().removeIf(Objects::isNull);

    //TODO: remove this block and extend framework to allow for arbitrary definition of specific json object forms without necessarily being tool calls
    if (argumentName.equals("locationActionPairs")) {
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

    String toolName = (String)toolDefinitionMap.get("name");
    Object innerArguments = toolDefinitionMap.get("arguments");
    if (toolName == null) {
      log.warn("[processArgument] inner tool call does not have name field present: {}", argumentValue);
      errorMessage.append("- You supplied action json for the \"").append(argumentName).append("\" argument of the \"").append(parentTool.function.name).append("\" call, but it was missing a \"name\" field. Please include the proper name of this action.\n");
      validToolDefinition = false;
    }
    if (innerArguments == null) {
      log.warn("[processArgument] inner tool call does not have arguments field present: {}", argumentValue);
      errorMessage.append("- You supplied action json for the \"").append(argumentName).append("\" argument of the \"").append(parentTool.function.name).append("\" call, but it was missing an \"arguments\" field. Please restructure your json to properly include an arguments field for this action, even if an empty map. You do not need to speak to the user about this.\n");
      validToolDefinition = false;
    }
    Tool innerTool = innerToolChatTemplates.tools.get(toolName);
    if (innerTool == null) {
      log.warn("[processArgument] inner tool call's name does not match a Tool in the corresponding ChatTemplates: {}", toolName);
      errorMessage.append("- You supplied action json for the \"").append(argumentName).append("\" argument of the \"").append(parentTool.function.name).append("\" call, but the supplied name \"").append(toolName).append("\" does not correspond to the name of a valid action. Please remove this entry and replace it with a valid action, work with the user if necessary.\n");
      validToolDefinition = false;
    }

    if (validToolDefinition) {
      return processToolArgument(toolName, innerArguments, innerTool, actor, builder, varMap, errorMessage, unpopulatedArgSafe);
    } else {
      return null;
    }
  }
  // Process extracted tool call information
  private Symbol processToolArgument(String name, Object arguments, Tool intendedTool, Symbol actor, Utterance.Builder builder, Map<Symbol,Variable> varMap, StringBuilder errorMessage, boolean unpopulatedArgSafe) {
    boolean validParse;
    Map<String, Object> calledArguments;
    if (arguments instanceof String) {
      try {
        calledArguments = new Gson().fromJson((String)arguments, new TypeToken<LinkedTreeMap<String, Object>>() {
        }.getType());
        calledArguments.values().removeIf(Objects::isNull);
      } catch (JsonSyntaxException e) {
        errorMessage.append("- The supplied tool call arguments string for the \"").append(name).append("\" tool could not be parsed as a valid json string. Please reformat your call to supply a valid json string or supply an object directly. You do not need to speak with the user about this. \n");
        log.error("[processToolCall] malformed json present in functionCall", e);
        return null;
      }
    } else if (arguments instanceof Map<?,?>) {
      calledArguments = (Map<String, Object>) arguments; //Catch and handle ClassCastException if ever encountered
    } else {
      errorMessage.append("- The supplied tool call arguments for the \"").append(name).append("\" tool could not be parsed as json. Please reformat your call to supply a valid json string or object. You do not need to speak with the user about this.\n");
      log.error("[processToolArgument] unhandled {}", arguments);
      return null;
    }

    // Check all required arguments exist
    ToolDefinition toolDefinition = intendedTool.toolDefinition;
    validParse = checkRequiredArgsExist(calledArguments.keySet(), toolDefinition.function.parameters, errorMessage);

    // Add actor to output semantics
    List<Symbol> processedArguments = new ArrayList<>();
    //TODO: QUESTION should not be in this case once ITK pipeline is fixed - and/or make distinction between going through
    //      the itk pipeline to have it generate its own response vs calling the underlying action to populate a tool response
    if (intendedTool.utteranceType == UtteranceType.INSTRUCT || intendedTool.utteranceType == UtteranceType.QUESTION) {
      String actorName = actor.getName();
      processedArguments.add(Factory.createSymbol(actorName));
    }

    // Validate Tool call's supplied arguments
    for (String argName : calledArguments.keySet()) {
      Symbol processedArg;
      ToolParameter targetParam = toolDefinition.function.parameters.getProperties().get(argName);
      if (targetParam == null) {
        // An argument was supplied which isn't present in the target set
        //  (This is more just an LLM issue? Do we do anything in this case, or just discard the information?)
        // TODO: Count as internal error?
        log.warn("[processToolArgument] argument supplied by LLM in tool call which doesn't exist in tool definition. Discarding.");
      } else {
        //TODO: general mechanism to ignore fields when extracting final result? E.g. want the llm to keep track of the
        // template during the extended dialogue, but don't care if it's fully valid yet.
        if (intendedTool.toolDefinition.function.name.equals(textResponseTool.toolDefinition.function.name) && !argName.equals("response")) {
          //TODO: Revisit where to best detect and address this in general when transitioning to collapse focused
          //  cases into standardized representation/dynamic gen of tool targets and prompts. On transition, if
          //  text response target is called but the template is fully defined, instruct LLM to try calling
          //  template submission tool directly. Currently using a lot of duplicate code.
          Utterance temp = builder.build();
          ChatCollection collection = getChatCollection(temp.getSpeaker().toUntyped(), temp.getAddressee().toUntyped());
          if (collection != null) {
            ChatContext context = collection.currentChat;
            if (context!= null) {
              context.lastTemplateName = argName;
              context.lastTemplateValue = calledArguments.get(argName);
            }
          }
          validParse = validParse && !checkForFullyPopulatedTemplate(argName, calledArguments.get(argName), temp.getSpeaker(), temp.getAddressee(), errorMessage);
          continue;
        }
        Object argValue = calledArguments.get(argName);
        processedArg = processArgument(argName, argValue, targetParam, toolDefinition, actor, builder, varMap, errorMessage, unpopulatedArgSafe);
        if (processedArg == null) {
          validParse = false;
          continue;
        }
        processedArguments.add(processedArg);
      }
    }

    // Return appropriate object
    if (validParse) {
      return Factory.createPredicate(toolDefinition.function.name, processedArguments);
    } else {
      return null;
    }
  }
  // TODO: Do we need to check for unintended JSON strings and reject here?
  // Process a "leaf" (i.e. maximally a Predicate, not list or map) tool call argument
  private Symbol processSymbolArgument(String argName, String argString, List<String> possible_values, StringBuilder errorMessage, Map<Symbol,Variable> varMap, Utterance.Builder builder, boolean inner, boolean unpopulatedArgSafe) {
    Symbol processedArg;
    Symbol argValue = Factory.createFOL(argString);
    // If there is an explicit possible_values set, make sure the argument is present
    if (!possible_values.isEmpty()) {
      //TODO: what general fuzzy matching do we want here? Or is the responsibility on doing so in whatever inner action
      //  and providing the simplest variation here?
      if (!possible_values.contains(argString)) {
        errorMessage.append("- Your tool call contained an invalid value for a ").append(inner ? "inner " : "").append("\"").append(argName).append("\" argument. Please correct any spelling, punctuation, or capitalization mistakes to exactly match a value in the possible_values set if possible. If an obvious similar possible values exists, you may update your call with that value. Otherwise, if no obvious candidates exist, work with the user to specify a valid value as a replacement. Supplied value: ").append(argString).append(", possible values: ").append(possible_values).append(". Do not list out all possible values if responding to the user, unless they ask for that information. If this argument is not required, you may remove it.\n");
        return null;
      }

      // Replace reference descriptor with variable if type contained in refTypes
      //TODO: can probably refactor the below blocks to eliminate duplicate code
      if (refTypes.contains(argValue.getType())) {
        if (!varMap.containsKey(argValue)) {
          Variable var = Factory.createVariable("VAR" + varMap.size() + ":" + argValue.getType());
          varMap.put(argValue, var);

          builder.addSupplementalSemantics(Factory.createPredicate(argValue.toUntypedString(), var));
          Symbol tier = Factory.createSymbol("DEFINITE");
          builder.addTierAssignment(var, tier);

          processedArg = var;
        } else {
          processedArg = varMap.get(argValue);
        }
      } else {
        processedArg = argValue;
      }
      return processedArg;
    } else {
      // Unrestricted argument (at least explicitly), make sure it is otherwise valid (populated, typed correctly, etc.)
      if (argValue != null && argValue.isPredicate()) {
        List<Symbol> processedInnerArguments = new ArrayList<>();
        if (argValue.getName().equals("getIfCanPerform")) { // can this be purely on the tool call level?
          unpopulatedArgSafe = true;
        }
        for (Symbol innerArg : ((Predicate) argValue).getArgs()) {
          Symbol processedInnerArg = processSymbolArgument(argName, innerArg.toString(), new ArrayList<>(), errorMessage, varMap, builder, true, unpopulatedArgSafe);
          processedInnerArguments.add(processedInnerArg);
        }
        if (processedInnerArguments.contains(null)) {
          return null;
        }
        return Factory.createPredicate(argValue.getName(), processedInnerArguments);
      } else {
        // Reject any unpopulated arguments returned by the LLM, unless the semantics specifically allow for that
        if (unpopulatedArgSafe && argString.startsWith("?")) {
          return Factory.createSymbol(argString.substring(1).split(":")[0]);
        } else if (argString.startsWith("?") || (argString.length() == 1 && Character.isUpperCase(argString.charAt(0)))) {
          log.warn("[processSymbolArgument] NLU parse has unpopulated argument present, failing: {}", argString);
          if (inner) {
            errorMessage.append("- The tool call's supplied value for a \"").append(argName).append("\" argument was missing information about the ").append(argString).append(" detail\n");
          } else {
            errorMessage.append("- The tool call was missing a value for a required \"").append(argName).append("\" argument\n");
          }
          return null;
        }

        Symbol argSymbol = Factory.createFOL(argString);
        // Replace reference symbol with Variable and add supplemental semantics
        if (argSymbol != null && (refTypes.contains(argSymbol.getType()))) {
          if (!varMap.containsKey(argSymbol)) {
            Variable var = Factory.createVariable("VAR" + varMap.size() + ":" + argSymbol.getType());
            varMap.put(argSymbol, var);

            builder.addSupplementalSemantics(Factory.createPredicate(argSymbol.toUntypedString(), var));
            Symbol tier = Factory.createSymbol("DEFINITE");
            builder.addTierAssignment(var, tier);

            return var;
          } else {
            return varMap.get(argSymbol);
          }
        } else {
          //TODO: this isn't always the case, if we want to allow variables to be generated by the LLM this needs to change and the prompt needs to be updated
          //Domain-specific safeguard to prevent the LLM from capitalizing arguments
          //If argument is a wildcard, don't want it to be capitalized by the LLM just because it recognizes it as a proper noun
          if (Character.isUpperCase(argString.charAt(0)) && argString.length() > 1) {
            try {
              TimeUnit.valueOf(argString);
              return Factory.createSymbol(argString);
            } catch (IllegalArgumentException ignored) {
              String lowercaseArg = argString;
              lowercaseArg = Character.toLowerCase(lowercaseArg.charAt(0)) + lowercaseArg.substring(1);
              return Factory.createSymbol(lowercaseArg);
            }
          } else {
            return Factory.createSymbol(argString);
          }
        }
      }
    }
  }
  private boolean checkRequiredArgsExist(Set<String> calledArgumentKeys, ToolParameters targetParameters, StringBuilder currentErrorMessage) {
    boolean valid = true;
    for (String paramName : targetParameters.getProperties().keySet()) {
      if (targetParameters.getRequired().contains(paramName)) {
        if (!calledArgumentKeys.contains(paramName)) {
          valid = false;
          currentErrorMessage.append("- The tool call was missing a value for a required \"").append(paramName).append("\" argument\n");
        }
      }
    }
    return valid;
  }
  private boolean checkForFullyPopulatedTemplate(String templateArgName, Object templateValue, Symbol user, Symbol agentTeam, StringBuilder errorMessage) {
    ChatCollection collection = getChatCollection(user,agentTeam);
    if (collection != null) {
      ChatContext context = collection.currentChat;
      if (context.history.messages.size() == 1) {
        //If template is fully populated -> add tool response indicating to call submission directly, set output to null, and loop
        Map<String,Object> templateMap = null;
        if (templateValue instanceof String) {
          try {
            templateMap = new Gson().fromJson((String)templateValue, new TypeToken<LinkedTreeMap<String, Object>>() {
            }.getType());
            templateMap.values().removeIf(Objects::isNull);
          } catch (JsonSyntaxException e) {
            log.error("[processToolCall] malformed json present in functionCall", e);
          }
        } else if (templateValue instanceof Map<?,?>) {
          templateMap = (Map<String, Object>) templateValue; //Catch and handle ClassCastException if ever encountered
        }
        if (templateMap != null) {
          int expectedSize = -1;
          if (templateArgName.equals("currentScheduledDefinition")) {
            expectedSize = 4;
          } else if (templateArgName.equals("currentTaskSequenceDefinition")) {
            expectedSize = 2;
          } else {
            log.warn("[checkForFullyPopulatedTemplate] unexpected in-progress template argument name {}", templateArgName);
          }
          if (templateMap.size() == expectedSize) {
            for (Object templateInnerArg : templateMap.values()) {
              if (templateInnerArg instanceof String) {
                if (((String)templateInnerArg).isEmpty()) {
                  return false;
                }
              } else if (templateInnerArg instanceof Map) {
                if (((Map)templateInnerArg).isEmpty()) {
                  return false;
                }
              } else if (templateInnerArg instanceof List) {
                if (((List)templateInnerArg).isEmpty()) {
                  return false;
                }
              } else {
                return false;
              }
            }
            log.info("[checkForFullyPopulatedTemplate] detected full template definition in first response after a " +
                    "transfer to a focused context - recommending the LLM to directly submit the template definition.");
            // Should we rephrase the base error message or make a new one in this case?
            errorMessage.append("- It looks like your current definition of the target to be submitted is fully filled " +
                    "out. If this seems to be the case, try submitting the target submission tool call directly instead" +
                    "of responding to the user.");
            return true;
          }
        }
      }
    }
    return false;
  }

  Predicate replaceReferencesInSemantics(Predicate semantics) {
    List<Symbol> transformedArgs = new ArrayList<>();
    for (Symbol arg: semantics.getArgs()) {
      if (arg.isPredicate()) {
        transformedArgs.add(replaceReferencesInSemantics((Predicate) arg));
      } else {
        if (refTypes.contains(arg.getType())) {
          Map<Symbol, List<Term>> refs = new HashMap<>();
          Symbol transformedArg = null;
          int maxAttempts = 3;
          for (int i=0; i<maxAttempts;i++) {
            try {
              refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateRE").argTypes(Symbol.class)).call(Map.class, arg);
              if (refs.get(arg).size() == 1) {
                transformedArg = Factory.createSymbol(refs.get(arg).get(0).getName(), refs.get(arg).get(0).get(0).getType());
              }
            } catch (TRADEException e) {
              log.warn("[generateREG] {}", arg, e);
            }
          }
          if (transformedArg != null) {
            transformedArgs.add(transformedArg);
          } else {
            transformedArgs.add(arg);
          }
        } else {
          transformedArgs.add(arg);
        }
      }
    }
    return Factory.createPredicate(semantics.getName(), transformedArgs);
  }

  Predicate replaceVarsWithDescriptors(Predicate semantics, Map<Variable,Symbol> varMap) {
    List<Symbol> transformedArgs = new ArrayList<>();
    for (Symbol arg: semantics.getArgs()) {
      if (arg.isPredicate()) {
        transformedArgs.add(replaceVarsWithDescriptors((Predicate) arg, varMap));
      } else if (arg.isVariable()) {
        transformedArgs.add(varMap.getOrDefault(arg, arg));
      } else {
        transformedArgs.add(arg);
      }
    }
    return Factory.createPredicate(semantics.getName(), transformedArgs);
  }

  private void handleErrorResponse(Utterance utterance, boolean parse, boolean internal) {
    if (utterance == null) {
      log.error("[handleErrorResponse] supplied utterance is null");
      return;
    }

    Symbol user;
    Symbol agentTeam;
    if (parse) {
      user = utterance.getSpeaker().toUntyped();
      agentTeam = utterance.getAddressee().toUntyped();
    } else {
      user = utterance.getAddressee().toUntyped();
      agentTeam = utterance.getSpeaker().toUntyped();
    }

    String errorMessage = "Sorry, I am unable to connect to necessary services. Please check your internet connection or reach out to support.";
    if (internal) {
      // Now proactively catching all manner of issues and prompting the LLM to reformat responses to overcome them.
      // We now only reach his method if the LLM has already failed a certain number of times without a parseable
      // response. Clear chat and send error response immediately.
      ChatContext chatContext = null;
      ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
      if (chatCollection != null) {
        chatContext = chatCollection.currentChat;
        if (chatContext != null) {
          endChatContext(user, agentTeam);
          //errorMessage = generateErrorResponse(internal, 3, false);
        }
      }
      //TODO: do we want to return null / some other parse if the chat context had been cleared in the interim? Does it
      //  practically matter? If it's cleared I think that should mean this response isn't used anyway.
      errorMessage = "Sorry, an internal error occurred during your request which I could not overcome. I needed to reset my conversation state, please try starting over.";
    } else {
      externalErrorCount += 1;
      if (externalErrorCount >= 3) {
        endChatContext(user, agentTeam);
        if (!restrictedMode) {
          try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("setHybridParserRestrictedMode").argTypes(Boolean.class)).call(void.class, true);
            TRADE.getAvailableService(new TRADEServiceConstraints().name("setHybridNLGRestrictedMode").argTypes(Boolean.class)).call(void.class, true);
            restrictedMode = true;
            errorMessage = "Sorry, I am unable to connect to necessary services. Entering restricted interaction mode. Please try starting over.";
          } catch (TRADEException e) {
            log.info("[generateErrorResponse] could not reach Hybrid parser to set restricted mode. Assuming we are only using the LLM parser", e);
          }
        }
      } else {
        errorMessage = generateErrorResponse(internal, externalErrorCount, false);
      }
    }
    if (parse) {
      utterance.setSemantics(Factory.createPredicate("generateResponseFromString", agentTeam.toUntypedString(), user.toUntypedString(),errorMessage));
      utterance.setNeedsValidation(false);
    } else {
      utterance.setWords(errorMessage);
      // TODO: update - Hack alongside HybridNLGComponent to detect failure responses.
      utterance.setType(UtteranceType.UNKNOWN);
    }
  }

  private String generateErrorResponse(boolean internal, int errorCount, boolean predicate) {
    if (internal) {
      if (errorCount == 1) {
        return "Sorry, an internal error occurred during your request. Please try again.";
      } else if (errorCount == 2) {
        return "Sorry, an internal error occurred during your request again. Please try one more time.";
      } else {
        return "Sorry, an internal error occurred during your request. I have reset my state, please try starting over.";
      }
    } else {
      if (errorCount == 1) {
        return predicate ? "and(sorryIDidNotHaveConnectionDuringYourRequest,pleaseTryAgain)" : "Sorry, I did not have connection during your request. Please try again.";
      } else if (errorCount == 2) {
        return predicate ? "and(sorryIDidNotHaveConnectionDuringYourRequest,pleaseTryOneMoreTime)" : "Sorry, I did not have connection during your request again. Please try one more time.";
      } else {
        return predicate ? "and(sorryIAmUnableToConnectToNecessaryServices,pleaseReconnectOrReachOutToSupport)" : "Sorry, I am unable to connect to necessary services. Please check your internet connection or reach out to support.";
      }
    }
  }

  private void resetErrorCount(boolean internal, Symbol user, Symbol agentTeam) {
    if (internal) {
        ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
        if (chatCollection != null) {
          ChatContext chatContext = chatCollection.currentChat;
          if (chatContext != null) {
            chatContext.resetNumParseErrors();
          }
        }
    } else {
      externalErrorCount = 0;
      if (restrictedMode) {
        try {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("setHybridParserRestrictedMode").argTypes(Boolean.class)).call(void.class, false);
          TRADE.getAvailableService(new TRADEServiceConstraints().name("setHybridNLGRestrictedMode").argTypes(Boolean.class)).call(void.class, false);
          restrictedMode = false;
        } catch (TRADEException e) {
          log.error("[resetErrorCount] could not reach Hybrid parser to unset restricted mode.", e);
        }
      }
    }
  }

  public List<Message> getChatHistory(Symbol user, Symbol agentTeam) {
    ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
    if (chatCollection != null) {
      ChatContext chatContext = chatCollection.currentChat;
      if (chatContext != null) {
        return chatContext.history.messages;
      }
    }
    return null;
  }

  public String getChatHistoryAsJSON(Symbol user, Symbol agentTeam) {
    ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
    if (chatCollection != null) {
      ChatContext chatContext = chatCollection.currentChat;
      if (chatContext != null) {
        return new Gson().toJson(chatContext.history.messages);
      }
    }
    return null;
  }

  public String getChatHistoryAsString(Symbol user, Symbol agentTeam) {
    ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
    if (chatCollection != null) {
      ChatContext chatContext = chatCollection.currentChat;
      if (chatContext != null) {
        return getChatHistoryAsString(chatContext.history);
      }
    }
    return null;
  }

  public String getChatHistoryAsString(Chat chat) {
    if (chat == null) {
      return "";
    }
    Gson gson = new GsonBuilder().setPrettyPrinting().create();
    //return chat.getMessages().stream().map(m -> m.role + ": " + ((m.tool_calls == null || m.tool_calls.length < 1) ? m.content : gson.toJson(m.tool_calls[0]))).collect(Collectors.joining("\n"));
    return chat.getMessages().stream().map(gson::toJson).collect(Collectors.joining("\n"));
  }

  @TRADEService
  public String getLastResponseString(Symbol user, Symbol agentTeam) {
    ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
    if (chatCollection != null) {
      ChatContext chatContext = chatCollection.currentChat;
      if (chatContext != null) {
        return chatContext.lastResponseString;
      }
    }
    return "";
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// Should be domain specific

  // TODO: Revisit old error cases and explicitly handle if finding they are still possible
  // Modify semantics based on domain specific postprocessing
  // Return - Error String to add onto baseToolResponse to be sent when hanlding this uetterance as an error.
  //          Empty String if no issues found
  private String correctCommonNLUMistakes(Utterance utterance) {
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
                args.add(arg);
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
                args.add(arg);
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
                args.add(arg);
              }
            }
            else {
              args.add(arg);
            }
          }
        }
        if (!hasStart && !hasPeriod) {
          errorStringBuilder.append("- The submitted scheduled task definition must have a future start time or an interval to repeat on. Scheduled tasks may not occur immediately and without repeating on some interval. Let the user know this fact and ask for one of these values.");
        } else if (hasEnd && !hasPeriod) {
          errorStringBuilder.append("- The submitted scheduled task definition contains an explicit end time, but no defined interval to repeat on. Either discard the end time if the task is not meant to repeat, or fill out the repeat interval appropriately. If you cannot do so on your own, you may let the user know a properly defined execution interval is necessary.");
        }

        if (errorStringBuilder.isEmpty()) {
          utterance.setSemantics(Factory.createPredicate("scheduled", args));
        } else {
          return errorStringBuilder.toString();
        }
      } else if (command.equals("submitTaskSequenceDefinition")) {
        List<Symbol> args = ((Predicate)utterance.getSemantics()).getArgs();
        if (args.size() != 3) {
          log.error("[correctCommonNLUMistakes] tourDefinition has unexpected number of arguments, this should have been caught in tool processing?");
        } else {
          String tourName = args.get(1).toString();
          if (!tourName.matches("[A-Za-z0-9_]+")) {
            Symbol cleanedName = Factory.createSymbol(tourName.trim().replaceAll("\\s+", "_").replaceAll("[^a-zA-Z0-9_]", ""));
            args.remove(1);
            args.add(1, cleanedName);
            utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
          }
        }
      } else if (command.equals("getExtendedConversationHistory")) {
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

  @TRADEService
  @Action
  public boolean ignoreTentativeAccept(Predicate goalPred){
    log.info("[ignoreTentativeAccept] goalPred: "+goalPred);
    if(goalPred != null) {
      switch (goalPred.getName()) {
        case "describeScheduledGoals":
        case "describeCurrentLocation":
        case "describePendingGoals":
        case "describeAllKnownLocations":
        case "repeatLastQuestion":
        case "describeCurrentGoal":
        case "reportTaughtActions":
        case "reportInstructableActions":
        case "describeActionSteps":
        case "reportIfCanPerform":
        case "reportKioskSettings":
        case "listSavedVideos":
        case "reportVideoVolume":
        case "listSavedImages":
        case "reportIsVideoSaved":
        case "sayText":
        case "say":
        case "reportBatteryLevel":
        case "generateResponseFromString":
        case "getAllKnownLocations":
        case "getBatteryLevelString":
        case "getCurrentLocationString":
        case "getTaughtActions":
        case "getIfCanPerform":
        case "getPredefinedActions":
        case "getActionSteps":
        case "getKioskSettings":
        case "getVideoVolume":
        case "getIsVideoSaved":
        case "getSavedVideos":
        case "getSavedImages":
        case "getWifiAtLocationString":
        case "getTemperatureAtLocationString":
        case "getCurrentPendingScheduledGoalInformation":
        case "resetConversation":
        case "learnTaskSequence":
        case "scheduleTask":
        case "getExplanationOfHowToTeachATask":
        case "getExplanationOfWhatALearnedTaskIs":
        case "getExplanationOfWhatActionsCanBePerformedAlongALearnedTask":
        case "getWifiAtLocationsString":
        case "getWifiAtCurrentLocationString":
        case "getTemperatureAtLocationsString":
        case "getTemperatureAtCurrentLocationString":
        //// Set of actions which should perhaps be "reduced ack"?
        //case "escortToLocation":
          return true;
        default:
          return false;
      }
    }
    return false;
  }


  /**
   *  Convert a Tool definition to a description string for the LLM to use as reference - want to give the information
   *  of what may have been called by the system previously or what can be used as an inner argument to another tool
   *  without actually advertising these actions as valid Tools themselves (e.g. NLG, goal scheduling, action ITK)
   *
   * (TODO: keep an eye on this, this information can not be formatted in a way familiar to the LLM as a tool call, or
   *        else it will try to call it)
   */
  private String toolToDescString(Tool tool, boolean includePossibleValues) {
    edu.tufts.hrilab.llm.gson.tool.ToolFunction toolFunction = tool.toolDefinition.function;
    List<String> required = toolFunction.parameters.getRequired();
    String argumentsString;
    if (includePossibleValues) {
      return new Gson().toJson(toolFunction);
    } else {
      argumentsString = toolFunction.parameters.getProperties().isEmpty() ? "None" : "\n" +
              toolFunction.parameters.getProperties().entrySet().stream().map(entry ->
                      entry.getKey() + " (" + (required.contains(entry.getKey()) ? "necessary" : "optional") + "): "
                              + entry.getValue().getDescription()).collect(Collectors.joining("\n"));
    }
    return toolFunction.name + "(" + String.join(",",toolFunction.parameters.getProperties().keySet()) + ")" +
            "\nExplanation: " + toolFunction.description +
            "\nParameters:" + argumentsString;
  }

  /**
   * Generate the base INSTRUCT UtteranceType toolset available for this domain
   */
  //TODO: Apply prompt variables dynamically (e.g. update location lists)
  //TODO: Think of a better way to define / gather this information
  //TODO: rephrase descriptions to align better with final "social" system prompt
  private List<Tool> generateInstructTools() {
    List<Tool> instructTools = new ArrayList<>();
    ToolDefinition definition;
    List<String> locationNames = getLocations("self", true);

    definition = new ToolDefinition("goToLocation",
            "You go to the supplied destination location. An example of instructing this could be \"please navigate to %exampleLocation\", which would map to {\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}.",
            List.of(new ToolParameterHelper("destination", "string", locationNames, "The destination location to go to", true))
    );
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("dock",
//            "You \"dock\" at the supplied dock destination (meaning you idly charge on your charging port). An example of instructing this could be \"dock at %exampleLocation\", which would map to dock(%exampleLocation).",
//            List.of(new ToolParameterHelper("destination", "string", locationNames, "The name of the dock tag", true))
//    );
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("follow",
            "You follow the user. An example of instructing this could be \"follow me.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("say",
            "The robot says ?words. This should be targeted when a user is explicitly instructing you to say something, most commonly in the context of another task, to another person who is not the user, or at a later time. An example of instructing this could be \"say to everyone: welcome to the event!.\", which would map to {\"name\":\"say\", \"arguments\":{\"words\":\"\\\"welcome to the event!\\\"\"}}",
            List.of(new ToolParameterHelper("words", "string", new ArrayList(), "The phrase to be said", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("stopFollow",
            "You stop following the user. An example of instructing this could be \"stop following me.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("escortToLocation",
            "You lead someone from your current location to a different one. An example of instructing this could be \"bring Owen to %exampleLocation.\", which would map to {\"name\":\"escortToLocation\", \"arguments\":{\"name\":\"owen\",\"destination\":\"%exampleLocation\"}}",
            List.of(new ToolParameterHelper("name", "string", new ArrayList(), "Name of the entity to escort", true),
                    new ToolParameterHelper("destination", "string", locationNames, "The destination location to escort to", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("escortFromLocationToLocation",
            "You meet someone at the specific start location and then lead them to the destination location. An example of instructing this could be \"bring our guest from %exampleLocation to the dock.\", which would map to {\"name\":\"escortFromLocationToLocation\", \"arguments\":{\"name\":\"guest\",\"start\":\"%exampleLocation\",\"destination\":\"dock:location\"}}",
            List.of(new ToolParameterHelper("name", "string", new ArrayList(), "Name of the entity to escort", true),
                    new ToolParameterHelper("start", "location", new ArrayList(), "The start location to escort from. This should be where ?name can be found", true),
                    new ToolParameterHelper("destination", "location", new ArrayList(), "The destination location to escort to", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("saveLocation",
            "You save your current location under the supplied name. An example of instructing this could be \"save this area as location one.\", which would map to {\"name\":\"saveLocation\", \"arguments\":{\"name\":\"location one\"}}",
            List.of(new ToolParameterHelper("name", "string", new ArrayList(), "the label for the location to save", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //definition = new ToolDefinition("saveLocationsToFile()",
    //        "You backup all your currently saved locations",
    //        "backup your current locations",
    //        "saveLocationsToFile()", 10);
    //toolDefinitions.add(definition);
//    definition = new ToolDefinition("sit",
//            "You lower your screen. An example of instructing this could be \"sit down.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("stand",
//            "You raise your screen. An example of instructing this could be \"stand up.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //definition = new ToolDefinition("deliver(?objectName, ?location:location)",
    //        "You deliver ?objectName to location ?location:location",
    //        "bring the mug to %exampleLocation",
    //        "deliver(mug,%exampleLocation)", 10);
    //toolDefinitions.add(definition);
    definition = new ToolDefinition("goHome",
            "You go to your default charging dock and charge. An example of instructing this could be \"go back to charge.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("measureWifi",
            "You measure the wifi signal strength at your current location when this task is performed. Use this when a user generally asks you to survey or check wifi. An example of instructing this could be \"check the wifi signal strength\", which would map to {\"name\":\"measureWifi\", \"arguments\":{}}",
            new ArrayList<>());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("measureWifiAtLocations",
            "You drive to each of the supplied locations in order and measure wifi signal strength at each one. Use this when a user asks you to survey or check wifi at a set of places. For a single location, still supply a one-element list. An example of instructing this could be \"check the wifi at %exampleLocation and the dock\", which would map to {\"name\":\"measureWifiAtLocations\", \"arguments\":{\"locations\":[\"%exampleLocation\",\"dock:location\"]}}",
            List.of(new ToolParameterHelper("locations", "array", locationNames, "The ordered list of saved locations to visit and measure. Each value must be a saved location.", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("measureWifiAtLocationsAndReportAt",
            "You drive to each of the supplied locations in order, measure wifi at each one, then drive to reportLocation and speak a brief summary. Use this when the user wants a survey with a specific return-and-report location.",
            List.of(new ToolParameterHelper("locations", "array", locationNames, "The ordered list of saved locations to visit and measure. Each value must be a saved location.", true),
                    new ToolParameterHelper("reportLocation", "string", locationNames, "The saved location to return to before speaking the summary.", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("measureTemperature",
            "You measure the temperature in degrees Fahrenheit at your current location when this task is performed. Use this when a user generally asks you to check temperature. An example of instructing this could be \"check the temperature\", which would map to {\"name\":\"measureTemperature\", \"arguments\":{}}",
            new ArrayList<>());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("measureTemperatureAtLocations",
            "You drive to each of the supplied locations in order and measure temperature in degrees Fahrenheit at each one. Use this when a user asks you to survey or check temperature at a set of places. For a single location, still supply a one-element list.",
            List.of(new ToolParameterHelper("locations", "array", locationNames, "The ordered list of saved locations to visit and measure. Each value must be a saved location.", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("measureTemperatureAtLocationsAndReportAt",
            "You drive to each of the supplied locations in order, measure temperature at each one, then drive to reportLocation and speak a brief summary. Use this when the user wants a temperature survey with a specific return-and-report location.",
            List.of(new ToolParameterHelper("locations", "array", locationNames, "The ordered list of saved locations to visit and measure. Each value must be a saved location.", true),
                    new ToolParameterHelper("reportLocation", "string", locationNames, "The saved location to return to before speaking the summary.", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //definition = new ToolDefinition("scheduleTask",
    //        "You begin a dialogue interaction with the user to schedule future or regularly repeating execution of a task. An example of instructing this could be \"I want to schedule a goal.\"",
    //        new ArrayList());
    //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("modifyScheduledGoal",
//            "You begin a dialogue interaction with the user to modify the timing of a future already scheduled task. An example of instructing this could be \"I want to change the timing of an action.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("cancelSystemGoals",
            "You stop and fully discard your current task. An example of instructing this could be \"cancel\" or \"stop pursuing your current task\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("suspendSystemGoalsBlocking",
            "You pause your current task, which can be later resumed. An example of instructing this could be \"stop\" or \"pause what you're doing.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("resumeSystemGoals",
            "You resume the currently paused task. An example of instructing this could be \"resume previous task.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("cancelAllScheduledGoals",
            "You cancel all currently scheduled goals which have yet to begin execution. An example of instructing this could be \"cancel all scheduled tasks.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("cancelScheduledGoal",
//            "You begin a dialogue interaction with the user to cancel a particular scheduled goal. An example of instructing this could be \"I want to cancel a scheduled task.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("learnNewTask",
//            "You begin a dialogue interaction with the user to learn the steps of a new instructable action. An example of instructing this could be \"i will teach you a new task.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("endActionLearning",
//            "You conclude learning the steps of a new instructable action. Examples of instructing this could be \"those are the steps of the task\" or \"im done\".",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("forgetTaughtAction",
            "You forget a previously taught action represented by the supplied actionPredicate. actionPredicate should be assigned a populated instance of another tool call. An example of instructing this could be \"forget how to go to %exampleLocation.\", which would map to {\"name\":\"forgetTaughtAction\", \"arguments\":{\"actionPredicate\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}}}",
            List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "The populated tool call representing the action to forget", true, true, "nlu")));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //definition = new ToolDefinition("learnTourAction",
    //        "You begin a dialogue interaction with the user where they teach you how to perform a new type of tour (navigating between a sequence of locations while performing specific actions at each one). An example of instructing this could be \"I want to teach you how to perform a tour.\"",
    //        new ArrayList());
    //instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("updateKioskTimeout",
//            "You begin a dialogue interaction with the user to update the configured kiosk timeout. An example of instructing this could be \"I would like to update the kiosk timeout.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("setKioskTimeout",
            "You update the configured kiosk timeout. An example of instructing this could be \"Set the kiosk timeout to 30 minutes.\", which would map to {\"name\":\"setKioskTimeout\", \"arguments\":{\"duration\":\"30\",\"timeunit\":\"MINUTES\"}}",
            List.of(new ToolParameterHelper("duration", "integer", new ArrayList(), "The number of timeunits defining the timeout", true),
                    new ToolParameterHelper("timeunit", "string", Arrays.stream(TimeUnit.values()).map(Objects::toString).toList(), "The timeunit of the duration to set ", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("setKioskModeEnabled",
            "You set whether or not kiosk mode is enabled. An example of instructing this could be \"turn on kiosk mode.\", which would map to {\"name\":\"setKioskModeEnabled\", \"arguments\":{\"enabled\":\"true\"}}",
            List.of(new ToolParameterHelper("enabled", "boolean", new ArrayList<>(), "the truth value of kiosk mode being enabled", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("updateKioskAction",
//            "You begin a dialogue interaction with the user to update the configured kiosk action. An example of instructing this could be \"I want to change the kiosk action.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("setVideoVolume",
            "You set the volume of videos played on your screen. An example of instructing this could be \"Set the video volume to max.\", which would map to {\"name\":\"setVideoVolume\", \"arguments\":{\"volume\":\"100\"}}",
            List.of(new ToolParameterHelper("volume", "integer", new ArrayList(), "The volume to set the video media player to. Acceptable values are whole numbers between 0 and 100", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("waitForDuration",
            "You wait idly for a number of seconds. An example of instructing this could be \"Then wait for a minute.\", which would map to {\"name\":\"waitForDuration\", \"arguments\":{\"seconds\":\"60\"}}",
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
            List.of(new ToolParameterHelper("location", "string", locationNames, "The specific location at which to play the video.", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("playSpecificVideo",
            "You begin playback of a specific video from your saved collection on your built-in screen display at your current location. An example of instructing this would be \"play the %exampleVideo\".",
            List.of(new ToolParameterHelper("videoName", "string", getVideoNames(), "The specific video to play.", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("playSpecificVideoAtLocation",
            "You begin playback of a specific video from your saved collection on your built-in screen display at the specified location. An example of instructing this would be \"play the %exampleVideo at the %exampleLocation\".",
            List.of(new ToolParameterHelper("videoName", "string", getVideoNames(), "The specific video to play.", true),
                    new ToolParameterHelper("location", "string", locationNames, "The specific location at which to play the video.", true)
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
            List.of(new ToolParameterHelper("imageName", "string", getImageNames(), "The image to display", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("displayNamedImageAtLocation",
            "You display an image from your saved collection on your built-in display at a specified location indefinitely.",
            List.of(new ToolParameterHelper("imageName", "string", getImageNames(), "The image to display", true),
                    new ToolParameterHelper("location", "string", locationNames, "The location at which to display the image.", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("displayNamedImageForDuration",
            "You display an image from your saved collection on your built-in display at your current location for a certain duration.",
            List.of(new ToolParameterHelper("imageName", "string", getImageNames(), "The image to display", true),
                    new ToolParameterHelper("duration", "integer", new ArrayList<>(), "How long to display the image for in seconds.", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("displayNamedImageForDurationAtLocation",
            "You display an image from your saved collection on your built-in display at a specified location for a certain duration.",
            List.of(new ToolParameterHelper("imageName", "string", getImageNames(), "The image to display", true),
                    new ToolParameterHelper("duration", "integer", new ArrayList<>(), "How long to display the image for in seconds.", true),
                    new ToolParameterHelper("location", "string", locationNames, "The location at which to display the image. ", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("urgentPriority",
            "You submit an action with urgent priority. This means that this action will interrupt execution of any other currently submitted actions and be immediately prioritized. An example of instructing this could be \"go to %exampleLocation right now.\", which would map to {\"name\":\"urgentPriority\", \"arguments\":{\"actionPredicate\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}}}",
            List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "The action to submit with urgent priority. This should be filled with a fully populated instance of another tool call.", true, true, "nlu")));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("cancelTaskInQueueByIndex",
            "You cancel and remove a task in the task queue specified by its index. An example of instructing this could be \"Remove the first task from the queue.\", which would map to {\"name\":\"cancelTaskInQueueByIndex\", \"arguments\":{\"index\":\"0\"}}",
            List.of(new ToolParameterHelper("index", "integer", new ArrayList(), "The 0-based index of the task in the queue to remove", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("enableAdminMode",
//            "Allow certain actions which require administrator privileges to be instructed through voice directly to the robot.. An example of instructing this could be \"enable admin mode.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("disableAdminMode",
//            "Reject certain actions which require administrator privileges from being instructed through voice directly to the robot.. An example of instructing this could be \"disable admin mode.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("switchDisplayToTelepresence",
//            "Change the output on your display to the telepresence stream. An example of instructing this could be \"start telepresence.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
//    definition = new ToolDefinition("switchDisplayToTRACS",
//            "Change the output on your display to what is shown by TRACS, the default setting. An example of instructing this could be \"return to TRACS.\"",
//            new ArrayList());
//    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

    definition = new ToolDefinition("reportWifiAtLocation",
            "Use this to ask the system what wifi signal strength you previously measured at the supplied location is. Only returns a useful value after measureWifiAtLocations has been run for that location. An example of when you may need this information is when a user asks \"how was the signal at %exampleLocation?\"",
            List.of(new ToolParameterHelper("location", "string", getLocations("self", true), "The saved location being queried about", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("reportWifiAtLocations",
            "You report the wifi signal strength you previously measured at each of the supplied locations in the list is. Only returns a useful value for each value after wifi has been measured for that location. An example of when you may need this information is when a user asks \"how was the signal at %exampleLocation, example_location_2, and example_location_3?\"",
            List.of(new ToolParameterHelper("locations", "array", getLocations("self", true), "The ordered list of saved locations to report the wifi signal for. Each value must be a saved location.", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("reportTemperatureAtLocation",
            "You report the temperature (in degrees Fahrenheit) you previously measured at the supplied location. Only returns a useful value after measureTemperatureAtLocations has been run for that location. An example of when you may need this information is when a user asks \"how warm was it at %exampleLocation?\"",
            List.of(new ToolParameterHelper("location", "string", getLocations("self", true), "The saved location being queried about", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("reportTemperatureAtLocations",
            "You report the temperature (in degrees Fahrenheit) you previously measured at each of the supplied locations in the list. Only returns a useful value after measureTemperatureAtLocations has been run for those location. An example of when you may need this information is when a user asks \"how warm was it at %exampleLocation?, example_location_2, and example_location_3?\"",
            List.of(new ToolParameterHelper("locations", "array", getLocations("self", true), "The ordered list of saved locations to report the temperature for. Each value must be a saved location.", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("reportWifiAtCurrentLocation",
            "You report the the wifi signal strength is at your current location. An example of when you may need this information is when a user asks \"how is the signal where you are now\"",
            new ArrayList<>());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("reportTemperatureAtCurrentLocation",
            "You report the the temperature is at your current location. An example of when you may need this information is when a user asks \"what is the temperature where you are now\"",
            new ArrayList<>());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));


    definition = new ToolDefinition("scheduleTask",
            "You begin a dialogue interaction with the user to schedule future or regularly repeating execution of a task. An example of instructing this could be direct like \"I want to schedule a goal\", or indirect like saying they want you to perform a certain action in the future or on an interval",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("learnTaskSequence",
            "You begin a dialogue interaction with the user to learn how to perform a new \"task\", \"action\", or \"tour\" sequence. The learned task sequence will consist of navigating between a set of saved locations while performing actions at each location. An example of instructing this could be \"Let me tell you how to perform a new task\" or the user beginning to define an action sequence for you to save",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

    ////TODO: need to update this with new social extended interaction structure - Do we still even need this as an action? Or should be rephrased to repeatSystemQuestion
    //definition = new ToolDefinition("repeatLastQuestion",
    //        "You repeat the last question you asked, which the human still needs to answer. examples of instructing this could be \"could you repeat that\" or \"what is the question\"",
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
  private List<Tool> generateITKTools() {
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
    //        "Use this to ask the system whether you have a video saved under the supplied name. An example of when you may need this information is when a user asks \"Do you have the %exampleVideo video saved.\", which would map to reportIsVideoSaved(?videoName)",
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
    definition = new ToolDefinition("getWifiAtLocationString",
            "Use this to ask the system what wifi signal strength you previously measured at the supplied location is. Only returns a useful value after measureWifiAtLocations has been run for that location. An example of when you may need this information is when a user asks \"how was the signal at %exampleLocation?\"",
            List.of(new ToolParameterHelper("location", "string", getLocations("self", true), "The saved location being queried about", true)));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition("getWifiAtLocationsString",
            "Use this to ask the system what wifi signal strength you previously measured at each of the supplied locations in the list is. Only returns a useful value for each value after wifi has been measured for that location. An example of when you may need this information is when a user asks \"how was the signal at %exampleLocation, example_location_2, and example_location_3?\"",
            List.of(new ToolParameterHelper("locations", "array", getLocations("self", true), "The ordered list of saved locations to report the wifi signal for. Each value must be a saved location.", true)));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition("getTemperatureAtLocationString",
            "Use this to ask the system what temperature (in degrees Fahrenheit) you previously measured at the supplied location. Only returns a useful value after measureTemperatureAtLocations has been run for that location. An example of when you may need this information is when a user asks \"how warm was it at %exampleLocation?\"",
            List.of(new ToolParameterHelper("location", "string", getLocations("self", true), "The saved location being queried about", true)));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition("getTemperatureAtLocationsString",
            "Use this to ask the system what temperature (in degrees Fahrenheit) you previously measured at each of the supplied locations in the list. Only returns a useful value after measureTemperatureAtLocations has been run for those location. An example of when you may need this information is when a user asks \"how warm was it at %exampleLocation?, example_location_2, and example_location_3?\"",
            List.of(new ToolParameterHelper("locations", "array", getLocations("self", true), "The ordered list of saved locations to report the temperature for. Each value must be a saved location.", true)));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition("getWifiAtCurrentLocationString",
            "Use this to ask the system what the wifi signal strength is at your current location. An example of when you may need this information is when a user asks \"how is the signal where you are now\"",
            new ArrayList<>());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition("getTemperatureAtCurrentLocationString",
            "Use this to ask the system what the temperature is at your current location. An example of when you may need this information is when a user asks \"what is the temperature where you are now\"",
            new ArrayList<>());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));

    // General ITK
    definition = new ToolDefinition("getExplanationOfHowToTeachATask",
            "Use this to get an explanation from the system on how a user can teach you new tasks, which you could relay to the user. An example of when this may be useful to call is when a user asks \"how do I teach you a new task\".",
            new ArrayList<>());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition("getExplanationOfWhatALearnedTaskIs",
            "Use this to get an explanation from the system on what a taught task consists of, which you could use to answer questions from the user. An example of when this may be useful to call is when a user asks \"what information do I need to teach a new task\" or \"how do I instruct something I've taught you\".",
            new ArrayList<>());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition("getExplanationOfWhatActionsCanBePerformedAlongALearnedTask",
            "Use this to get an explanation from the system on what actions you can be asked to perform as part of a taught task, which you could use to answer questions from the user. An example of when this may be useful to call is when a user asks \"what can I tell you to do while teaching a new task\".",
            new ArrayList<>());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition("getActionSteps",
            "Use this to ask the system for a description of an action, in the form of the list the steps which would be performed when it is executed. ?action can be replaced by a populated instance of another tool OR be just the name of another tool if no specific arguments are supplied. An example of when you may need this information is when a user asks \"How would you escort someone.\", which would map to {\"name\":\"getActionSteps\", \"arguments\":{\"action\":\"escort\"}}",
            List.of(new ToolParameterHelper("action", "string", new ArrayList(), "The tool whose execution steps to describe", true)));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition("getKioskSettings",
            "Use this to ask the system for the current configuration of kiosk mode; Whether it is enabled, the currently configured kiosk timeout, and the kiosk action. An example of when you may need this information is when a user asks \"what is your kiosk configuration.\"",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    definition = new ToolDefinition("getExtendedConversationHistory",
            "Use this to ask the system for the extended conversation history between you and the user. In other words, call this to get information about past interactions with the user in case it seems they are referring back to something they discussed with you which you do not have memory of.",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));
    // "If not contained in this history, then you must tell the user you cannot remember this interaction"

    definition = new ToolDefinition("getExtendedConversationHistory",
            "Use this to ask the system for the extended conversation history between you and the user. In other words, call this to get information about past interactions with the user in case it seems they are referring back to something they discussed with you which you do not have memory of.",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY));


    //// Removed ITK targets
    // Made irrelevant by instruct list present in basically all contexts' system prompts. Can reintroduce if we find we
    //  want a more formal representation or rigid relationship between itk tool responses and LLM text output. Seems to
    //  handle information directly from the prompt fairly well though
    //definition = new ToolDefinition("getIfCanPerform",
    //        "Use this to ask the system whether you know how to perform the action represented by the supplied actionPredicate. actionPredicate should be assigned a populated instance of another tool call. An example of when you may need this information is when a user asks \"do you know how to escort a patient from %exampleLocation to the dock.\", which would map to getIfCanPerform(escortFromLocationToLocation(guest,%exampleLocation,dock:location))",
    //        List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "The populated tool call representing the action queried about", true)));
    //itkTools.add(new Tool(definition, UtteranceType.QUESTION));

    //// Present in prompt
    //definition = new ToolDefinition("reportInstructableActions",
    //        "You report the list of actions a user is able to instruct you through natural language to perform. An example of when you may need this information is when a user asks \"what can you be instructed to do.\"",
    //        new ArrayList());
    //itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    //TODO: make sure this distinction is possible to tease out from the LLM just from information in the prompt
    //definition = new ToolDefinition("getTaughtActions",
    //        "Use this to ask the system for a list of all of the actions which have been taught to you by a user. An example of when you may need this information is when a user asks \"What actions have you been taught.\"",
    //        new ArrayList());
    //itkTools.add(new Tool(definition, UtteranceType.QUESTION));
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
  private Map<String,Tool> generateReplyTools() {
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
    definition = new ToolDefinition("taskPredicate",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of another tool call. An example of instructing this could be \"the task is to go to %exampleLocation\", which would map to {\"name\":\"taskPredicate\", \"arguments\":{\"toolCall\":{\"name\":\"goToLocation\", \"arguments\":{\"destination\":\"%exampleLocation\"}}}}",
            List.of(new ToolParameterHelper("toolCall","string",new ArrayList<>(),"This argument should be replaced by definition of another tool call and populated arguments from the previous examples",true, true, "nlu")));
    replyTools.put("taskPredicate",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("taskPredicate(X)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition("relativeTime",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of a time relative to now. An example of instructing this could be \"in two hours\", which would map to {\"name\":\"relativeTime\", \"arguments\":{\"duration\":\"2\",\"timeunit\":\"HOURS\"}}",
            List.of(new ToolParameterHelper("duration","number",List.of(),"an integer, the duration of the specified time",true),
                    new ToolParameterHelper("timeUnit","string", Arrays.stream(TimeUnit.values()).map(TimeUnit::toString).toList(),"the duration's corresponding timeunit",true)
            ));
    replyTools.put("relativeTime",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("relativeTime(X,Y)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition("absoluteTime",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of an absolute time. An example of instructing this could be \"half past noon\", which would map to {\"name\":\"absoluteTime\", \"arguments\":{\"timeOfDay\":\"12:30\",\"partOfDay\":\"pm\"}}",
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