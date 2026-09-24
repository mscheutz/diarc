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
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
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
import edu.tufts.hrilab.llm.gson.tool.ToolFunction;
import edu.tufts.hrilab.llm.gson.tool.ToolParameterV2;
import edu.tufts.hrilab.llm.gson.tool.response.ToolCall;
import edu.tufts.hrilab.llm.gson.tool.response.ToolCallFunction;
import edu.tufts.hrilab.llm.openai.Qwen3CoderToolParser;
import edu.tufts.hrilab.llm.openai.response.OpenaiChatCompletionResponse;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.dialogue.Question;
import org.apache.commons.lang3.tuple.Pair;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Random;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

// Expansive higher level TODO list currently in personal notes
public abstract class SocialTRLLMServiceProvider extends LLMServiceProvider {
  HashMap<String, Qwen3CoderToolParser> streamingResults = new HashMap<>();
  private final java.util.Set<String> ttsStreamedIds = java.util.concurrent.ConcurrentHashMap.newKeySet();
  private volatile String currentStreamingResponseId;
  private volatile long currentStreamingUid = -1;

  // TODO: extend all context/chat/related collections to be indexed by groups of interactors, similar to conversations
  /**
   * Maintained set of ChatCollections, split by user/AgentTeam pair
   * Contains all relevant conversation contexts and their managed histories between each pair
   */
  protected final Map<Pair<Symbol, Symbol>, ChatCollection> currentChatCollections = new HashMap<>();

  /**
   * Extended History of chat contexts, stored for reference from ITK actions for the LLM to conditionally
   * retrieve information about past interactions without cluttering current chat context.
   */
  protected final Map<Pair<Symbol, Symbol>, List<ChatContext>> chatContextHistories = new HashMap<>();

  /**
   * Prompt variables to be replaced in sent Completion requests. Refer to the Prompt class and apply method if this
   * notion is unfamiliar to you.
   */
  //TODO: Refactor to align closer to how this method exists in the TRLLMServiceProvider
  //TODO: These are currently (re)populated incredibly inefficiently. Should largely maintain state and update on
  //      relevant event listeners.
  //TODO: At least elements of this should exist per user/AgentTeam pair - include parallel notion in ChatCollections for specific information
  protected Map<String, String> promptVars = new LinkedHashMap<>();
  //// Revisit when making base/subclass distinction
  protected abstract Map<String,String> initDomainPromptVars();
  protected abstract Map<String,String> updateDomainPromptVars(Utterance utterance, boolean parsing);

  /**
   * List of types handled by consultants in the system - when encountering symbols with these types during
   * NLG preprocessing/NLU postprocessing, replaces references with descriptors/typed descriptors with variables.
   */
  //TODO: pull from consultants automatically
  protected abstract List<String> getRefTypes();

  protected Prompt getNluSystemPrompt() { return Prompts.getPrompt("socialnlu");}
  protected Prompt getNlgSystemPrompt() { return Prompts.getPrompt("socialnlg");}
  protected Prompt getValidationNLUSystemPrompt() { return Prompts.getPrompt("socialvalidationnlu");}
  protected Prompt getValidationNLGSystemPrompt() { return Prompts.getPrompt("socialvalidation");}
  protected Prompt getQuestionAnswerSystemPrompt() {return Prompts.getPrompt("questionAnswer");}
  protected Prompt getGoalTranslationPrompt() {return Prompts.getPrompt("goalPredToString");}

  protected abstract List<Tool> generateInstructTools(boolean xml);
  protected abstract List<Tool> generateITKTools();
  protected abstract Map<String,Tool> generateReplyTools();
  protected abstract Map<String,ChatTemplates> generateFocusedContextChatTemplates();

  protected abstract String correctCommonNLUMistakes(Utterance utterance);

  /**
   * Collection of system prompts and Tool sets utilized in this domain. Ths set of prompts which must be defined are
   * as follows:
   * Required core, domain specific:
   * "nlu" - used when making calls from parseUtteranceLLMService
   * "nlg" - used when making calls from convertSemanticsToTextLLMService
   * "validationnlg" - used when making calls from convertSemanticsToTextLLMService while validating a parse result
   * "validationnlu" - used when making calls from parseUtteranceLLMService while validating a parse result
   * Available core, general:
   * "scheduleTask" - Social LLM alternative to templated turn-based scheduleGoal script
   * "learnTaskSequence" (though flavored differently in subclasses - variation on general action learning, learn sequence of action/location pairs
   * "modifyExistingtaskStatus" (currently just in SocialAva subcalls TODO - pull to base) - more flexible cancel/suspend/resuming of tasks
   *      TODO: reorderGoals, handleHigherPriorityConflictingGoal, modifyScheduledGoal, ...
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
  Map<String, ChatTemplates> chatTemplates;
  Map<String, Tool> universalFocusedToolset;
  List<ToolDefinition> universalFocusedToolDefinitions;

  /**
   * Explicitly holding REPLY Tool set separate because they will conditionally be added to outgoing parse requests as
   * targets based on current system questions (explicitly represented by the DialogueComponent)
   */
  Map<String, Tool> replyToolset;

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
   * 1. The call itself to the method leading to runContextSpecificChat/focusDialogieChat (e.g. scheduleTask)
   * 2. Needing to call the completion again under that specific context/ChatTemplate for the same input and handling
   * the result.
   * a) generateResponse - The system also responding to the instruction (prompting the user for necessary information, etc.)
   * (E.g. "I want to scheduled a task")
   * b) Underlying tool call - If enough information was supplied upfront, another action call may be made from the focused tool set
   * (E.g. "I want you to perform the introductory tour every hour starting in 20 minutes")
   * It's really hard to handle two temporally disjoint side effects from the same input with the current dialogue
   * pipeline, so this collection + a wrapper submission script associated with runContextSpecificChat is used to
   * circumvent the current limitations.
   */
  Map<Long, Utterance> contextTransferResponseUtterances = new HashMap<>();

  private boolean registeredForQuestionAskingNotifications = false;

  //Temporary list holding itk action names to detect in dialogue history callback whether UtteranceType Question
  //  was overwritten with INSTRUCT due to ITK / QueueEM incompatibility workarounds.
  List<String> itkActionNames = new ArrayList<>();

  private int externalErrorCount = 0;
  private boolean restrictedMode = false;

  @Override
  public void init() {
    super.init();
    chatTemplates = new HashMap<>();
    //TODO: apply promptVars to Tool definitions/examples and most importantly possible_values sets as well
    List<Tool> instructTools = generateInstructTools(true);
    List<Tool> itkTools = generateITKTools();
    replyToolset = generateReplyTools();

    for (Tool itkTool : itkTools) {
      itkActionNames.add(itkTool.toolDefinition.function.name);
    }

    //// Generate universal Tools ////
    Tool focusedEndConversationTool;
    Tool defaultGoalInfoTool;
    focusedEndConversationTool = new Tool(new ToolDefinition(new ToolFunction("endConversation",
            "End the conversation with the user and discard the current in progress definition.")),
            UtteranceType.INSTRUCT, Tool.PostCallBehavior.KEEP_HISTORY
    );
    defaultGoalInfoTool = new Tool(new ToolDefinition(new ToolFunction("getAssignedTasks",
            "Use this to ask the system for up to date information about your current list of tasks. Call this" +
                    "tool in order to get relevant information to help answer user questions about what you are currently" +
                    " doing, what you will be doing after your current task is complete, or what you are scheduled to" +
                    "perform in the future")),
            UtteranceType.QUESTION, Tool.PostCallBehavior.KEEP_HISTORY);
    itkActionNames.add("getAssignedTasks");

    textResponseTool = new Tool(
            new ToolDefinition(
                    new ToolFunction("stillGatheringInformation",
                            "You are still gathering information before being able to submit an accurate, fully defined " +
                                    "instruction and need to speak with the user to gather missing information.",
                                    new LinkedHashMap<>(
                                            Map.of(
                                                    "response",
                                                    new ToolParameterV2(
                                                            "string",
                                                            "What you want to say to the user"))),
                                    new String[]{"response"})
                    ),
            UtteranceType.INSTRUCT);

    itkTools.add(defaultGoalInfoTool);
    //// Gather Tool Sets and System Prompts ////
    // Base "nlu" tool set:
    //  - INSTRUCT targets
    //  - LLM<->Diarc ITK targets
    //  - (done dynamically at call-time) relevant REPLY targets
    List<Tool> baseNLUTools = new ArrayList<>(instructTools);
    baseNLUTools.addAll(itkTools);
    //baseNLUTools.add(defaultGoalInfoTool);
    baseNLUTools.add(textResponseTool);
    chatTemplates.put("nlu", new ChatTemplates(getNluSystemPrompt(), baseNLUTools));

    // Universal "focused" tool set:
    //  - Template specific targets (gathered at call-time based on entry corresponding to root action name - should be added by subclass)
    //  - Base LLM<->Diarc ITK targets
    //  - relevant REPLY targets - TODO: temporarily removed aside from during validation, should these be allowed? Should there be a mechanism to define REPLY targets per focused context?
    //  - suspend/cancel interaction targets
    universalFocusedToolset = new HashMap<>();
    universalFocusedToolDefinitions = new ArrayList<>();
    universalFocusedToolset.put("endConversation", focusedEndConversationTool);
    universalFocusedToolset.put("suspendCurrentInteraction", new Tool(new ToolDefinition(new ToolFunction("suspendCurrentInteraction",
            "Pause your current task, which can be later resumed.")),
            UtteranceType.INSTRUCT, Tool.PostCallBehavior.KEEP_HISTORY));
    universalFocusedToolset.put("cancelCurrentInteraction", new Tool(new ToolDefinition(new ToolFunction("cancelCurrentInteraction",
            "Stop and fully discard your current task.")),
            UtteranceType.INSTRUCT, Tool.PostCallBehavior.KEEP_HISTORY));
    for (Tool itkTool : itkTools) {
      universalFocusedToolset.put(itkTool.toolDefinition.function.name, itkTool);
    }
    for (Tool universalTool : universalFocusedToolset.values()) {
      universalFocusedToolDefinitions.add(universalTool.toolDefinition);
    }

    // Validation tool set:
    //  - Just reply forms, which are added dynamically (yes,no,abort)
    //  - ITK
    chatTemplates.put("validationnlu", new ChatTemplates(getValidationNLUSystemPrompt(), itkTools));

    //TODO: Only conditionally add cancel/suspend if there is actually an underlying task present?
    List<Tool> questionAnswerToolList = new ArrayList<>(itkTools);
    questionAnswerToolList.add(new Tool(new ToolDefinition(new ToolFunction("cancelCurrentTask",
            "You cancel and fully discard your current goal, whether it is active or suspended.")),UtteranceType.INSTRUCT));
    questionAnswerToolList.add(new Tool(new ToolDefinition(new ToolFunction("suspendCurrentTask",
            "You pause your current goal, which can be later resumed or canceled")),UtteranceType.INSTRUCT));
    chatTemplates.put("questionAnswer", new ChatTemplates(getQuestionAnswerSystemPrompt(), questionAnswerToolList));

    // NLG tool sets: none
    chatTemplates.put("nlg", new ChatTemplates(getNlgSystemPrompt(), new ArrayList<>()));
    chatTemplates.put("validation", new ChatTemplates(getValidationNLGSystemPrompt(), new ArrayList<>()));

    // Focused context toolsets:
    chatTemplates.putAll(generateFocusedContextChatTemplates());

    // TODO: apply promptVars to Tool definitions/examples and most importantly possible_values sets as well
    // init prompt vars
    promptVars.putAll(initPromptVars());

    // Generating string representations of toolset as reference for NLG and scheduleTask (want to present them, but not actually supply them as targets)
    // TODO: move into initPromptVars
    instructTools = generateInstructTools(false);
    promptVars.put("%instructNLGList", instructTools.stream().map(t -> toolToDescString(t, false)).collect(Collectors.joining("\n#\n")));

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

  protected Map<String,String> initPromptVars() {
    Map<String, String> vars = new LinkedHashMap<>();
    vars.put("%baseRoleDescription", getBaseRoleDescription());
    vars.put("%baseContextualInformation", getBaseContextualInformation());
    vars.put("%baseResponseConstraints", getBaseResponseConstraints());
    vars.put("%baseResponseStyle", getBaseResponseStyle());
    vars.put("%focusedResponseConstraints", getFocusedResponseConstraints());
    vars.put("%focusedResponseStyle", getFocusedResponseStyle());
    vars.put("%focusedResponsePolicy", getFocusedResponsePolicy());
    vars.put("%focusedFinalInstruction", getFocusedFinalInstruction());
    vars.putAll(initDomainPromptVars());
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
    } else {
      StringBuilder sb = new StringBuilder();
      populateSemanticDefinitionsString(utterance.getSemantics(), sb);
      if (sb.isEmpty()) {
        promptVars.put("%NLGSemanticDefinitions","<none found>");
      } else {
        promptVars.put("%NLGSemanticDefinitions",sb.toString());
      }
    }

    ChatCollection chatCollection = getChatCollection(user, agent);
    if (chatCollection != null) {
      ChatContext chatContext = chatCollection.currentChat;
      if (chatContext != null) {
        if (chatContext.chatState == ChatState.VALIDATION) {
          if (parsing) {
            //TODO: include tool information, not parsed semantics
            Map<Variable, Symbol> varMap = new HashMap<>();
            for (Term t : chatContext.utteranceUpForValidation.getSupplementalSemantics()) {
              varMap.put((Variable) t.get(0), Factory.createSymbol(t.getName(), t.getType()));
            }
            promptVars.put("%toolUpForValidationDescription", replaceVarsWithDescriptors((Predicate) chatContext.utteranceUpForValidation.getSemantics(), varMap).toString());
          } else {
            // Sample validation question format - potentially based on previous filler phrase
            String format;
            if (chatContext.utteranceUpForValidation.getSemantics().getName().equals("scheduled")) {
              format = "I should";
            } else if (chatContext.utteranceUpForValidation.getSemantics().getName().equals("submitTaskSequenceDefinition") || chatContext.utteranceUpForValidation.getSemantics().getName().equals("startMission")) {
              format = "The task is";
            } else {
              //INSTRUCT
              int option = new Random().nextInt(6);
              switch (option) {
                case 0 -> format = "Would you like me to";
                case 1 -> format = "Should I";
                case 2 -> format = "Did you mean for me to";
                case 3 -> format = "Did I hear correctly that";
                case 4 -> format = "Are you saying I should";
                case 5 -> format = "Confirming I should";
                default -> format = "Confirming I should";
              }
              //REPLY
              //  Did you mean, was it, did I hear correctly
            }
            promptVars.put("%confirmationFormat", format);
            promptVars.put("%goals", getCurrentGoalsSummary(utterance.getAddressee()));
          }
        } else if (chatContext.chatType == ChatType.FOCUSED && parsing) {
          //TODO: should be done automatically and/or in subclass specifically. Easy to miss updating this
          if (chatContext.contextID.equals("scheduleTask")) {
            promptVars.put("%templateSubmissionTargetName", "submitScheduledTaskDefinition");
            promptVars.put("%templateArgumentName", "currentScheduledDefinition");
          } else if (chatContext.contextID.equals("learnTaskSequence")) {
            promptVars.put("%templateSubmissionTargetName", "submitTaskSequenceDefinition");
            promptVars.put("%templateArgumentName", "currentTaskSequenceDefinition");
          } else if (chatContext.contextID.equals("learnMission")) {
            promptVars.put("%templateSubmissionTargetName", "startMission");
            promptVars.put("%templateArgumentName", "currentMissionDefinition");
          } else if (chatContext.contextID.equals("modifyExistingTaskStatus")) {
            promptVars.put("%templateSubmissionTargetName", "submitTaskStatusModificationDefinition");
            promptVars.put("%templateArgumentName", "currentTaskStatusModificationDefinition");
          }
        }
      }
    }
    promptVars.put("%addressee", agent.getName());
    promptVars.putAll(updateDomainPromptVars(utterance,parsing));
    if (parsing) {
      promptVars.putAll(updateFocusedContextPromptVars(user, agent));
    }
  }

  //TODO: move to/overridable by subclass
  protected Map<String,String> updateFocusedContextPromptVars(Symbol user, Symbol agent) {
    Map<String,String> vars = new HashMap<>();
    ChatCollection chatCollection = getChatCollection(user, agent);
    if (chatCollection != null) {
      ChatContext chatContext = chatCollection.currentChat;
      if (chatContext != null && chatContext.chatType == ChatType.FOCUSED && chatContext.chatState != ChatState.VALIDATION) {
        String contextId = chatContext.contextID;
        if (contextId.equals("modifyExistingTaskStatus")) {
          vars.put("%goalSummaryString", getFullGoalsSummary(agent));
        }
      }
    }
    return vars;
  }
  protected Map<String,Tool> getContextDependentTools(Symbol agentTeam) {
    Map<String,Tool> contextDependentTools = new HashMap<>();
    List<Predicate> currentGoals = getCurrentGoals(agentTeam);
    if (currentGoals != null && !currentGoals.isEmpty()) {
      if (currentGoals.get(0).get(0).getName().equals("follow")) {
        Tool stopFollowTool = chatTemplates.get("nluextended").tools.get("stopFollowing");
        if (stopFollowTool != null) {
          contextDependentTools.put("stopFollowing",stopFollowTool);
        }
      }
    }
    return contextDependentTools;
  }

  private void populateSemanticDefinitionsString(Symbol semantics, StringBuilder sb) {
    if (semantics == null || !semantics.isPredicate()) {
      return;
    }

    Predicate pred = (Predicate)semantics;
    String predDefinition = "";
    switch (pred.getName()) {
      case "mean" ->
              predDefinition = "name: mean(X,Y)\ndescription: Represents a question asking X if they meant to instruct or specify Y\n.";
      case "attempting" ->
              predDefinition = "name: attempting(X,Y)\ndescription: Report that you are starting the task defined by Y\n";
      case "and" ->
              predDefinition = "name: and(X,Y,...)\ndescription: Conjunctive form signalling desired translation consists of translating X and Y and ...\n";
      case "taskPredicate" ->
              predDefinition = "name: taskPredicate(X)\ndescription: Specification of a task specified by X\n";
      case "timingOf" ->
              predDefinition = "name: timingOf(X)\ndescription: Specification of the type of timing detail X. interval references the amount of time between occurrence and execution references the time of a specific occurrence\n";
      case "abort" ->
              predDefinition = "name: abort(question)\ndescription: Conveyed intent to decline answering an asked question\n";
      case "taskName" ->
              predDefinition = "name: taskName(X)\ndescription: The name of a task to be learned, X\n";
      case "not" ->
              predDefinition = "name: not(X)\ndescription: Negation of the fact X\n";
      case "ack" ->
              predDefinition = "name: ack(X)\ndescription: Format as \"<acknowledgement_word_or_phrase_variation>,<translation_of_X>\".\n";
      case "error" ->
              predDefinition = "name: error(X)\ndescription: Signals X should be conveyed in the form of a failure message\n";
      case "kioskGoal" ->
        predDefinition = "name: kioskGoal(X,Y)\ndescription: Specification of a 'kiosk goal' (a goal which occurs automatically after a period of inactivity) configuration. X is a task to be performed after Y seconds of inactivity\n";
      default -> {
        //TODO: account for conversions in correctCommonNLUMistakes
        Tool correspondingTool = chatTemplates.get("nlu").tools.get(pred.getName());
        if (correspondingTool == null) {
          correspondingTool = chatTemplates.get("nluextended").tools.get(pred.getName());
        }
        if (correspondingTool != null) {
          predDefinition = toolToDescString(correspondingTool, false);
        }
      }
    }

    if (!predDefinition.isEmpty()) {
      sb.append(predDefinition).append("\n##");
    }

    if (pred.hasArgs()) {
      for (Symbol arg : pred.getArgs()) {
        populateSemanticDefinitionsString(arg, sb);
      }
    }
  }

  /// ////////////////////////////////
  /// / NLU methods and utilities ////
  /// ////////////////////////////////
  public Utterance parseUtteranceLLMService(Utterance utterance) {
    return parseUtteranceLLMService(utterance, false, false);
  }

  public Utterance parseUtteranceLLMService(Utterance utterance, boolean useStreaming) {
    return parseUtteranceLLMService(utterance, useStreaming, false);
  }

  /**
   * Main NLU method used by LLMParserComponent
   *
   * @param utterance        Utterance with populated words to parse to semantics
   * @param reattemptedParse Whether this is a followup call after negative user validation (so we do not update the
   *                         Chat history with the same utterance more than once)
   * @return The utterance with populated semantics, or null if no valid parse is found
   */
  public Utterance parseUtteranceLLMService(Utterance utterance, boolean useStreaming, boolean reattemptedParse) {
    //// Get existing chat collections or create new if none exists
    ChatCollection chatCollection;
    ChatContext chatContext;
    Chat chat;
    Symbol user = getUntypedSymbol(utterance.getSpeaker());
    Symbol agentTeam = getUntypedSymbol(utterance.getAddressee());
    synchronized (currentChatCollections) {
      chatCollection = getChatCollection(user, agentTeam);
      if (chatCollection == null) {
        chatCollection = new ChatCollection(new Chat(user.getName(), agentTeam.getName()));
        currentChatCollections.put(Pair.of(user.toUntyped(), agentTeam.toUntyped()), chatCollection);
      }
    }

    // New: Synchronization - need to wait for any previous chat inputs and corresponding side effects to finish being
    // processed before applying this one
    if (!reattemptedParse) {
      if (!chatCollection.grabInputLock(utterance, true)) {
        // Dialogue history update showed that external parser completed before reaching this point. Do not need to proceed.
        return utterance;
      }
    }

    // Prepare Chat to send to completion endpoint: sort out system message, tool list, and any desired contextual history manipulation
    try {
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
          chat = new Chat(user.getName(), agentTeam.getName());
          chatContext = new ChatContext(chat);
          chatCollection.currentChat = chatContext;
        }

        //// Update chats based on current state
        ChatTemplates chatTemplates = this.chatTemplates.get(chatContext.contextID);
        if (chatTemplates == null) {
          log.error("[parseUtteranceLLMService] could not find chat template for contextID {}", chatContext.contextID);
          chatCollection.signalInputProcessed(utterance);
          return utterance;
        }
        Utterance utt = reattemptedParse ? null : utterance;
        ChatToolList chatToolList = prepChatAndToolList(chatContext, agentTeam, user, utt);
        chat = chatToolList.chat;
        tools = chatToolList.tools;
      }

      //// Send created chat to LLM completion
      Completion response;
      if (useStreaming) {
        response = callChatCompletionStreaming(chat, tools, utterance.getUid());
      } else {
        response = callChatCompletion(chat, tools);
      }

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
          output = handleTextResponse(response, utterance, chatContext, errorResponseBuilder, useStreaming);
        } else {
          if (chatContext.chatState == ChatState.VALIDATION) {
            // Validation Tool Call - return appropriate reply semantics and modify chatCollection state as needed (possible
            //  to terminate and/or transfer between base and focused contexts in this case)
            output = handleValidationResponse(response, utterance, chatCollection, tools, errorResponseBuilder, useStreaming);
          } else {
            // Tool Call - extended parse recovery on invalid parse, or enter validation of valid parse
            output = handleToolResponse(response, utterance, chatCollection, tools, errorResponseBuilder, useStreaming);
          }
        }

        if (output == null) {
          output = runCompletionRetryLoop(response, tools, utterance, chatCollection, errorResponseBuilder, useStreaming, chat);
        }
        return output;
      }
    } catch (RuntimeException e) {
      // the dialogue history update is what normally releases the lock we took above, and
      // it never runs if we throw out of here, so later utterances would just sit waiting
      log.error("[parseUtteranceLLMService] failed while processing {}", utterance, e);
      if (!reattemptedParse) {
        chatCollection.signalInputProcessed(utterance);
      }
      throw e;
    }
  }

  private record ChatToolList(Chat chat, List<ToolDefinition> tools) {};

  private ChatToolList prepChatAndToolList(ChatContext chatContext, Symbol agentTeam, Symbol user) {
    return prepChatAndToolList(chatContext, agentTeam, user, null);
  }

  private ChatToolList prepChatAndToolList(ChatContext chatContext, Symbol agentTeam, Symbol user, Utterance utterance) {
    List<ToolDefinition> tools;
    Chat chat;
    Prompt systemTemplate;
    ChatTemplates chatTemplates = this.chatTemplates.get(chatContext.contextID);
    if (chatTemplates == null) {
      log.error("[prepChatAndToolList] chatContext's stored contextID does not relate to an existing chatTemplate, using default nlu set");
      chatTemplates = this.chatTemplates.get("nlu");
    }

    // Higher level interaction decision: Question asking (including validation) strongly gates response instructions/possibilities
    boolean hasQuestions = false;
    try {
      hasQuestions = TRADE.getAvailableService(new TRADEServiceConstraints().name("isAgentWaitingForResponse").argTypes(Symbol.class)).call(Boolean.class, agentTeam);
    } catch (TRADEException e) {
      log.error("[parseUtteranceLLMService] unable to query agents QA status");
    }
    // hasQuestions should always be true if validation is, make sure this is the case
    // 9/10 (can remove if never seeing this log for a sufficient amount of time)
    if (chatContext.chatState == ChatState.VALIDATION && !hasQuestions) {
      log.error("[parseUtteranceLLMService] mismatched states - in VALIDATION but agentTeam has no system questions");
    }

    // Pull correct template and update system message and tool targets based off current state
    if (hasQuestions) {
      // Question asking has chat history as part of system message. Otherwise, content in the history will be parsed as responses.
      //  Creating a new chat just for QA, but maintaining proper history in Chat stored in this ChatContext in case of continued interaction
      chat = new Chat(user.getName(), agentTeam.getName());
      int currentHistorySize = chatContext.history.messages.size();
      int historySizeBeforeQA = chatContext.onQuestionAskedHistorySize;
      if (historySizeBeforeQA == -1) {
        chatContext.onQuestionAskedHistorySize = currentHistorySize;
        historySizeBeforeQA = currentHistorySize;
        // (Make sure this entry never gets erased/overwritten and doesn't exist when it should)
        promptVars.put("%QAChatHistoryAsString", getChatHistoryAsString(new ArrayList<>(chatContext.history.messages), historySizeBeforeQA));
      }
      if (historySizeBeforeQA > currentHistorySize) {
        log.warn("[prepChatAndToolList] onQuestionAskedHistorySize somehow larger than current history size, not updated properly somewhere");
        chatContext.onQuestionAskedHistorySize = 0;
        historySizeBeforeQA = 0;
      }
      chat.messages = new ArrayList<>(chatContext.history.messages).subList(historySizeBeforeQA,currentHistorySize);

      // Pull correct text response tool first based on whether in focused context with associated template or not
      tools = new ArrayList<>();
      Tool focusedTextResponseTool = chatTemplates.tools.get(textResponseTool.toolDefinition.function.name);
      if (focusedTextResponseTool != null) {
        tools.add(focusedTextResponseTool.toolDefinition);
      } else {
        tools.add(textResponseTool.toolDefinition);
      }

      // Add appropriate QA tool list
      if (chatContext.chatState == ChatState.VALIDATION) {
        chatTemplates = this.chatTemplates.get("validationnlu");
      } else {
        chatTemplates = this.chatTemplates.get("questionAnswer");
      }
      tools.addAll(chatTemplates.toolDefinitions);
      systemTemplate = new Prompt(chatTemplates.systemMessageTemplate.getText());
      // Add relevant response form tools based on question
      tools.addAll(getQuestionResponseToolDefinitions(agentTeam));

      // Also add message to actual history in case the conversation extends
      if (utterance != null) {
        chatContext.history.addUserMessage(utterance.getWordsAsString());
        chat.addUserMessage(utterance.getWordsAsString());
        updatePromptVars(utterance, true);
      }

      // Update and apply prompt vars
      String systemMessage = systemTemplate.apply(promptVars);
      chat.setSystem(systemMessage);
      // Also update general context's system message - should be reverted to non QA state when relevant
      chatContext.history.setSystem(systemMessage);
    } else {
      systemTemplate = new Prompt(chatTemplates.systemMessageTemplate.getText());
      tools = new ArrayList<>(chatTemplates.toolDefinitions);
      chat = chatContext.history;

      // Experimental, allow advertisement of certain tools based on current context
      tools.addAll(getContextDependentTools(agentTeam).values().stream().map(t -> t.toolDefinition).toList());

      if (chatContext.chatType == ChatType.FOCUSED) {
        tools.addAll(universalFocusedToolDefinitions);
      }

      // Apply input as user message
      if (utterance != null) {
        chat.addUserMessage(utterance.getWordsAsString());
        updatePromptVars(utterance, true);
      }

      // Update and apply prompt vars
      chat.setSystem(systemTemplate.apply(promptVars));
    }
    return new ChatToolList(chat, tools);
  }

  private Utterance runCompletionRetryLoop(Completion lastResponse, List<ToolDefinition> toolList, Utterance utterance, ChatCollection chatCollection, StringBuilder errorResponseBuilder, boolean useStreaming) {
    return runCompletionRetryLoop(lastResponse, toolList, utterance, chatCollection, errorResponseBuilder, useStreaming, null);
  }

  //TODO: revisit - only add calls to this if in synchronized block and understand what you're doing
  private Utterance runCompletionRetryLoop(Completion lastResponse, List<ToolDefinition> toolList, Utterance utterance, ChatCollection chatCollection, StringBuilder errorResponseBuilder, boolean useStreaming, Chat completionChat) {
    Symbol user = utterance.getAddressee().toUntyped();
    Symbol agentTeam = utterance.getSpeaker().toUntyped();
    ChatContext chatContext = chatCollection.currentChat;
    if (completionChat == null) {
      completionChat = chatContext.history;
    }
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
          //TODO: also mirror message in context history, or should these failures be hidden?
          completionChat.addRobotMessage("", List.of(lastToolCall));
        }
        String errorString = errorResponseBuilder.toString();
        log.info("[runCompletionRetryLoop] running extra chatCompletion request with additional error message: {}", errorString);
        tool_call_id = lastToolCall == null ? "" : lastToolCall.id;
        //TODO: also mirror message in context history, or should these failures be hidden?
        completionChat.addToolMessage(errorString, tool_call_id);
      }
      errorResponseBuilder = new StringBuilder();

      // Re-call completion for text response
      Completion retryResponse;
      if (useStreaming) {
        retryResponse = callChatCompletionStreaming(completionChat, toolList, utterance.getUid());
      } else {
        retryResponse = callChatCompletion(completionChat, toolList);
      }

      if (retryResponse == null) {
        handleErrorResponse(utterance, true, false);
        return utterance;
      } else {
        resetErrorCount(false, user, agentTeam);
        if (retryResponse.toolCalls.isEmpty()) {
          lastToolCall = null;
          resetErrorCount(true, utterance.getSpeaker().toUntyped(), utterance.getAddressee().toUntyped());
          output = handleTextResponse(retryResponse, utterance, chatContext, errorResponseBuilder, useStreaming);
        } else {
          lastToolCall = retryResponse.toolCalls.get(0);
          output = handleToolResponse(retryResponse, utterance, chatCollection, toolList, errorResponseBuilder, useStreaming);
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
   * - Base: Return an Utterance populated with the Tool's underlying semantics. In the case of an external cache hit,
   * the context will be discarded and handleValidatedToolResponse will be called from the corresponding dialogue history update.
   * - LLM ITK - will return semantics for goal without validation (Underlying semantics should match a SKIPPENDING priority goal which calls {@link #addToolResponse(Symbol, Symbol, String, String)})
   * - Invalid parse - Add a system error message to the Chat, call the completion again, and return a text response to the user with the result
   * - *NEW* Stand-in tool(s) for text responses - Experimenting targeting a specific tool to give text responses rather than using
   * handleTextResponse above with promising initial success. Allowing the LLM to target text responses or tool calls was giving it a
   * hard time calling tools rather than just saying it performed the underlying functionality.
   */
  //public Utterance handleToolResponse(Completion llmResponse, Utterance inputUtterance, ChatCollection chatCollection, StringBuilder errorResponseBuilder) {
  //  return handleToolResponse(llmResponse,inputUtterance,chatCollection,errorResponseBuilder,false);
  //}
  public Utterance handleToolResponse(Completion llmResponse, Utterance inputUtterance, ChatCollection chatCollection, List<ToolDefinition> advertisedToolList, StringBuilder errorResponseBuilder, boolean useStreaming) {
    if (llmResponse.toolCalls.size() > 1) {
      //TODO
      log.warn("[handleToolResponse] Multiple tool calls returned -> TODO handle both. Only using first...");
    }
    ToolCall toolCall = llmResponse.toolCalls.get(0);
    for (ToolCall tc : llmResponse.toolCalls) {
      log.info("[handleToolResponse] " + tc);
    }

    ChatContext chatContext = chatCollection.currentChat;

    //// Check that tool response is valid
    ToolCallFunction functionCall = toolCall.function;
    Tool correspondingTool = null;
    try {
      if (advertisedToolList.stream().filter(td -> td.function.name.equals(functionCall.name)).toList().isEmpty()) {
        log.warn("[handleToolResponse] called tool not present in advertised list, rejecting");
      } else {
        if (chatContext.chatType == ChatType.FOCUSED) {
          correspondingTool = chatTemplates.get(chatContext.contextID).tools.get(functionCall.name);
          if (correspondingTool == null) {
            correspondingTool = universalFocusedToolset.get(functionCall.name);
          }
        } else {
          correspondingTool = chatTemplates.get("nlu").tools.get(functionCall.name);
          if (correspondingTool == null) {
            correspondingTool = replyToolset.get(functionCall.name);
          }
        }
        //TODO: more formally define and integrate how this is done. Don't want this inefficient block here
        if (correspondingTool == null) {
          correspondingTool = getContextDependentTools(inputUtterance.getAddressee()).get(functionCall.name);
        }
      }
    } catch (NullPointerException e) {
      log.error("[handleToolResponse] error finding Tool definition for called tool found in Completion response", e);
    }

    // Internal LLM error - should there be LLM specific error responses in this case?
    if (correspondingTool == null) {
      errorResponseBuilder.append("- Your chosen tool \"").append(toolCall.function.name).append("\" is no longer available for you to call in your current context. Please find an alternate specific tool call to use which is explicitly present in the tool list, or respond using the ").append(textResponseTool.toolDefinition.function.name).append(" tool");
      if (!toolCall.function.name.equals(textResponseTool.toolDefinition.function.name)) {
        chatContext.history.addRobotMessage("", List.of(toolCall));
      }
      return null;
    }

    if (errorResponseBuilder.isEmpty()) {
//      errorResponseBuilder.append("Your most recent tool call with id ").append(toolCall.id).append(" contains missing or invalid information. Please correct the following errors to supply" +
//              " valid json to make the desired tool call. Solve these issues on your own where possible. If you must ask" +
//              " the user for missing or clarifying information, precede your question by saying that your interpretation" +
//              " of the instruction contained invalid information:\n");
      errorResponseBuilder.append("Your most recent tool call with id ").append(toolCall.id).append(" was rejected by the system. " +
              "Please address the following reasons why and make an updated tool call. Solve these issues on your own where possible. If you must ask" +
              " the user for missing or clarifying information, let them know why:\n");
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

    //(Moved to after correctCommonNLUMistakes are applied - This is desired for NLG, does it affect parsing negatively?)
    //Experimental - adding tool calls back into history now that we have safeguards against them being called from
    // incorrect contexts when no longer available. Text response tool calls will be added from the dialogue history listener.
    if (!toolCall.function.name.equals(textResponseTool.toolDefinition.function.name)) {
      //TODO: make sure it is okay to not include text response calls here. Believe the only time it
      //    really matters would be on malformed/invalid target.
      chatContext.history.addRobotMessage("", List.of(toolCall));
    }

    //// Handle processed response
    if (!error) {
      //Valid parse - proceed to validation

      // Experimental text response tool stand-in (described above)
      if (toolCall.function.name.equals(textResponseTool.toolDefinition.function.name)) {
        if (llmResponse.text.isEmpty()) {
          if (!outputUtterance.getSemantics().isPredicate() || ((Predicate) outputUtterance.getSemantics()).getArgs().size() < 2) {
            // Is this case even possible anymore?? Don't think so. Think would have to be developer error, otherwise would
            // have been caught in tool postprocessing.
            log.error("[handleToolResponse] malformed text response tool call **Believe this must be due to Developer Error**");
            errorResponseBuilder.append("- your " + textResponseTool.toolDefinition.function.name + " tool call needs to be properly formatted json including a response field");
            return null;
          }
          llmResponse.text = ((Predicate) outputUtterance.getSemantics()).get(1).toString();
          if (llmResponse.text.startsWith("\"") && llmResponse.text.endsWith("\"")) {
            llmResponse.text = llmResponse.text.substring(1, llmResponse.text.length() - 1);
          }
        }
        return handleTextResponse(llmResponse, inputUtterance, chatContext, errorResponseBuilder, useStreaming);
      }

      //// Universal postprocessing for accepted cases
      // TODO: Reevaluate if these are in fact conventions we want to universally accept / how we want to do these things
      String toolName = outputUtterance.getSemantics().getName();
      chatContext.lastToolCallName = toolName;
      chatContext.identicalResponses = 0;

      // Temporary special wrappers - revisit if ever reintroducing "offContext" functionality
      if (chatContext.chatType != ChatType.BASE) {
        if (toolName.equals("endConversation") || toolName.equals("cancelCurrentInteraction")) {
          outputUtterance.setSemantics(Factory.createPredicate("cancelCurrentTask", inputUtterance.getAddressee()));
        } else if (toolName.equals("suspendCurrentInteraction")) {
          outputUtterance.setSemantics(Factory.createPredicate("suspendCurrentTask", inputUtterance.getAddressee()));
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
        boolean ignoreVal = false;
        try {
          ignoreVal = TRADE.getAvailableService(new TRADEServiceConstraints().name("ignoreValidation").argTypes(Predicate.class)).call(Boolean.class, outputUtterance.getSemantics());
        } catch (TRADEException ignored){};
        if (outputUtterance.needsValidation() && !ignoreVal) {
          chatContext.chatState = ChatState.VALIDATION;
          chatContext.utteranceUpForValidation = outputUtterance; //copy?
          chatContext.toolCallIdUpForValidation = toolCall.id;
        } else {
          // noConfirmation case - deferred until dialogue history update in case of external parser hit
          //handleValidatedToolCall(outputUtterance, chatCollection);
          outputUtterance.setNeedsValidation(false);
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
   * and return generateResponseFromString semantics with needsValidation set to false
   */
  //public Utterance handleTextResponse(Completion llmResponse, Utterance inputUtterance, ChatContext chatContext, StringBuilder errorResponseBuilder) {
  //  return handleTextResponse(llmResponse,inputUtterance,chatContext,errorResponseBuilder,streamResults);
  //}
  public Utterance handleTextResponse(Completion llmResponse, Utterance inputUtterance, ChatContext chatContext, StringBuilder errorResponseBuilder, boolean streaming) {
    String responseString = llmResponse.getText().trim();
    if (responseString.isEmpty()) {
      responseString = llmResponse.reasoning.trim();
    }

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

    //// Handle simplest case of reasoning loops - consecutive identical responses (practically seen to be able to get stuck in)
    // TODO: should common phrases like "yes", "no", "and then what", etc. be explicitly excluded? - The LLM doesn't seem to (at least currently) really do this unless stuck though
    // Detect consecutive text responses and attempt to break out if finding too many
    //TODO: This does not interact with streaming responses well. Would have to interrupt sayText or be an explicit
    //  extra turn afterwards. Cannot just modify and send out response here.
    // a regenerated response never went through the stream, so it must be spoken below
    boolean regenerated = false;
    // drained here so the retry return below cannot leak the id in the set
    boolean responseStreamed = ttsStreamedIds.remove(llmResponse.getId());
    chatContext.lastResponseStreamed = responseStreamed;
    if (chatContext.lastResponseString != null && chatContext.lastResponseString.equals(responseString)) {
      chatContext.identicalResponses += 1;
      if (chatContext.chatType == ChatType.FOCUSED && chatContext.identicalResponses >= 2) {
        log.warn("[handleTextResponse] (FOCUSED) Encountering consecutive identical responses - retaining template but clearing history and informing the user");
        // the identical reply already streamed and may still be playing; drop it before
        // speaking the replacement over it
        ttsCancel(inputUtterance.getUid());
        // Focused contexts - retain current template, clear history, and reprompt with custom message based on template state
        //  (Could also try less drastic clearing of history first like for base context, but I'm worried that it will
        //    result in a different phrasing of a response while still not breaking out of the same reasoning loop)
        chatContext.history.messages.clear();
        chatContext.onQuestionAskedHistorySize = 0;
        if (!chatContext.lastTemplateName.isEmpty() && chatContext.lastTemplateValue != null) {
          chatContext.history.addMessage("user",""); //Safe? Seems to be okay
          chatContext.history.addMessage("tool", "You have already had an extended conversation with the user, but got confused and needed to clear some history. The progress of your previous conversation which you were able to retain is" + chatContext.lastTemplateName + ": " + chatContext.lastTemplateValue + ". Please respond to the user by saying \"Sorry, I got confused and had to clear some of my history.\", then let the user know what you remember based on the current definition you retained (in natural language), and finally continue with your task as instructed.");
          responseString = callChatCompletionForTextResponse(Factory.createSymbol(chatContext.history.userName), Factory.createSymbol(chatContext.history.robotName), chatContext.cid);
        } else {
          responseString = "Sorry, I got confused and had to clear some of my history, can we start from the top please?";
        }
        regenerated = true;
        chatContext.identicalResponses = 0;
      } else if (chatContext.chatType != ChatType.FOCUSED && chatContext.identicalResponses == 2) {
        log.warn("[handleTextResponse] detected multiple consecutive identical responses, clearing history of those responses and trying again");
        // 3rd identical response - modify recent history to try to pull out of loop. Keep last input message and respond again based off that.
        //  (TODO: If streaming, then need to signal the purpose of this extra response turn or at least outwardly relate the responses - or wait to start responding on TTS side if n-1 identical responses already encountered and start of response matches so far)
        List<Message> history = chatContext.history.messages;
        int responsesFound = 0;
        // Remove messages until all duplicate responses found
        for (int i = history.size()-2; i >= 0; i--) {
          Message m = history.remove(i);
          if (m.tool_calls != null && m.tool_calls.length > 0) {
            ToolCallFunction tf = m.tool_calls[0].function;
            if (tf.name.equals(textResponseTool.toolDefinition.function.name) && tf.arguments.contains(responseString)) {
              responsesFound += 1;
              if (responsesFound == 2) {
                break;
              }
            }
          }
        }
        // Remove any lingering user messages
        for (int i = history.size()-2; i >= 0; i--) {
          if (history.get(i).role.equals("assistant")) {
            break;
          }
          history.remove(i);
        }
        // Last user input should still be present, submit for a new (hopefully different) generated response.
        // Not streaming the retry: we already streamed and finished under this uid once, and
        // when the retry streamed too it looked like it got recorded as spoken without
        // anything actually playing. Might be able to stream it again if a push ever says
        // which request it came from.
        ttsCancel(inputUtterance.getUid());
        return parseUtteranceLLMService(inputUtterance, false, true);
      } else if (chatContext.chatType != ChatType.FOCUSED && chatContext.identicalResponses > 2) {
        log.warn("[handleTextResponse] Still encountering identical responses, clearing history fully and informing user");
        ttsCancel(inputUtterance.getUid());
        // 4th identical response - inform user and more drastically reset context
        chatContext.history.messages.clear();
        chatContext.onQuestionAskedHistorySize = 0;
        responseString = "Sorry, I got confused and had to clear some of my history, can we start from the top please?";
        regenerated = true;
        chatContext.identicalResponses = 0;
      }
    } else {
      chatContext.identicalResponses = 0;
    }
    chatContext.lastResponseString = responseString;
    if (llmResponse.toolCalls != null && !llmResponse.toolCalls.isEmpty()) {
      chatContext.lastResponseToolCallId = llmResponse.toolCalls.get(0).id;
    }

    // newlines, backslashes, or nested quotes in the reply were blowing up the belief
    // asserts that read this text back out of a term -- nothing escapes it on the way in.
    // Swapping those characters out here is a hack for the breakage I was seeing.
    // TODO: this almost certainly belongs where the term gets written, not here
    responseString = responseString
            .replace('\n', ' ')
            .replace('\r', ' ')
            .replace('\t', ' ')
            .replace('\\', ' ')
            .trim();
    if (responseString.length() > 1 && responseString.startsWith("\"") && responseString.endsWith("\"")) {
      responseString = responseString.substring(1, responseString.length() - 1);
    }
    responseString = "\"" + responseString.replace('"', '\'') + "\"";

    // Updating response from dialogue history now - in case this parse result is not used
    //chatContext.history.addRobotMessage(responseString, llmResponse.toolCalls);
    Predicate generateResponsePred;
    // a streamed request whose fragments never reached the TTS still has to be spoken here
    if (streaming && responseStreamed && !regenerated) {
      generateResponsePred = Factory.createPredicate("updateDialogueHistoryFromString", inputUtterance.getAddressee().toString(), inputUtterance.getSpeaker().toString(), responseString);
    } else {
      generateResponsePred = Factory.createPredicate("generateResponseFromString", inputUtterance.getAddressee().toString(), inputUtterance.getSpeaker().toString(), responseString);
    }
    inputUtterance.setSemantics(generateResponsePred);
    inputUtterance.setType(UtteranceType.INSTRUCT);
    inputUtterance.setNeedsValidation(false);
    return inputUtterance;
  }

  // (Don't need to do any disjoint updates/checking for external cache hits because assuming we are already using LLM only by this point)

  /**
   * Method for handling Completion responses containing during validation.
   * <p>
   */
  public Utterance handleValidationResponse(Completion llmResponse, Utterance inputUtterance, ChatCollection chatCollection, List<ToolDefinition> advertisedToolList, StringBuilder errorResponseBuilder, boolean useStreaming) {
    if (llmResponse.toolCalls.size() > 1) {
      log.warn("[handleValidationResponse] Multiple tool calls returned -> TODO handle both. Currently only using first...");
    }
    ToolCall toolCall = llmResponse.toolCalls.get(0);
    for (ToolCall tc : llmResponse.toolCalls) {
      log.info("[handleValidationResponse] " + tc);
    }

    // Tool -> semantics. Yes/No/Abort
    //  Context specific prompt - no special cases. Set utterance semantics and terminate chat if validated or aborted
    //  Base prompt - special cases based on whether context specific prompt exists or not
    Utterance.Builder output = new Utterance.Builder(inputUtterance);
    ChatContext chatContext = chatCollection.currentChat;

    // Experimental text response tool stand-in (described above)
    if (toolCall.function.name.equals(textResponseTool.toolDefinition.function.name)) {
      return handleToolResponse(llmResponse, inputUtterance, chatCollection, advertisedToolList, errorResponseBuilder, useStreaming);
    }

    // Handle expected validation question yes/no response semantics
    // (Associated logic / additional side effects present in question answer callback)
    if (toolCall.function.name.equals("val") || toolCall.function.name.equals("validated")) {
      String argument = toolCall.function.arguments.contains("yes") ? "yes" : "no";
      output.setSemantics(Factory.createPredicate("val", argument));
      output.setNeedsValidation(false);
      output.setUtteranceType(UtteranceType.REPLY);
    } else if (toolCall.function.name.equals("abort")) {
      // User declined to answer the validation question - terminate context entirely and refocus base parse state
      output.setSemantics(Factory.createPredicate("abort(question)"));
      output.setNeedsValidation(false);
      output.setUtteranceType(UtteranceType.REPLY);
    } else {
      // Be careful here if ever changing accepted/possible behavior around ITK calls or exposing additional tools in a validation context
      log.debug("[handleValidationResponse] received tool call which is not a validation response or text response, assuming ITK call");
      return handleToolResponse(llmResponse,inputUtterance,chatCollection,advertisedToolList,errorResponseBuilder,useStreaming);
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
        if (copy.history.messages.get(i).role.equals("user")) {
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
    Tool calledTool = null;
    // Check specific chat template for corresponding tool
    if (ct != null) {
      calledTool = ct.tools.get(toolName);
      if (calledTool == null) {
        calledTool = ct.tools.get(chatContext.lastToolCallName);
      }
    }
    // Check universal focused toolset for corresponding tool
    if (calledTool == null && chatContext.chatType == ChatType.FOCUSED) {
      calledTool = universalFocusedToolset.get(toolName);
      if (calledTool == null) {
        calledTool = universalFocusedToolset.get(chatContext.lastToolCallName);
      }
    }
    if (calledTool != null) {
      postCallBehavior = calledTool.postCallBehavior;
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
  public boolean inExtendedParseLoop(Symbol user, Symbol agentTeam) {
    if (restrictedMode) {
      return false;
    }

    ChatCollection collection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
    // Without this, there could be an issue here if any TLDL greeting, reply, ack responses are present in the history
    //  but the LLM is failing calls to the endpoint. Would be returning true, but because of population from a tldl call
    if (collection != null) {
      ChatContext chatContext = collection.currentChat;
      if (chatContext != null) {
        return (chatContext.containsLLMParse || chatContext.chatType != ChatType.BASE) && chatContext.chatState != ChatState.VALIDATION; // exclude error responses?
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

  @TRADEService
  public void reduceChatContextHistory() {
    synchronized (currentChatCollections) {
      //TODO: ideally need user/agent pair as an argument
      int maxLength = 0;
      ChatCollection mostLikelyOffender = null;
      for (Map.Entry<Pair<Symbol,Symbol>,ChatCollection> collectionPair : currentChatCollections.entrySet()) {
        int convoLength = getLLMConversationLength(collectionPair.getKey().getLeft(),collectionPair.getKey().getRight());
        if (mostLikelyOffender == null) {
          mostLikelyOffender = collectionPair.getValue();
          maxLength = convoLength;
        } else {
          if (convoLength > maxLength) {
            mostLikelyOffender = collectionPair.getValue();
            maxLength = convoLength;
          }
        }
      }
      //TODO: do something more principled here - need ability to internally measure token lengths of individual messages
      //      and then relate it to the configured max input size
      //Doubled with fixing conversation history ITK response size and persistence in history this should be okay.
      if (mostLikelyOffender != null) {
        while (maxLength > 10) {
          mostLikelyOffender.currentChat.history.messages.remove(0);
          if (mostLikelyOffender.currentChat.onQuestionAskedHistorySize > 0) {
            mostLikelyOffender.currentChat.onQuestionAskedHistorySize -= 1;
          }
          maxLength--;
        }
      }
    }
  }

  /**
   * Method called from nlu.asl after negative validation of an LLM parse. Adds a system message informing the LLM of
   * what occurred and calling the completion endpoint again with the user response. May result in an updated tool call
   * or a text response for the user.
   */
  @TRADEService
  public boolean reattemptParse(Utterance utterance, boolean streamingTTS) {
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
      Predicate generateResponsePred = Factory.createPredicate("generateResponseFromString", utterance.getAddressee().toString(), utterance.getSpeaker().toString(), "\"Sorry, an internal error has occurred. Could you repeat that?\"");
      utterance.setSemantics(generateResponsePred);
      utterance.setType(UtteranceType.INSTRUCT);
      utterance.setNeedsValidation(true);
      utterance.setIsValidated(false);
      return false;
    }

    //// Wait for relevant side effects from dialogue history and question answer callbacks to occur
    //currentContext.waitOnValidationResponseSideEffects();

    Symbol semantics = utterance.getSemantics().clone();
    if (semantics.isPredicate() && utterance.getSupplementalSemantics() != null) {
      ////Note: if including tool calls in the history, this message needs to include matching tool_call_id
      // Add tool response stating what was negatively validated by the user
      Map<Variable, Symbol> varMap = new HashMap<>();
      for (Term t : utterance.getSupplementalSemantics()) {
        varMap.put((Variable) t.get(0), Factory.createSymbol(t.getName(), t.getType()));
      }
      semantics = replaceVarsWithDescriptors((Predicate) semantics, varMap);
    }
    currentContext.history.addToolMessage("The user has just informed you that your previous interpretation of their command " +
            "which resulted in your tool call with id " + currentContext.toolCallIdUpForValidation + " represented by " + semantics + " was not what they intended to instruct. Use the last " +
            "user message to determine one of two ways to proceed: 1. If the user's message includes additional information clarifying " +
            "their original intention or adding additional information, then you may update your definition with the new information and attempt another updated tool submission call. Do not immediately make the same tool call with the same arguments. " +
            "2. Otherwise, please continue with your original task to resolve the misunderstanding.", currentContext.toolCallIdUpForValidation);

    // Call completion with user response and tool message. Built from the original utterance so
    // the reattempt streams under the same uid the ASL unbuffers; the rejected parse's semantics,
    // bindings, and descriptors are cleared so the corrected parse starts clean.
    Utterance tempUtterance = new Utterance.Builder(utterance)
            .setBindings(new ArrayList<>())
            .setSupplementalSemantics(new ArrayList<>())
            .setIndirectSemantics(new ArrayList<>())
            .setTierAssignments(new LinkedHashMap<>())
            .build();
    tempUtterance.setSemantics(null);
    tempUtterance.setNeedsValidation(true);
    // the negative validation reply's input claim is still held here (its history update
    // skips the signal), so it has to be released by identity once the reattempt is done
    Utterance claimed = chatCollection.utteranceBeingProcessed;
    try {
      tempUtterance = parseUtteranceLLMService(tempUtterance, streamingTTS, true);

      if (tempUtterance.getSemantics() == null) {
        log.warn("[reattemptParse] original utterance returned from inner parse method - indicates exception occurred");
        tempUtterance.setSemantics(Factory.createPredicate("generateResponseFromString", utterance.getAddressee().toString(), utterance.getSpeaker().toString(), "\"Sorry, an internal error has occurred. What did you mean to instruct me to do?\""));
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
    } finally {
      if (claimed != null) {
        chatCollection.signalInputProcessed(claimed);
      }
    }
    boolean ret = utterance.needsValidation();
    if (!ret) {
      // This is kind of weird, but we need to set isValidated to true even for things normally not needing validation
      //  (e.g. LLM text responses) so that outer loop in NLU.asl doesn't discard the utterance.
      utterance.setIsValidated(true);
    }
    return ret;
  }

  /// ////////////////////////////////
  /// / NLG methods and utilities ////
  /// ////////////////////////////////
  @Override
  public Utterance convertSemanticsToTextLLMService(Utterance utterance) {
    return convertSemanticsToTextLLMService(utterance, false);
  }

  /**
   * Main NLG method used by LLMNLGComponent
   *
   * @param utterance        Utterance with populated semantics to transform to words
   * @param streamingSayText - whether the response should be feed incrementally to TTS using a streaming Completion
   * @return The utterance with populated words, or null if no generation could be formed
   */
  @Override
  public Utterance convertSemanticsToTextLLMService(Utterance utterance, boolean streamingSayText) {
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
        promptVars.put("%chatHistoryAsString", getChatHistoryAsString(chatContext.history));
        if (chatContext.chatState == ChatState.VALIDATION) {
          chatTemplates = this.chatTemplates.get("validation");
        }
      }
    } else {
      promptVars.put("%chatHistoryAsString", "<There is no relevant chat history>");
    }
    promptVars.put("%user", user.getName());
    if (chatTemplates == null) {
      chatTemplates = this.chatTemplates.get("nlg");
    }

    semantics = preprocessNLGSemantics(semantics);

    // Supplying history in the system prompt itself as to not confuse the LLM - need to supply history for context, but
    //    the NLU history would contradict the stated task in this system prompt.
    // Create a new chat as to not destroy history
    Chat genChat = new Chat(user.getName(), agentTeam.getName());
    utterance = new Utterance.Builder(utterance).build();
    if (semantics.isPredicate()) {
      semantics = replaceReferencesInSemantics((Predicate) semantics);
    }
    utterance.setSemantics(semantics);
    genChat.addUserMessage(utterance.toString());
    genChat.setSystem(chatTemplates.systemMessageTemplate.apply(promptVars));

    Completion llmResponse;
    if (streamingSayText) {
      llmResponse = callChatCompletionStreaming(genChat, null, utterance.getUid());
    } else {
      llmResponse = callChatCompletion(genChat, null);
    }

    if (llmResponse == null) {
      log.error("[convertSemanticsToTextLLMService] received null completion response");
      handleErrorResponse(utterance, false, false);
      return utterance;
    } else {
      // handleTextResponse takes these back out again on the nlu side, but I did not find
      // anything doing it over here, so take it out so the set does not just keep growing
      ttsStreamedIds.remove(llmResponse.getId());
      resetErrorCount(false, user, agentTeam);
    }

    log.info(llmResponse.getText());
    String responseString = llmResponse.getText().trim();
    if (responseString.isEmpty()) {
      responseString = llmResponse.reasoning.trim();
    }

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
            String updatedString = inputString.replaceAll(patternString, command + "(" + actor + "," + goalPred.toString() + ")");
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
          updateFocusedContextPromptVars(user, agentTeam);
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
          } else {
            focusedTextResponseTool = textResponseTool;
          }
          chat.addUserMessage("The previous conversation history leading up to this point is as follows:\n" + chatHistoryAsString + "\nLook for any relevant information from this conversation history to inform your initial response.");
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
            Completion response;
            //if (streamResults) {
            //  // Currently not including universal focused tools here (i.e. suspend/cancel and itk targets). Is this in fact correct?
            //  response = callChatCompletionStreaming(chat, contextSpecificTemplate.toolDefinitions);
            //} else {
              response = callChatCompletion(chat, contextSpecificTemplate.toolDefinitions);
            //}

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
              output = handleTextResponse(response, originalUtterance, collection.currentChat, errorResponseBuilder, false);
            } else {
              // Direct tool call on transfer
              lastToolCall = response.toolCalls.get(0);
              output = handleToolResponse(response, originalUtterance, collection,contextSpecificTemplate.toolDefinitions, errorResponseBuilder, false);
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
   *
   * @param user      external interactor who is a part of this conversation
   * @param agentTeam DIARC AgentTeam which is a part of this conversation
   * @param cid       unique identifier for this focused chat context ("context ID" as in DIARC Context class ID)
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
  private Utterance getOriginalContextSpecificUtterance(Symbol user, Symbol agentTeam, String contextIdentifier) {
    // Method 1: Works if engaging in (extended) LLM validation - check in appropriate ChatContext
    user = getUntypedSymbol(user);
    agentTeam = getUntypedSymbol(agentTeam);
    synchronized (currentChatCollections) {
      ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
      if (chatCollection != null) {
        ChatContext pendingContext = chatCollection.pendingFocusedChats.get(contextIdentifier);
        if (pendingContext != null) {
          return pendingContext.utteranceUpForValidation;
        }
      }
    }
    // Backup Method: Old impl checking dialogue history - covers external parser case (which shouldn't actually be possible unless domain is misconfigured)
    // (Added above method because this worked well other than a recently found issue where culling history interfered with properly being able to find the original utterance)
    user = getUntypedSymbol(user);
    agentTeam = getUntypedSymbol(agentTeam);
    List<Utterance> conversationHistory = getConversationHistory(user, agentTeam);
    if (!conversationHistory.isEmpty()) {
      for (int i = conversationHistory.size() - 1; i >= 0; i--) {
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

  /// / General Completion / ChatCollection Utilities ////

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
    Completion response;
    try {
      // What amount of time here?
      response = future.get(10, TimeUnit.SECONDS);
    } catch (Exception e) {
      log.error("[callChatCompletion] exception or timeout waiting on endpoint completion request", e);
      return null;
    }

    return parseContentForTools(response, tools);
  }

  private Completion parseContentForTools(Completion completion, List<ToolDefinition> advertisedTools) {
    if (completion == null) {
      return completion;
    }

    String text = completion.text;
    //TODO: why does NLG output get parsed as reasoning content?
    if (text.isEmpty()) {
      text = completion.reasoning;
    }

    String tool_call_start_token = "<tool_call>";
    String tool_call_end_token = "</tool_call>";
    String function_call_start_token = "function=";
    String function_call_end_token = "</function>";
    String parameter_call_start_token = "<parameter=>";
    String parameter_call_end_token = "</parameter>";
    if (!text.contains(tool_call_start_token)) {
      return completion;
    }

    String tool_call_regex_string = "<tool_call>(.*?)</tool_call>";
    Pattern tool_call_pattern = Pattern.compile(tool_call_regex_string, Pattern.DOTALL);
    Matcher matcher = tool_call_pattern.matcher(text);
    List<String> raw_tool_calls = new ArrayList<>();
    //Gson gson = new Gson();
    //List<HashMap<String,String>> call_maps = new ArrayList<>();
    while (matcher.find()) {
      String res = matcher.group(0).isEmpty() ? matcher.group(0) : matcher.group(1);
      raw_tool_calls.add(res);
      //call_maps.add(gson.fromJson(res,new TypeToken<HashMap<String,String>>(){}.getType()));
      //call_maps.add(gson.fromJson(res,HashMap.class));
    }

    if (raw_tool_calls.isEmpty()) {
      raw_tool_calls.add(text);
    }

    String tool_call_function_regex_string = "<function=(.*?)</function>|<function=(.*)$";
    tool_call_pattern = Pattern.compile(tool_call_function_regex_string, Pattern.DOTALL);
    matcher = tool_call_pattern.matcher(text);
    List<String> raw_function_calls = new ArrayList<>();
    while (matcher.find()) {
      String res = matcher.group(1) != null ? matcher.group(1) : matcher.group(2);
      raw_function_calls.add(res);
    }

    Gson gson = new Gson();
    List<ToolCall> toolCalls = new ArrayList<>();
    String tool_call_parameter_regex_string = "<parameter=(.*?)</parameter>|<parameter=(.*)$";
    if (!raw_function_calls.isEmpty()) {
      for (String raw_function_call : raw_function_calls) {
        //parse_function_call
        int end_index = raw_function_call.indexOf(">");
        if (end_index == -1) {
          continue;
        }
        String function_name = raw_function_call.substring(0, end_index);
        ToolDefinition matchingTool = null;
        for (ToolDefinition toolDefinition : advertisedTools) {
          if (toolDefinition.function.name.equals(function_name)) {
            matchingTool = toolDefinition;
            break;
          }
        }
        String parameters = raw_function_call.substring(end_index + 1);
        HashMap<String, Object> parameterMap = new LinkedHashMap<>();
        tool_call_pattern = Pattern.compile(tool_call_parameter_regex_string, Pattern.DOTALL);
        matcher = tool_call_pattern.matcher(parameters);
        while (matcher.find()) {
          String res = matcher.group(1) != null ? matcher.group(1) : matcher.group(2);
          int index = res.indexOf(">");
          String param_name = res.substring(0, index);
          String param_value = res.substring(index + 1);
          if (param_value.startsWith("\n")) {
            param_value = param_value.substring(1);
          }
          if (param_value.endsWith("\n")) {
            param_value = param_value.substring(0, param_value.length() - 1);
          }
          Object converted_value = null;
          if (matchingTool != null) {
            ToolParameterV2 matchingParam = matchingTool.function.parameters.getProperties().get(param_name);
            if (matchingParam != null) {
              String type = matchingParam.getType();
              if (type.equals("array")) {
                //TODO: Try to address this through prompting/examples or transfer to only using nested XML and not
                // internal JSON. Model seems to add excess close bracket incorrectly in this case specifically.
                String stripped = param_value;
                int numStripped = 0;
                while (numStripped < 2 && stripped.startsWith("[") && !stripped.endsWith("]")) {
                  stripped = stripped.substring(0, stripped.length() - 1);
                  numStripped += 1;
                }
                if (numStripped < 2) {
                  param_value = stripped;
                }
              }
              converted_value = convertParamStringValue(param_value,type);
            }
          }
          if (converted_value == null) {
            parameterMap.put(param_name, param_value);
          } else {
            parameterMap.put(param_name, converted_value);
          }
        }
        ToolCall toolCall = new ToolCall();
        toolCall.type = "function";
        toolCall.function = new ToolCallFunction();
        toolCall.function.name = function_name;
        toolCall.function.arguments = gson.toJson(parameterMap);
        toolCall.id = "call_" + UUID.randomUUID().toString().replace("-", "").substring(0, 24);
        toolCall.type = "function";
        toolCalls.add(toolCall);
      }
    }

    completion.toolCalls = toolCalls;
    completion.text = completion.text.substring(0, completion.text.indexOf(tool_call_start_token));

    return completion;
  }

  private Object convertParamStringValue(String paramValue, String paramType) {
    if (paramType.equals("string") || paramType.equals("str") || paramType.equals("text")
            || paramType.equals("varchar") || paramType.equals("char") || paramType.equals("enum")) {
      return paramValue;
    } else if (paramType.startsWith("int") || paramType.startsWith("uint")
            || paramType.startsWith("long") || paramType.startsWith("short")
            || paramType.startsWith("unsigned")) {
      try {
        return Long.parseLong(paramValue.trim());
      } catch (NumberFormatException e) {
        return paramValue;
      }
    } else if (paramType.startsWith("num") || paramType.startsWith("float")) {
      try {
        double f = Double.parseDouble(paramValue.trim());
        if (f - (long) f != 0.0) return f;
        return (long) f;
      } catch (NumberFormatException e) {
        return paramValue;
      }
    } else if (paramType.equals("boolean") || paramType.equals("bool") || paramType.equals("binary")) {
      String pv = paramValue.toLowerCase();
      return pv.equals("true");
    } else {
      Object converted = paramValue;
      if (paramType.equals("object") || paramType.equals("array") || paramType.equals("arr")
              || paramType.startsWith("dict") || paramType.startsWith("list")) {
        try {
          converted = new Gson().fromJson(paramValue, Object.class);
          return converted;
        } catch (Exception ignored) {
        }
      }
      try {
        String normalized = paramValue
                .replace("True", "true")
                .replace("False", "false")
                .replace("None", "null");
        StringBuilder sb = new StringBuilder(normalized.length());
        boolean inDouble = false;
        for (int i = 0; i < normalized.length(); i++) {
          char c = normalized.charAt(i);
          if (c == '\\' && i + 1 < normalized.length()) {
            sb.append(c).append(normalized.charAt(++i));
            continue;
          }
          if (c == '"') inDouble = !inDouble;
          if (c == '\'' && !inDouble) {
            sb.append('"');
            continue;
          }
          sb.append(c);
        }
        converted = new Gson().fromJson(sb.toString(), Object.class);
      } catch (Exception ignored) {
      }
      return converted;
    }
  }

  @TRADEService
  public void streamingChunkHandler(OpenaiChatCompletionResponse lastChunk, OpenaiChatCompletionResponse currentResponse, Chat currentChat) {
    log.trace("[streamingChunkHandler] in method");

    log.trace("[last_chunk] {}", lastChunk);
    log.trace("[currentResponse] {}", currentResponse);

    if (currentResponse.choices == null || currentResponse.choices.length == 0) {
      return;
    }
    String content = lastChunk.choices[0].delta.content;
    if (content == null || content.isEmpty()) {
      content = lastChunk.choices[0].delta.reasoning;
    }
    if (content == null || content.isEmpty()) {
      return;
    }

    //TODO: cleanup parsers when complete
    //TODO: can short circuit once seeing different function name
    //TODO: separate between NLG and NLU

    String id = currentResponse.id;
    currentStreamingResponseId = id;
    Qwen3CoderToolParser parser = streamingResults.get(id);
    if (parser == null) {
      parser = new Qwen3CoderToolParser();
      parser.resetStream();
      streamingResults.put(id, parser);
    }

    // TODO(pete): is silently dropping the delta the right posture if parser.feed throws?
    try {
      parser.feed(content);
    } catch (Exception e) {
      log.error("[streamingChunkHandler] parser.feed threw; skipping this delta", e);
      return;
    }

    //NLU
    if (parser.currentFunctionName != null && parser.currentFunctionName.equals(textResponseTool.toolDefinition.function.name)) {
      if (parser.currentParamName != null && parser.currentParamName.equals("response")) {
        if (!parser.currentParamValueDelta.isEmpty()) {
          log.info("[streamingChunkHandler] have partial response string: {}. Delta: {}",parser.currentParamValue, parser.currentParamValueDelta);
          //TODO: See code block in handleTextResponse - Need to store in buffer and prevent TTS until knowing the response
          //       is safe to use
          //String user = currentChat.userName;
          //String agentTeam = currentChat.robotName;
          //ChatContext chatContext = null;
          //ChatCollection collection = getChatCollection(Factory.createSymbol(user), Factory.createSymbol(agentTeam));
          //if (collection != null) {
          //  chatContext = collection.currentChat;
          //}
          //if (chatContext != null && chatContext.identicalResponses > 1 && chatContext.lastResponseString.startsWith(currentBufferedDelta)) {
          //    log.info("[streamingChunkHandler] detecting start of repeated response, waiting to push results to TTS until cleared");
          //} else {
            ttsPush(parser.currentParamValueDelta);
          //}
          return;
        }
      }
    }
    if (parser.completedParam && parser.lastParamName.equals("response")) {
      log.info("[streamingChunkHandler] Done parsing response");
      // arm only when fragments really went out; un-streamed replies arm on their spoken path
      if (ttsStreamedIds.contains(id)) {
        armRearm();
      }
      ttsEnd();
      return;
    }

    //TODO: Don't need to actually use the Qwen3CoderToolParser for NLG - go back and separate callbacks used
    //NLG
    String currentText = currentResponse.choices[0].message.content;
    if (currentText == null || currentText.isEmpty()) {
      currentText = currentResponse.choices[0].message.reasoning; //TODO: why starting as null
    }
    if (currentText == null) {
      return;
    }
    if (parser.nlgText.isEmpty()) {
      // accept the same whitespace variants around the key as the completed-parse regex below
      Matcher startMatcher = Pattern.compile("\\{\\s*\"?translation\"?\\s*:\\s*\"?").matcher(currentText);
      if (startMatcher.find()) {
        Matcher m = Pattern.compile(".*\"?translation\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL).matcher(currentText);
        if (m.matches()) {
          log.info("[streamingChunkHandler] One shot NLG parse: {}", m.group(m.groupCount()));
          ttsPush(m.group(m.groupCount()));
          ttsEnd();
        } else {
          parser.nlgText = currentText.substring(startMatcher.end());
          parser.nlgTextDelta = parser.nlgText;
          log.info("[streamingChunkHandler] first NLG delta: {}", parser.nlgTextDelta);
          ttsPush(parser.nlgTextDelta);
        }
      }
    } else {
      Matcher m = Pattern.compile(".*\"?translation\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL).matcher(currentText);
      if (m.matches()) {
        log.info("[streamingChunkHandler] Complete NLG parse: {}", m.group(m.groupCount()));
        ttsEnd();
      } else {
        parser.nlgTextDelta = content;
        parser.nlgText += parser.nlgTextDelta;
        log.info("[streamingChunkHandler] (NLG) have partial response string: {}. Delta: {}",parser.nlgText, parser.nlgTextDelta);
        ttsPush(parser.nlgTextDelta);
      }
    }
  }

  //TODO: in StreamingTTSComponent, have output be buffered by default and add methods unbufferStreamingTTSOutputForUID and cancelStreamingTTSOutputForUID
  private void ttsPush(String fragment) {
    if (fragment == null || fragment.isEmpty()) return;
    String id = currentStreamingResponseId;
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("pushStreamingSpeech").argTypes(String.class, Long.class))
              .call(void.class, fragment, currentStreamingUid);
      // only a push that actually reached the TTS counts as streamed
      ttsStreamedIds.add(id);
    } catch (TRADEException e) {
      log.warn("[streamingChunkHandler] pushStreamingSpeech failed", e);
      if (e.getMessage().contains("didn't find unique service for constraints")) {
        log.error("[streamingChunkHandler] TTS component doesn't exist or contain appropraite services for streaming setup, setting streamResults to false");
      }
    }
  }

  private void ttsEnd() {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("endStreamingSpeech").argTypes(Boolean.class, Long.class))
              .call(Boolean.class, false, currentStreamingUid);
    } catch (TRADEException e) {
      log.warn("[streamingChunkHandler] endStreamingSpeech failed", e);
      if (e.getMessage().contains("didn't find unique service for constraints")) {
        log.error("[streamingChunkHandler] TTS component doesn't exist or contain appropraite services for streaming setup, setting streamResults to false");
      }
    }
  }

  private void armRearm() {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("armPostStreamRearm")).call(void.class);
    } catch (TRADEException e) {
      log.debug("[streamingChunkHandler] armPostStreamRearm unavailable");
    }
  }

  private void ttsCancel(long uid) {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("cancelStreamingTTSOutputForUID").argTypes(Long.class))
              .call(void.class, uid);
    } catch (TRADEException e) {
      log.debug("[ttsCancel] cancelStreamingTTSOutputForUID unavailable");
    }
  }

  private Completion callChatCompletionStreaming(Chat chat, List<ToolDefinition> tools, long uid) {
    currentStreamingUid = uid;
    TRADEServiceInfo tsi = getMyService("streamingChunkHandler", OpenaiChatCompletionResponse.class, OpenaiChatCompletionResponse.class, Chat.class);
    ExecutorService endpointCallExecutor = Executors.newSingleThreadExecutor();
    Future<Completion> future = endpointCallExecutor.submit(() -> {
      try {
        return TRADE.getAvailableService(new TRADEServiceConstraints().name("streamingChatCompletion").argTypes(Chat.class, List.class, TRADEServiceInfo.class)).call(Completion.class, chat, tools, tsi);
      } catch (TRADEException e) {
        log.error("[callChatCompletionStreaming] Error performing chat completion", e);
        return null;
      }
    });
    Completion response;
    try {
      // What amount of time here?
      response = future.get(10, TimeUnit.SECONDS);
    } catch (Exception e) {
      log.error("[callChatCompletionStreaming] exception or timeout waiting on endpoint completion request", e);
      return null;
    }

    return parseContentForTools(response, tools);
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
          chatCollection.currentChat.history = new Chat(user.getName(), agentTeam.getName());
          chatCollection.currentChat.onQuestionAskedHistorySize = 0;
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
      log.debug("[getChatCollection] no chat collection found for {}/{} pair", user, agentTeam);
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
      ToolDefinition td = collection.currentChat.chatType == ChatType.FOCUSED ? chatTemplates.get(collection.currentChat.contextID).tools.get(textResponseTool.toolDefinition.function.name).toolDefinition : textResponseTool.toolDefinition;
      Utterance out = runCompletionRetryLoop(null, List.of(td), dummy, collection, new StringBuilder(), false);
      if (out.getSemantics() != null && (out.getSemantics().getName().equals("generateResponseFromString") || out.getSemantics().getName().equals("updateDialogueHistoryFromString")) && out.getSemantics().isPredicate() && ((Predicate)out.getSemantics()).getArgs().size() > 2) {
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
  public Utterance addToolResponse(Symbol user, Symbol agentTeam, String toolResponse, String toolCallName) {
    Symbol speaker = getUntypedSymbol(user);
    Symbol addressee = getUntypedSymbol(agentTeam);
    ChatCollection collection;
    ChatContext chatContext;
    List<ToolDefinition> tools;
    Chat chat;
    String toolCallId = "";
    synchronized (currentChatCollections) {
      collection = getChatCollection(speaker, addressee);
      if (collection == null) {
        log.error("[addToolResponse] supplied user/agent pair {}/{} does not have an existing Chat history", speaker, addressee);
        //TODO: return an error string? Or does this failure case mean collision with other input/output and we should not respond
        return null;
      } else if (collection.currentChat == null) {
        log.warn("[addToolResponse] no current chat for pair with id {}/{}", speaker, addressee);
        //TODO: return an error string? Or does this failure case mean collision with other input/output and we should not respond
        return null;
      } else {
        chatContext = collection.currentChat;
        int size = chatContext.history.messages.size();
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

        ChatToolList chatToolList = prepChatAndToolList(chatContext, agentTeam, user);
        tools = chatToolList.tools;
        chat = chatToolList.chat;
      }

      Utterance dummy = new Utterance.Builder().setSpeaker(user).setAddressee(agentTeam).build();
      updatePromptVars(dummy,true);
      boolean useValidation = true;
      try {
        useValidation = TRADE.getAvailableService(new TRADEServiceConstraints().name("usingValidation")).call(Boolean.class);
      } catch (TRADEException e) {
        log.error("[addToolResponse] error reaching out to FrontEndConfigurationComponent for validation setting");
      }
      dummy.setNeedsValidation(useValidation);
      Utterance out = runCompletionRetryLoop(null, tools, dummy, collection, new StringBuilder(), false, chat);

      // TODO: quick band-aid fix, revisit - Removing retrieved past conversation history from the current history as it
      //    can get very large very quickly. Can more safely leave in if proper history trimming and request token length
      //    checking is implemented using an internal tokenizer.
      //   See if the model appropriately re-calls getExtendedConversationHistory if necessary and having a multi-turn conversation about it
      if (toolCallName.equals("checkPastInteractionHistoryForMissingInformation")) {
        if (!toolCallId.isEmpty()) {
          chatContext = collection.currentChat;
          int size = chatContext.history.messages.size();
          int toolCallIndex = -1;
          int toolCallResponseIndex = -1;
          for (int i=size-1;i>0;i--) {
            Message msg = chatContext.history.messages.get(i);
            if (msg.tool_calls != null && msg.tool_calls.length>0) {
              if (msg.tool_calls[0].id.equals(toolCallId)) {
                toolCallIndex = i;
              }
            } else if (msg.tool_call_id != null && msg.tool_call_id.equals(toolCallId)) {
              toolCallResponseIndex = i;
            }
          }
          if (toolCallResponseIndex != -1) {
            chatContext.history.messages.remove(toolCallResponseIndex);
          }
          if (toolCallIndex != -1) {
            chatContext.history.messages.remove(toolCallIndex);
          }
        }
      }

      if (out.getSemantics() != null) {

        // New - any sort of response allowed
        if (out.getSemantics().getName().equals(toolCallName)) {
          log.warn("[addToolResponse] ITK response attempting to call itself, not allowing");
        } else {
          // Need to grab input lock to have side effect occur because we are not explicitly hitting parseUtteranceLLMService
          // again, but forwarding this return value to the NLU pipeline
          collection.grabInputLock(out, true);
          return out;
        }

        log.info("[addToolResponse] Trying again with added guidance before failing");
        ToolDefinition toolDefinition = null;
        ChatTemplates chatTemplates = this.chatTemplates.get(chatContext.contextID);
        if (chatTemplates != null) {
          Tool tool = chatTemplates.tools.get(textResponseTool.toolDefinition.function.name);
          if (tool != null) {
            toolDefinition = tool.toolDefinition;
          }
        }
        if (toolDefinition == null) {
          toolDefinition = textResponseTool.toolDefinition;
        }
        if (toolCallId.isEmpty()) {
          chatContext.history.addMessage("tool", "Please respond using the " + toolDefinition.function.name + "tool call, you cannot call any other tools at this moment.");
        } else {
          chatContext.history.addToolMessage("Please respond using the " + toolDefinition.function.name + "tool call, you cannot call any other tools at this moment.", toolCallId);
        }
        //TODO: should this use the same gen chat or not?
        out = runCompletionRetryLoop(null, List.of(toolDefinition), dummy, collection, new StringBuilder(), false);
        if (out.getSemantics() != null && (out.getSemantics().getName().equals("generateResponseFromString") || out.getSemantics().getName().equals("updateDialogueHistoryFromString")) && out.getSemantics().isPredicate() && ((Predicate) out.getSemantics()).getArgs().size() > 2) {
          collection.grabInputLock(out, true);
          return out;
        } else {
          log.error("[addToolResponse] got unexpected output even after asking to use text response {}", out);
          collection.grabInputLock(out, true);
          out.setSemantics(Factory.createPredicate("generateResponseFromString", agentTeam.toString(), "Sorry, an internal error occurred. Could you repeat that?"));
          return out;
        }
      }
      log.error("[addToolResponse] got unexpected output {}", out);
      collection.grabInputLock(out, true);
      out.setSemantics(Factory.createPredicate("want",user,Factory.createPredicate("generateResponseFromString", agentTeam.toString(), "Sorry, an internal error occurred. Could you repeat that?")));
      return out;
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
    long kioskGoalGid = -1;
    List<Predicate> pendingGoalPredicates = getPendingGoals(agent);
    List<Predicate> scheduledGoals = getScheduledGoals(agent);
    //TODO: call endpoint to transform all of the predicates to String descs
    int numCurrent = currentGoalPredicates.size();
    numCurrent = numCurrent == 0 ? 0 : 1; //TODO: handle multiple current if not using queue
    int numPending = pendingGoalPredicates.size();
    int numScheduled = scheduledGoals.size();
    List<String> translatedGoalList = new ArrayList<>();
    if (numCurrent + numPending + numScheduled != 0) {
      Chat goalGenChat = new Chat(getGoalTranslationPrompt().apply(promptVars));
      StringBuilder goalListString = new StringBuilder();

      if (numCurrent != 0) {
        try {
          kioskGoalGid = TRADE.getAvailableService(new TRADEServiceConstraints().name("getKioskGoalGid").argTypes(Symbol.class)).call(Long.class,agent);
        } catch (TRADEException e) {
          log.debug("[getFullGoalsSummary] Kiosk goal manager not in system or otherwise unable to get kiosk goal gid", e);
        }
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

//    StringBuilder stringBuilder = new StringBuilder("At any point in time, you may have been instructed to perform various" +
//            " tasks by different users, but cannot do everything all at once. As a result, you keep track of the list of" +
//            " things you need to do and order them according to a priority value. By default, this is the order in which the tasks" +
//            " are submitted. You do not currently have the capability to change priority of submitted tasks, but will be able" +
//            " to soon in a future update. Because you cannot perform multiple tasks at once, there will be one task which" +
//            " you are actively focused on. This task can be referred to as the \"active task\", \"suspended task\", \"current task\"," +
//            " or simply \"what you are currently doing\". As stated, this task can be temporarily \"suspended\". In this state, that task" +
//            " will remain as your current focus, and you will wait until it is resumed or canceled to perform any other tasks, even if" +
//            " others are present. The rest of the list of things you need to do can be referred to as your \"pending tasks\", " +
//            "\"upcoming tasks\", or simply \"what you still need to do\" after your current task. There is no limit to the amount of" +
//            " pending tasks you can have at once. Finally, there are also \"scheduled tasks\". These are tasks which are scheduled" +
//            " to occur at some specific point in the future, optionally on a repeating interval, but are not currently on" +
//            " the active or pending list of tasks to perform. When the scheduled time arrives, the associated task will be added to the" +
//            " list of things you must do.\n-------\n");

    StringBuilder stringBuilder = new StringBuilder("Current Task Overview:\n" +
            "Information about Tasks:\n" +
            "- You have a collection of tasks you currently have assigned to you at any time. This collection is split into three types of tasks:\n" +
            "-- One Current Task:\n" +
            "--- The current task is the task you are focused on. There is only one current task at a time.\n" +
            "--- The current task has an associated status. This status may be ACTIVE or SUSPENDED. \n" +
            "---- ACTIVE status == actively engaging in actions pursuing the task. \n" +
            "---- SUSPENDED status == paused and waiting for the current task to be resumed or canceled. You are still focusing on this task even in a SUSPENDED state. \n" +
            "-- A queue of Pending Tasks\n" +
            "--- Queued/Pending tasks are tasks which will become the current task after the existing current task completes. These tasks are in an ordered list." +
            "--- Newly instructed tasks are placed at the end of the pending task queue by default.\n" +
            "--- Once the current task completes, the first pending task will become the new current task.\n" +
            "-- A set of Scheduled Tasks\n" +
            "--- Scheduled tasks are tasks which are scheduled to become the current task at specific times in the future" +
            "\nCollection of Existing Tasks:\n");

    stringBuilder.append("Current Task:\n");
    if (!currentGoalPredicates.isEmpty()) {
        if (currentGoalPredicates.get(0).hasArgs()) {
          String statusString = currentGoalPredicates.get(0).getName();
          stringBuilder.append(" - 0: Status: ").append(statusString).append(", Description: ").append(translatedGoalList.get(0)).append("\n");
        } else {
          stringBuilder.append("- 0: Description: ").append(translatedGoalList.get(0)).append("\n");
        }
      if (kioskGoalGid != -1) {
        stringBuilder.append("This is a kiosk task which you defaulted to performing due to not receiving instructions for an extended amount of time. This task will be canceled and overwritten by any subsequent task instruction. If asked about this task, make sure to include that it is a kiosk task triggered due to inactivity and that you are ready to move on at any time.\n");
      }
    } else {
      stringBuilder.append(" - <You have no current task>\n");
    }

    stringBuilder.append("Pending Task Queue:\n");
    if (pendingGoalPredicates.isEmpty()) {
      stringBuilder.append(" - <You have no pending tasks>\n");
    } else {
      for (int i = 0; i < numPending; i++) {
        stringBuilder.append(" - ").append(i).append(": ").append(translatedGoalList.get(i+1)).append("\n");
      }
    }

    stringBuilder.append("Scheduled Tasks:\n");
    if (scheduledGoals.isEmpty()) {
      stringBuilder.append(" - <You have no scheduled tasks>\n");
    } else {
      for (int i = 0; i < numScheduled; i++) {
        stringBuilder.append(" - ").append(i).append(": ").append(translatedGoalList.get(i+numCurrent+numPending)).append("\n");
      }
    }
    //Doesn't seem to do anything - may need to just split calls for retrieving current vs pending vs scheduled vs all tasks
//    stringBuilder.append("-------\nOnly respond to the user with information relevant to what they actually asked about " +
//            "which caused you to trigger the getAssignedTasks tool call. For example: asking what" +
//            " you are doing should cause you to only say what your current task is, being asked how many scheduled goals you have" +
//            " should cause you to only answer with the number of scheduled goals, and being asked what you have to do after your" +
//            " current task should cause you to only begin listing your pending tasks. Do not list more than 5 tasks at once" +
//            " without being asked to list more.");

//    stringBuilder.append("-------\nResponse Rules:\n" +
//            "- Only respond to the user with information relevant to what they actually asked about.\n" +
//            "- Do not list more than 5 tasks at once without explicitly being asked to list more.");
    return stringBuilder.toString();
  }

  private void saveContextHistory(Symbol user, Symbol agentTeam, ChatContext chatContext) {
    if (chatContext == null || chatContext.history == null || chatContext.history.messages.isEmpty()) {
      return;
    }

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

    //Prevent the LLM from just regurgitating the history directly
    // "If not contained in this history, then you must tell the user you cannot remember this interaction"
    StringBuilder extendedHistoryString = new StringBuilder("-Extended History:\n##\n");

    if (extendedHistory == null) {
      extendedHistoryString.append("<You do not remember any past histories with this user>\n##\n");
    } else {
      // TODO: should be based on token length, not number of chats. (4 should now be safe without having conversation histories inside of histories)
      // What should cap size be? TODO: Should we allow iteratively longer history retrieval?
      int numHistories = Math.min(extendedHistory.size(), 4);
      for (int i=0; i<numHistories;i++) {
        ChatContext cc = extendedHistory.get(i);
        if (cc != null) {
          extendedHistoryString.append("-- Past Conversation History Number ").append(i).append(":\n#");
          extendedHistoryString.append(getChatHistoryAsString(cc.history)).append("\n##\n");
        }
      }
      ChatCollection collection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
      if (collection != null && !collection.suspendedChats.isEmpty()) {
        extendedHistoryString.append("\n- You also have some interrupted conversations which you may still eventually return to and complete. They are as follows:\n##\n");
        List<ChatContext> suspendedContexts = collection.suspendedChats;
        for (int i=0; i<suspendedContexts.size();i++) {
          ChatContext cc = suspendedContexts.get(i);
          if (cc != null) {
            extendedHistoryString.append("Suspended Conversation History Number ").append(i).append(":\n#");
            extendedHistoryString.append(getChatHistoryAsString(cc.history)).append("\n#\n");
          }
        }
      }
    }

    extendedHistoryString.append("\n##\n-Response Guideline: " +
            "-- Please use this information in order to inform your future responses to the user, whether it is to answer " +
            "a question directly about what you have discussed in the past or to simply understand something from the past " +
            "that the user is referring to.\n" +
            "-- Do not relay this information back to the user unless they have directly asked about it, in " +
            "which case you may summarize relevant conversations in natural language.\n" +
            "-- If the user has referenced something which you cannot find in any of these histories or your current one, you must tell the user that you are sorry and cannot remember what they are referring to.\n" +
            "-- If none of this information is relevant, you may ignore it.");
    return extendedHistoryString.toString();
  }

  //TODO: does this need to be extended to agent pairs?
  protected String getCurrentQuestionsString(Symbol agent) {
    String output = "";
    try {
      String questionString = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestion").argTypes(Symbol.class)).call(String.class,agent);
      List<Term> expectedResponseForms = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestionResponseForm").argTypes(Symbol.class)).call(List.class,agent);
      if (expectedResponseForms != null && !expectedResponseForms.isEmpty()) {
        //output = "Through previous interaction between the user and the back end system, the system has came up with its own question(s) for the user. The question(s) and their corresponding expected reply forms are presented below.";
        output = "-- OUTSTANDING QUESTION: \"" + questionString + "\""+ "\n-- EXPECTED REPLY FORMS:" + String.join(", ",expectedResponseForms.stream().map(Term::toString).toList());
        output += "\n-- There should be function entries with name and arguments matching each of the expected reply forms. It is very likely that the most recent user input should be interpreted as a response to this question(s) as opposed to a separate instruction in the case where it may reasonably be interpreted as either.";
      } else {
        output += "<No questions have been asked by the back end system>";
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
    Tool correspondingTool;
    correspondingTool = new Tool(new ToolDefinition(new ToolFunction(signature.getName(), "An action previously taught by the user. Call this if the user asks you to perform something referring to the supplied name.")),UtteranceType.INSTRUCT);
    String nluString = toolToDescString(correspondingTool, false);//true);
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
      promptVars.put("%scheduleInstructList", promptVars.get("%scheduleInstructList") + String.format("\n%s\n#", nluString));
      promptVars.put("%taskInstructList", promptVars.get("%taskInstructList") + String.format("\n%s\n#", nluString));
      // Add as tool target in base context
      chatTemplates.get("nlu").toolDefinitions.add(correspondingTool.toolDefinition);
      chatTemplates.get("nlu").tools.put(signature.getName(), correspondingTool);
      chatTemplates.get("nluextended").toolDefinitions.add(correspondingTool.toolDefinition);
      chatTemplates.get("nluextended").tools.put(signature.getName(), correspondingTool);
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
      promptVars.put("%scheduleInstructList", promptVars.get("%scheduleInstructList").replace(String.format("\n%s\n#", nluString), ""));
      promptVars.put("%taskInstructList", promptVars.get("%taskInstructList").replace(String.format("\n%s\n#", nluString), ""));
      // Remove as tool target in base context
      chatTemplates.get("nlu").toolDefinitions.remove(correspondingTool.toolDefinition);
      chatTemplates.get("nlu").tools.remove(name);
      chatTemplates.get("nluextended").toolDefinitions.remove(correspondingTool.toolDefinition);
      chatTemplates.get("nluextended").tools.remove(name);
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

    ChatCollection chatCollection;
    Predicate semantics = (Predicate) utterance.getSemantics();
    Symbol agentTeam;
    Symbol user;

    if (utterance.isInputUtterance()) {
      synchronized (currentChatCollections) {
        //NLU
        if (utterance.getSemantics() == null || !utterance.getSemantics().isPredicate()) {
          return;
        }
        agentTeam = utterance.getAddressee().toUntyped();
        user = utterance.getSpeaker().toUntyped();
        chatCollection = getChatCollection(user, agentTeam);

        // Create new collection in case another parser hit completes and triggers a history update
        //  before the input even reaches this parser. In this case, we want to short-circuit the parse call for this
        //  utterance to prevent any unncessary side effects.
        if (chatCollection == null) {
          chatCollection = new ChatCollection();
          currentChatCollections.put(Pair.of(user.toUntyped(), agentTeam.toUntyped()), chatCollection);
        }
      }

      if (chatCollection.utteranceBeingProcessed != null && chatCollection.utteranceBeingProcessed.getUid() != utterance.getUid()) {
        log.error("[updateLLMConversationHistory] waiting for other utterance to finish being processed before applying effects");
        chatCollection.grabInputLock(utterance);
      }

      synchronized (currentChatCollections) {
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

            // If negative validation response, don't release inputLock until reattemptParse completes
            if ((semantics.getName().equals("val") || semantics.getName().equals("validated"))) {
              //// Synchronize history update before question answer response callback side effects (new because external parsers can handle validation responses now)
              //chatContext.signalValidationResponseHistoryUpdate();
              //chatContext.history.addUserMessage(utterance.getWordsAsString()); //Removed with recent update: adding to separate QA chat which is merged back in if the interaction continues
              if (semantics.get(0).getName().equals("no")) {
                return;
              }
            }
          }

          // Perform necessary side effects to current chat. 2 cases handled collectively here:
          //   External parser hit for an action instruction - discard chat context holding any side effects from unused LLM parse
          //   LLM noConfirmation case - handle side effects as if a validated instruction just occurred
          //   +1
          // 9/9 modification: do not clear context / handle validated tool call if currently in a VALIDATION state. As
          //  of now, any INSTRUCT input will be rejected if an outstanding system question still exists.
          if (utterance.getType() == UtteranceType.INSTRUCT && !utterance.needsValidation() && chatContext.chatState != ChatState.VALIDATION) {
            String command = chatContext.lastToolCallName;
            chatContext.lastToolCallName = "";
            if (utterance.getSemantics() != null && utterance.getSemantics().isPredicate()) {
              if (semantics.getArgs().size() > 1) {
                if (command.isEmpty() || semantics.get(1).getName().equals("generateResponseFromString") || semantics.get(1).getName().equals("updateDialogueHistoryFromString")) {
                  command = semantics.get(1).getName();
                }
              }
            }
            if (!(command.equals("generateResponseFromString") || command.equals("updateDialogueHistoryFromString"))) {
              handleValidatedToolCall(command, utterance.getSpeaker(), utterance.getAddressee(), chatCollection);
            }
          }
        }

        // TODO: do we want to extend holding the lock until around tentative accept response or failure response?
        // Side effects complete - Unlock input lock for this user/agent pair
        chatCollection.signalInputProcessed(utterance);
      }
    } else {
      synchronized (currentChatCollections) {
        //NLG - add to current context if exists
        user = utterance.getAddressee().toUntyped();
        agentTeam = utterance.getSpeaker().toUntyped();
        chatCollection = getChatCollection(user, agentTeam);
        if (chatCollection == null) {
          chatCollection = new ChatCollection(new Chat(user.getName(), agentTeam.getName()));
          currentChatCollections.put(Pair.of(user.toUntyped(), agentTeam.toUntyped()), chatCollection);
        }
        ChatContext chatContext = chatCollection.currentChat;
        if (chatContext == null) {
          Chat chat = new Chat(user.getName(), agentTeam.getName());
          chatContext = new ChatContext(chat);
          chatCollection.currentChat = chatContext;
        }
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
          ToolCallFunction toolCallFunction = new ToolCallFunction();
          toolCallFunction.name = "validatingToolSubmission";
          Map<String, Object> responseArguments = new HashMap<>();
          responseArguments.put("response", utterance.getWordsAsString());
          toolCallFunction.arguments = new Gson().toJson(responseArguments);
          toolCall.function = toolCallFunction;
          chatContext.history.addRobotMessage("", List.of(toolCall));
        } else {
          // TODO: Keep an eye on this, make sure it's okay that these seemingly unprompted assistant messages are present.
          Tool textResponseTool = chatTemplates.get(chatContext.contextID).tools.get(this.textResponseTool.toolDefinition.function.name);
          if (textResponseTool != null) {
            ToolCall toolCall = new ToolCall();
            toolCall.id = chatContext.lastResponseToolCallId;
            toolCall.type = "function";
            ToolCallFunction toolCallFunction = new ToolCallFunction();
            toolCallFunction.name = this.textResponseTool.toolDefinition.function.name;
            Map<String, Object> responseArguments = new LinkedTreeMap<>();
            //TODO: this argument order needs to align with how it is defined on init - automatically enforce this
            responseArguments.put("response", utterance.getWordsAsString());
            if (!chatContext.lastTemplateName.isEmpty() && textResponseTool.toolDefinition.function.parameters.getProperties().containsKey(chatContext.lastTemplateName)) {
              responseArguments.put(chatContext.lastTemplateName, chatContext.lastTemplateValue);
            }
            toolCallFunction.arguments = new Gson().toJson(responseArguments);
            toolCall.function = toolCallFunction;
            chatContext.history.addRobotMessage("", List.of(toolCall));
          }
        }
      }
      //TODO: Should this be added to a new history/context if none currently exists? Hard question to answer because
      //  sometimes it may be relevant and sometimes it may throw off future parses significantly. Best case is to
      //  have an "extended dialogue history" itk target which is handled well
      // ^9/10 rewrote the above in order to add this to new contexts. Added specifically for things like escort, where
      //  the instruction from the user may come from a remote place, but the first output from the robot as part of the
      //  action addresses the local user.
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

    //TODO: should history be cleared in general after question asking termination?

    //// Synchronize between necessary dialogue history side effects and potential reattemptParse call
    ChatContext chatContext;
    Symbol user = question.getAddressee().toUntyped();
    Symbol agentTeam = agent.toUntyped();
    synchronized (currentChatCollections) {
      ChatCollection chatCollection = getChatCollection(user, agentTeam);
      if (chatCollection != null) {
        chatContext = chatCollection.currentChat;
        if (chatContext != null) {
          // Reset QA specific "past history"
          chatContext.onQuestionAskedHistorySize = -1;
          if (chatContext.chatState == ChatState.VALIDATION) {
            if (!question.wasAnswered()) {
              // If validation question expired but chat context is still in validation state, we know the validation question expired.
              //  So update chat state accordingly.
              chatCollection.signalInputProcessed();
              if (chatContext.chatType == ChatType.FOCUSED) {
                chatContext.chatState = ChatState.PARSE;
                chatContext.history.addToolMessage("Your last validation question was not answered after a long duration and timed out from the perspective of the system. You may need to remind the user of your task if they seem unaware of your current task of \"" + chatContext.contextID + "\" and will need to call your target tool call again if they try answering your question affirmatively.", chatContext.toolCallIdUpForValidation);
                chatContext.toolCallIdUpForValidation = "";
                chatContext.utteranceUpForValidation = null;
              } else {
                chatContext.history.addToolMessage("Your last validation question was not answered after a long duration and timed out from the perspective of the system", chatContext.toolCallIdUpForValidation);
                saveContextHistory(user, agentTeam, chatCollection.currentChat);
                chatCollection.currentChat = null;
              }
            } else {
              // TODO: block reattemptParse call until this (and dialogue history update) happens
              // (This logic was moved from handleValidatedToolCall to here to support external parsers parsing validation question responses)
              // Validation question was explicitly answered, update chat state accordingly
              Map<Variable, Symbol> bindings = question.getAnswerBindings();
              if (bindings != null) {
                //chatContext.history.addUserMessage(question.getWordsAsString());
                Symbol answer = bindings.get(Factory.createVariable("X"));
                if (answer != null) {
                  if (answer.getName().equals("yes")) {
                    // If positively validated, terminate associated context and submit instruction
                    handleValidatedToolCall(chatContext.utteranceUpForValidation, chatCollection);
                  } else if (answer.getName().equals("no")) {
                    chatContext.lastToolCallName = "";
                    // Negative validation - reset to PARSE state to continue interaction. Architecture messages and other
                    //  side effects will be triggered from reattemptParse / the NLU asl pipeline
                    chatContext.chatState = ChatState.PARSE;
                  }
                } else {
                  // User explicitly declined to answer the validation question - terminate context entirely and refocus base parse state
                  chatCollection.currentChat.signalCompletion();
                  saveContextHistory(user, agentTeam, chatCollection.currentChat);
                  chatCollection.currentChat = null;
                }
              }
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
  private Utterance processToolCall(ToolCallFunction functionCall, Tool tool, Utterance inputUtterance, StringBuilder errorMessage) {
    Utterance.Builder builder = new Utterance.Builder(inputUtterance);
    Symbol actor = inputUtterance.getAddressee();
    Map<Symbol,Variable> varMap = new HashMap<>();
    boolean includeActor = tool.utteranceType == UtteranceType.INSTRUCT || tool.utteranceType == UtteranceType.QUESTION;
    Symbol semantics = processMapArgument(functionCall.name, functionCall.arguments, tool.toolDefinition.function.parameters, includeActor, actor, builder, varMap, errorMessage, false, false);
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
  private Symbol processArgument(String parameterName, Object parameterValue, ToolParameterV2 parameterDefinition, String parentArgumentName, Symbol actor, Utterance.Builder builder, Map<Symbol,Variable> varMap, StringBuilder errorMessage, boolean unpopulatedArgSafe, boolean emptyArgAllowed) {
    Symbol processedArg = null;
    boolean argValid = true;
    unpopulatedArgSafe = unpopulatedArgSafe || parentArgumentName.equals("getIfCanPerform"); //TODO; revisit this and associated cases
    if (parameterValue instanceof List<?>) {
      //// Handle List
        List<Object> argObjectList = (List<Object>) parameterValue; //Can you actually have a ClassCastException here? Catch and handle if so
        List<Symbol> innerArgs = new ArrayList<>();
        for (Object argValue : argObjectList) {
          Symbol innerArg = processArgument(parameterName, argValue, parameterDefinition, parentArgumentName, actor, builder, varMap, errorMessage, unpopulatedArgSafe, emptyArgAllowed);
          if (innerArg == null) {
            argValid = false;
            continue;
          }
          innerArgs.add(innerArg);
        }
        processedArg = Factory.createPredicate("and",innerArgs);
    } else if (parameterValue instanceof Map<?,?>) {
      //// Handle Map (Tool)
      if (parameterDefinition.getProcessAsTool()) {
        processedArg = processArbitraryToolArgument(parameterName, parameterValue, parameterDefinition, parentArgumentName, actor, builder, varMap, errorMessage, unpopulatedArgSafe, emptyArgAllowed);
      } else {
        processedArg = processMapArgument(parameterName, parameterValue, parameterDefinition, true, actor, builder, varMap, errorMessage, unpopulatedArgSafe, emptyArgAllowed);
      }
    } else if (parameterValue instanceof String) {
      if (parameterDefinition.getProcessAsTool()) {
        //// Handle Tool Json String
        processedArg = processArbitraryToolArgument(parameterName, parameterValue, parameterDefinition, parentArgumentName, actor, builder, varMap, errorMessage, unpopulatedArgSafe, emptyArgAllowed);
      } else {
        //// Handle General String (Symbol/Predicate)
        String stringValue = (String) parameterValue;
        if (parentArgumentName.equals(textResponseTool.toolDefinition.function.name) || parentArgumentName.equals("sayLocally")) {
          if (!(stringValue.startsWith("\"") && stringValue.endsWith("\""))) {
            stringValue = "\"" + stringValue + "\"";
          }
        }
        processedArg = processSymbolArgument(parameterName, stringValue, parameterDefinition.getPossible_values(), errorMessage, varMap, builder, false, unpopulatedArgSafe, emptyArgAllowed);
      }
    } else {
      //// Handle Number - anything else possible?
      String stringValue = String.valueOf(parameterValue);
      processedArg = processSymbolArgument(parameterName, stringValue, parameterDefinition.getPossible_values(), errorMessage, varMap, builder, false, unpopulatedArgSafe, emptyArgAllowed);
    }

    if (argValid) {
      return processedArg;
    } else {
      return null;
    }
  }
  // Process an inner argument which has the flag set to be processed as a tool call
  protected Symbol processArbitraryToolArgument(String argumentName, Object argumentValue, ToolParameterV2 argumentDefinition, String parentArgumentName, Symbol actor, Utterance.Builder builder, Map<Symbol,Variable> varMap, StringBuilder errorMessage, boolean unpopulatedArgSafe, boolean emptyArgAllowed) {
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
          log.info("[processToolArgument] hit array as string case, trying to handle as list. {}", argumentValue);
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
            Symbol innerArg = processArbitraryToolArgument(argumentName, argValue, argumentDefinition, parentArgumentName, actor, builder, varMap, errorMessage, unpopulatedArgSafe, emptyArgAllowed);
            if (innerArg == null) {
              return null;
            }
            innerArgs.add(innerArg);
          }
          return Factory.createPredicate("and", innerArgs);
        } catch (JsonSyntaxException e2) {
          //TODO: this error message doesn't actually cause the LLM to fix itself all the time - manually inspect what type of object it is and try to process as such manually?
          errorMessage.append("- The supplied definition for the \"").append(argumentName).append("\" parameter of the \"").append(parentArgumentName).append("\" call could not be parsed as a valid json string. Please reformat your call to supply a valid json string or supply an object directly. Make sure all brackets, parentheses, and quotes are matched up properly. If you are supplying an array, do not supply it as a string; supply the object directly. You do not need to speak with the user about this. Respond with the updated call. \n");
          log.error("[processToolCall] malformed json present in functionCall", e2);
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
        Symbol innerArg = processArbitraryToolArgument(argumentName, argValue, argumentDefinition, parentArgumentName, actor, builder, varMap, errorMessage, unpopulatedArgSafe, emptyArgAllowed);
        if (innerArg == null) {
          return null;
        }
        innerArgs.add(innerArg);
      }
      return Factory.createPredicate("and", innerArgs);
    } else {
      errorMessage.append("- The supplied tool call definition for the \"").append(argumentName).append("\" parameter of the \"").append(parentArgumentName).append("\" call could not be parsed as json. Please reformat your call to supply a valid json string or object. You do not need to speak with the user about this.\n");
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

    String toolName = (String)toolDefinitionMap.get("name");
    Object innerArguments = toolDefinitionMap.get("arguments");
    if (toolName == null || toolName.isEmpty()) {
      if (emptyArgAllowed) {
        return Factory.createSymbol("empty");
      }
      log.warn("[processArgument] inner tool call does not have name field present: {}", argumentValue);
      errorMessage.append("- You supplied action json for the \"").append(argumentName).append("\" argument of the \"").append(parentArgumentName).append("\" call, but it was missing a \"name\" field. Please include the proper name of this action.\n");
      validToolDefinition = false;
    }
    if (innerArguments == null && !unpopulatedArgSafe) {
      log.warn("[processArgument] inner tool call does not have arguments field present: {}", argumentValue);
      errorMessage.append("- You supplied action json for the \"").append(argumentName).append("\" argument of the \"").append(toolName).append("\" call, but it was missing an \"arguments\" field. Please restructure your json to properly include an arguments field for this action, even if an empty map. You do not need to speak to the user about this.\n");
      validToolDefinition = false;
    }
    Tool innerTool = innerToolChatTemplates.tools.get(toolName);
    if (innerTool == null) {
      log.warn("[processArgument] inner tool call's name does not match a Tool in the corresponding ChatTemplates: {}", toolName);
      errorMessage.append("- You supplied action json for the \"").append(argumentName).append("\" argument of the \"").append(parentArgumentName).append("\" call, but the supplied name \"").append(toolName).append("\" does not correspond to the name of a valid action. If you misspelled an existing valid action, then correct the spelling. If there is no such action, then remove the entry and inform the user that you do not know how to perform that action. Generate a new response string in your next call focusing on resolving this fact.\n");
      validToolDefinition = false;
    }

    if (validToolDefinition) {
      return processMapArgument(toolName, innerArguments, innerTool.toolDefinition.function.parameters, true, actor, builder, varMap, errorMessage, unpopulatedArgSafe, emptyArgAllowed);
    } else {
      return null;
    }
  }

  // Process extracted tool call information
  protected Symbol processMapArgument(String name, Object arguments, ToolParameterV2 parameterDefinition, boolean includeActor, Symbol actor, Utterance.Builder builder, Map<Symbol,Variable> varMap, StringBuilder errorMessage, boolean unpopulatedArgSafe, boolean emptyArgAllowed) {
    boolean validParse = true;
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
    if (!emptyArgAllowed) {
      validParse = checkRequiredArgsExist(name, calledArguments.keySet(), parameterDefinition, errorMessage);
    }

    // Add actor to output semantics
    List<Symbol> processedArguments = new ArrayList<>();
    //TODO: QUESTION should not be in this case once ITK pipeline is fixed - and/or make distinction between going through
    //      the itk pipeline to have it generate its own response vs calling the underlying action to populate a tool response
    if (includeActor) {
      String actorName = actor.getName();
      processedArguments.add(Factory.createSymbol(actorName));
    }

    // Validate Tool call's supplied arguments
    //TODO: Swapped direction we check matching arguments in - add check back in for a supplied argument which doesn't exist?
    for (String argName : parameterDefinition.getProperties().keySet()) {
      Symbol processedArg;
      ToolParameterV2 targetParam = parameterDefinition.getProperties().get(argName);
      Object argValue = calledArguments.get(argName);
      boolean emptyInnerArgAllowed = false;
      //TODO: general mechanism to ignore fields when extracting final result? E.g. want the llm to keep track of the
      // template during the extended dialogue, but don't care if it's fully valid yet.
      if (name.equals(textResponseTool.toolDefinition.function.name) && !argName.equals("response")) {
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
        validParse = validParse && processFocusedTemplate(argName, calledArguments.get(argName), temp.getSpeaker(), temp.getAddressee(), errorMessage);
        emptyInnerArgAllowed = true;
      }
      processedArg = processArgument(argName, argValue, targetParam, name, actor, builder, varMap, errorMessage, unpopulatedArgSafe, emptyArgAllowed || emptyInnerArgAllowed);
      if (processedArg == null) {
        validParse = false;
        continue;
      }
      processedArguments.add(processedArg);
    }

    // Return appropriate object
    if (validParse) {
      //TODO: remove when changing task teaching format (otherwise if keeping any version of this, the actual action and
      //  associated NLG needs to be updated from expecting "and" to "locationActionPairs"
      if (name.equals("locationActionPairs")) {
        processedArguments.remove(0);
        return Factory.createPredicate("and", processedArguments);
      }
      return Factory.createPredicate(name, processedArguments);
    } else {
      return null;
    }
  }
  // TODO: Do we need to check for unintended JSON strings and reject here?
  // Process a "leaf" (i.e. maximally a Predicate, not list or map) tool call argument
  protected Symbol processSymbolArgument(String argName, String argString, List<String> possible_values, StringBuilder errorMessage, Map<Symbol,Variable> varMap, Utterance.Builder builder, boolean inner, boolean unpopulatedArgSafe, boolean emptyArgAllowed) {
    if (argString.isEmpty() && emptyArgAllowed) {
      return Factory.createSymbol("empty");
    }
    Symbol processedArg;
    Symbol argValue = Factory.createFOL(argString);
    // If there is an explicit possible_values set, make sure the argument is present
    if (!possible_values.isEmpty()) {
      //TODO: what general fuzzy matching do we want here? Or is the responsibility on doing so in whatever inner action
      //  and providing the simplest variation here?
      if (!possible_values.contains(argString)) {
        errorMessage.append("- Your tool call contained an invalid value for a ").append(inner ? "inner " : "").append("\"").append(argName).append("\" argument. Please correct any spelling, punctuation, or capitalization mistakes to exactly match a value in the possible_values set if possible. If an obvious similar possible values exists, you may update your call with that value. Otherwise, if no obvious candidates exist, work with the user to specify a valid value as a replacement. Supplied value: ").append(argString).append(", possible values: ").append(possible_values).append(". Do not list out all possible values if responding to the user, unless they ask for that information. Do not include :<type> in your explanation of these values to the user. If this argument is not required, you may remove it.\n");
        return null;
      }

      // Replace reference descriptor with variable if type contained in refTypes
      //TODO: can probably refactor the below blocks to eliminate duplicate code
      if (getRefTypes().contains(argValue.getType())) {
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
          Symbol processedInnerArg = processSymbolArgument(argName, innerArg.toString(), new ArrayList<>(), errorMessage, varMap, builder, true, unpopulatedArgSafe, emptyArgAllowed);
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

        if (argString.isEmpty()) {
          errorMessage.append("- The tool call's supplied value for a \"").append(argName).append("\" argument was empty. If it is a required argument, populate it with the help of the user if necessary. Otherwise, simply omit this argument.");
          return null;
        }

        Symbol argSymbol = Factory.createFOL(argString);
        // Replace reference symbol with Variable and add supplemental semantics
        if (argSymbol != null && (getRefTypes().contains(argSymbol.getType()))) {
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
  private boolean checkRequiredArgsExist(String outerArgName, Set<String> calledArgumentKeys, ToolParameterV2 targetParameters, StringBuilder currentErrorMessage) {
    boolean valid = true;
    for (String paramName : targetParameters.getProperties().keySet()) {
      if (targetParameters.getRequired().contains(paramName)) {
        if (!calledArgumentKeys.contains(paramName)) {
          valid = false;
          currentErrorMessage.append("- One of the ").append(outerArgName).append(" fields was missing an entry value for the required \"").append(paramName).append("\" argument\n");
        }
      }
    }
    return valid;
  }

  //TODO: in subclass?
  private boolean processFocusedTemplate(String templateArgName, Object templateValue, Symbol user, Symbol agentTeam, StringBuilder errorMessage) {
    ChatCollection collection = getChatCollection(user,agentTeam);
    if (collection != null) {
      ChatContext context = collection.currentChat;
      Map<String, Object> templateMap = null;
      if (templateValue instanceof String) {
        try {
          templateMap = new Gson().fromJson((String) templateValue, new TypeToken<LinkedTreeMap<String, Object>>() {
          }.getType());
          templateMap.values().removeIf(Objects::isNull);
        } catch (JsonSyntaxException e) {
          errorMessage.append("- The supplied definition for the \"").append(templateArgName).append("\" parameter of the \"").append(textResponseTool.toolDefinition.type).append("\" call could not be parsed as a valid json string. Please reformat your call to supply a valid json string or supply an object directly. Make sure all brackets, parentheses, and quotes are matched up properly. If you are supplying an array, do not supply it as a string; supply the object directly. You do not need to speak with the user about this. Respond with the updated call.\n");
          log.error("[processToolCall] malformed json present in functionCall", e);
          return false;
        }
      } else if (templateValue instanceof Map<?, ?>) {
        templateMap = (Map<String, Object>) templateValue; //Catch and handle ClassCastException if ever encountered
      }
      if (templateMap != null) {
        // If template is fully populated on first turn -> add tool response indicating to call submission directly, set output to null, and loop
        if (context.history.messages.size() == 1) {
          int expectedSize = -1;
          //A bit more nuance necessary for task modification output - allowed to have empty or missing taskIndex list if directed at the current goal
          if (templateArgName.equals("currentTaskStatusModificationDefinition")) {
            Object taskType = templateMap.get("taskType");
            if (taskType != null && taskType.equals("current") && templateMap.containsKey("modification")) {
              return true;
            }
          }
          if (templateArgName.equals("currentScheduledDefinition")) {
            expectedSize = 4;
          } else if (templateArgName.equals("currentTaskSequenceDefinition")) {
            expectedSize = 2;
          } else if (templateArgName.equals("currentMissionDefinition")) {
            expectedSize = 1;
          } else if (templateArgName.equals("currentTaskStatusModificationDefinition")) {
            expectedSize = 1;
          } else {
            log.warn("[checkForFullyPopulatedTemplate] unexpected in-progress template argument name {}", templateArgName);
            return true;
          }
          if (templateMap.size() == expectedSize) {
            for (Object templateInnerArg : templateMap.values()) {
              if (templateInnerArg instanceof String) {
                if (((String) templateInnerArg).isEmpty()) {
                  return true;
                }
              } else if (templateInnerArg instanceof Map) {
                if (((Map) templateInnerArg).isEmpty()) {
                  return true;
                }
              } else if (templateInnerArg instanceof List) {
                if (((List) templateInnerArg).isEmpty()) {
                  return true;
                }
              } else {
                return true;
              }
            }
          } else {
            return true;
          }
          log.info("[checkForFullyPopulatedTemplate] detected full template definition in first response after a " +
                  "transfer to a focused context - recommending the LLM to directly submit the template definition.");
          // Should we rephrase the base error message or make a new one in this case?
          errorMessage.append("- It looks like your current definition of the target to be submitted is fully filled " +
                  "out. If this seems to be the case, try submitting the target submission tool call directly instead" +
                  "of responding to the user.");
          return false;
        }
      }
    }
    return true;
  }

  Predicate replaceReferencesInSemantics(Predicate semantics) {
    if (semantics.getName().equals("actionDescription")) {
      Pattern refPattern = Pattern.compile(",[^()]*?:location");
      List<String> refStrings = new ArrayList<>();
      String semanticString = semantics.toString();
      Matcher matcher = refPattern.matcher(semanticString);
      while (matcher.find()) {
        String str = matcher.group();
        refStrings.add(str.substring(1));
      }
      int maxAttempts = 3;
      for (String str : refStrings) {
        Symbol sym = Factory.createSymbol(str);
        for (int i=0; i<maxAttempts;i++) {
          try {
            Map<Symbol, List<Term>> refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateRE").argTypes(Symbol.class)).call(Map.class, sym);
            if (refs != null && refs.get(sym).size() == 1) {
              semanticString = semanticString.replaceAll(str,refs.get(sym).get(0).getName());//, refs.get(arg).get(0).get(0).getType());
              break;
            }
          } catch (TRADEException e) {
            log.warn("[generateREG] {}", str, e);
          }
        }
      }
      return Factory.createPredicate(semanticString);
    }

    List<Symbol> transformedArgs = new ArrayList<>();
    for (Symbol arg: semantics.getArgs()) {
      if (arg.isPredicate()) {
        transformedArgs.add(replaceReferencesInSemantics((Predicate) arg));
      } else {
        if (getRefTypes().contains(arg.getType())) {
          Map<Symbol, List<Term>> refs = new HashMap<>();
          Symbol transformedArg = null;
          int maxAttempts = 3;
          for (int i=0; i<maxAttempts;i++) {
            try {
              refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateRE").argTypes(Symbol.class)).call(Map.class, arg);
              if (refs.get(arg).size() == 1) {
                transformedArg = Factory.createSymbol(refs.get(arg).get(0).getName());//, refs.get(arg).get(0).get(0).getType());
                break;
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
        return "\"Sorry, an internal error occurred during your request. Please try again.\"";
      } else if (errorCount == 2) {
        return "\"Sorry, an internal error occurred during your request again. Please try one more time.\"";
      } else {
        return "\"Sorry, an internal error occurred during your request. I have reset my state, please try starting over.\"";
      }
    } else {
      if (errorCount == 1) {
        return predicate ? "and(sorryIDidNotHaveConnectionDuringYourRequest,pleaseTryAgain)" : "\"Sorry, I did not have connection during your request. Please try again.\"";
      } else if (errorCount == 2) {
        return predicate ? "and(sorryIDidNotHaveConnectionDuringYourRequest,pleaseTryOneMoreTime)" : "\"Sorry, I did not have connection during your request again. Please try one more time.\"";
      } else {
        return predicate ? "and(sorryIAmUnableToConnectToNecessaryServices,pleaseReconnectOrReachOutToSupport)" : "\"Sorry, I am unable to connect to necessary services. Please check your internet connection or reach out to support.\"";
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
    return getChatHistoryAsString(new ArrayList<>(chat.messages), -1);
  }
  public String getChatHistoryAsString(List<Message> messages, int numMessages) {
    Gson gson = new GsonBuilder().setPrettyPrinting().create();
    //return chat.getMessages().stream().map(m -> m.role + ": " + ((m.tool_calls == null || m.tool_calls.length < 1) ? m.content : gson.toJson(m.tool_calls[0]))).collect(Collectors.joining("\n"));
    if (numMessages == -1 || numMessages > messages.size()) {
      return messages.stream().map(gson::toJson).collect(Collectors.joining("\n"));
    } else {
      return messages.subList(0,numMessages).stream().map(gson::toJson).collect(Collectors.joining("\n"));
    }
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

  @TRADEService
  public boolean getLastResponseStreamed(Symbol user, Symbol agentTeam) {
    ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
    if (chatCollection != null) {
      ChatContext chatContext = chatCollection.currentChat;
      if (chatContext != null) {
        return chatContext.lastResponseStreamed;
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
  protected String toolToDescString(Tool tool, boolean includePossibleValues) {
    edu.tufts.hrilab.llm.gson.tool.ToolFunction toolFunction = tool.toolDefinition.function;
    //String jsonString = toolFunction.description.replaceAll("<parameter=(.*?)>(.*?)</parameter>", "\"$1\":$2");
    List<String> required = toolFunction.parameters.getRequired();
    String argumentsString;
    if (includePossibleValues) {
      //return new Gson().toJson(toolFunction);
      argumentsString = toolFunction.parameters.getProperties().isEmpty() ? "None" : "\n" +
              toolFunction.parameters.getProperties().entrySet().stream().map(entry ->
                                                                              " - " + entry.getKey() + " (" + (required.contains(entry.getKey()) ? "necessary" : "optional") + ")" + ", (type="+entry.getValue().getType()+"): "
                                                                              + entry.getValue().getDescription() + (entry.getValue().getPossible_values().isEmpty() ? "" : (". (possible_values = " + entry.getValue().getPossible_values()))).collect(Collectors.joining("\n"));
    } else {
      argumentsString = toolFunction.parameters.getProperties().isEmpty() ? "None" : "\n" +
              toolFunction.parameters.getProperties().entrySet().stream().map(entry ->
                      " - " + entry.getKey() + " (" + (required.contains(entry.getKey()) ? "necessary" : "optional") + ")" + ", (type="+entry.getValue().getType()+"): "
                              + entry.getValue().getDescription()).collect(Collectors.joining("\n"));
    }
    return "\nName: " + toolFunction.name + //"(" + String.join(",",toolFunction.parameters.getProperties().keySet()) + ")" +
            "\nDescription: " + toolFunction.description +
            "\nParameters:" + argumentsString;
  }

  //TODO: can specialize responses based on contextId
  @TRADEService
  public String getContextTransferFillerPhrase(String contextId, long cid, Symbol user, Symbol agent) {
    //TODO: make externally configurable
    if (contextId.equals("modifyExistingTaskStatus")) {
      return "";
    }
    //based on input/history length choose between "gathering relevant information ..." type response and simple "okay" type
    boolean additionalInformationHeuristic = false;
    ChatCollection chatCollection = getChatCollection(user, agent);
    if (chatCollection != null) {
      ChatContext chatContext = chatCollection.pendingFocusedChats.get(contextId);
      if (chatContext != null) {
        int numMessages = chatContext.history.messages.size();
        if (numMessages > 3) {
          int numWordsInLastMessage = chatContext.history.messages.get(chatContext.history.messages.size() - 4).content.length();
          if (numMessages > 6 || numWordsInLastMessage > 15) {
            additionalInformationHeuristic = true;
          }
        }
      }
    }
    if (additionalInformationHeuristic) {
      int option = new Random().nextInt(4);
      return switch (option) {
        case 0 -> "Gathering all relevant details.";
        case 1 -> "Processing your instruction.";
        case 2 -> "Setting up your task.";
        case 3 -> "Initializing your task.";
        default -> "Okay";
      };
    } else {
      int option = new Random().nextInt(2);
      return switch (option) {
        case 0 -> "Just a second.";
        case 1 -> "One moment please.";
        default -> "Okay";
      };
    }
  }

  //TODO: define in/load from external txt files?
  //////////////////////////////////////////////////////////////////
  /// Default prompt variable definitions shared across contexts ///
  //////////////////////////////////////////////////////////////////
  protected String getBaseRoleDescription() {
    return "You are %addressee, the natural language surface of an assistive agent with an associated back-end system.\n";
  }

  //TODO: Don't believe questions need ot be included because should be in questionAnswer context
  protected String getBaseContextualInformation() {
    return "- OUTSTANDING QUESTIONs the back-end is awaiting a response for:\n%questionList";
  }

  protected String getBaseResponseConstraints() {
    return "- Use only tool descriptions, tool arguments, and tool results for robot- or system-specific claims.\n" +
            "- Respond to the user only by using the stillGatheringInformation tool.\n" +
            "- Never make assumptions in order to answer any questions when you are missing context.\n" +
            "- Never tell the user you can or will do something for them unless it is by making a specific tool call. The back-end controls all interactions with the real-world.\n" +
            "- Do not use profanity or engage in controversial discussion.";
  }

  protected String getBaseResponseStyle() {
    return "- Be concise, calm, and soft-spoken.\n" +
            "- Respond with one short spoken sentence.\n" +
            "- No newlines\n" +
            "- No numbered lists\n" +
            "- No quoted phrases\n" +
            "- No unnecessary restatement of user input\n" +
            "- No follow-up questions unless clarification is required.\n" +
            "- Do not list more than 5 things at a time unless explicitly asked to.";
  }

  protected String getFocusedResponseConstraints() {
    return "- Do not go off topic or attempt to help the user with anything other than the stated goal.\n" +
            "- Do not invent missing values just to fill in the current template definition.";
  }

  protected String getFocusedResponseStyle() {
    return "- Do not restate or confirm details you have just been told when asking for additional information.\n" +
            "- Ask only for the next missing piece of information when leading the conversation.";
  }

  protected String getFocusedResponsePolicy() {
    return "- If the user indicates they are finished and the definition is complete, call %templateSubmissionTargetName.\n" +
            "- If the task status modification definition is fully filled out, call %templateSubmissionTargetName immediately.\n" +
            "- If the user asks what you currently understand, answer based on the %templateArgumentName.\n" +
            "- If the user does not want to modify a task's status or seems confused about what is happening, call endConversation.\n" +
            "- If the user says bye, leaves, or no longer wants to continue, call endConversation.\n" +
            "- If the user asks to pause this interaction, call suspendCurrentInteraction.\n" +
            "- If the user asks to cancel this interaction, call cancelCurrentInteraction.\n" +
            "- Otherwise, call stillGatheringInformation and include a sensible response which follows the Style guidelines" +
            " and is aligned with your goal. Update %templateArgumentName as the user adds, corrects, removes, or replaces information.";
  }

  protected String getFocusedFinalInstruction() {
    return "You will first be presented with a conversation history containing the instruction causing you to believe the " +
            "user wanted to modify an assigned task's status. This history may contain pertinent information about what task" +
            " should be modified and what modification to perform. Extract and include every single piece of information " +
            "found in your initial response's scheduled task definition. If the definition is clear and complete immediately, " +
            "you may call %templateSubmissionTargetName immediately without further conversation. If it seems that this " +
            "context was triggered incorrectly based on the past history, you may call endConversation immediately without further conversation.";
  }
}