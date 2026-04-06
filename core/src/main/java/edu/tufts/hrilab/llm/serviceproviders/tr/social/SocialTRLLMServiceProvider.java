package edu.tufts.hrilab.llm.serviceproviders.tr.social;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.common.reflect.TypeToken;
import com.google.gson.Gson;
import com.google.gson.internal.LinkedTreeMap;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.LLMServiceProvider;
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
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
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
   *      //TODO: evolve how these "focused chat contexts" work based on desired functionality which comes up moving
   *              forward. Currently set up to exit after hitting a tool call, though I can see a world where we want
   *              to constrain/modify LLM behavior/targets for an extended period.
   *      TODO: (Ava) tourTeaching
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

  /**
   * Explicitly holding REPLY Tool set separate because they will conditionally be added to outgoing parse requests as
   *  targets based on current system questions (explicitly represented by the DialogueComponent)
   */
  Map<String,Tool> replyToolset;

  /**
   * For "focused" dialogue contexts; Tool target to allow the user to temporarily back out of the specific context into
   * the base, standard "nlu" context to give one off instructions. Providing internal, standard functionality to
   * support this without forcing the user to understand how to explicitly suspend and resume these types of goals in
   * order to give other instructions.
   *
   * //TODO: not sure how well the LLM actually handles this target. Needs more testing and development
   */
  Tool offContextSubmissionTool;

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
    //TODO: Once everything is in place and doing final polish, update these descriptions and names for best performance
    offContextSubmissionTool = new Tool(new ToolDefinition("offContextSubmission",
            "Call this function when it seems like the user is attempting to instruct you to do something out " +
                    "of context for your current conversation. Common examples would be asking to cancel or suspend your " +
                    "current task.",new ArrayList<>()), UtteranceType.INSTRUCT);
    Tool focusedEndConversationTool = new Tool(new ToolDefinition("endConversation",
            "End the conversation with the user. This is usually desired in two main cases. 1. The user no" +
                    " longer wants to perform your shared task. Or 2. The user has said goodbye or needs to leave.",
            new ArrayList<>()),
            UtteranceType.INSTRUCT
    );
    //TODO: give signal in prompt that a task has been superseded/in offtopic state
    Tool offtopicEndConversationTool = new Tool(new ToolDefinition("endConversation",
            "Call this function when the user signals that they want to return to your previous task or no " +
                    "longer want to give you a new instruction.",
            new ArrayList<>()),
            UtteranceType.INSTRUCT
    );
    Tool defaultGoalInfoTool = new Tool(new ToolDefinition("getCurrentPendingScheduledGoalInformation",
            "You report what your currently pending/upcoming tasks are. An example of instructing this could" +
                    " be \"what are you doing after your current task.\"",
            List.of(new ToolParameterHelper("supersededGoal","boolean",List.of("false"),"",true))),
            UtteranceType.QUESTION);
    Tool offtopicGoalInfoTool = new Tool(new ToolDefinition("getCurrentPendingScheduledGoalInformation",
            "You report what your currently pending/upcoming tasks are. An example of instructing this could" +
                    " be \"what are you doing after your current task.\"",
            List.of(new ToolParameterHelper("supersededGoal","boolean",List.of("true"),"",true))),
            UtteranceType.QUESTION);
    itkActionNames.add("getCurrentPendingScheduledGoalInformation");

    //Brainstorm: Add an argument which explicitly tracks current \"best guess\" parse at each step? (Even if not fully parameterized)
    //  ^ Think this could be helpful to provide focused architecture messages guiding interactions, but may be too hard
    //    of a task or confuse the LLM into calling Tools without proper full parameterization or ...
    textResponseTool = new Tool(new ToolDefinition("stillGatheringInformation",
            "You are still gathering information before being able to submit an accurate, fully defined " +
                    "instruction and need to speak with the user to gather missing information.",
            List.of(new ToolParameterHelper("response","string",new ArrayList<>(),
                    "What you want to say to the user", true))),
            UtteranceType.INSTRUCT);
    //TODO: add "socialInteraction" analog which does the same thing under the covers?

    //// Gather Tool Sets and System Prompts ////
    // Base "nlu" tool set:
    //  - INSTRUCT targets
    //  - LLM<->Diarc ITK targets
    //  - (done dynamically at call-time) relevant REPLY targets
    //  - Generic endConversation target (Removing for now - TODO: have base context history time out after T time with no interaction)
    List<Tool> baseNLUTools = new ArrayList<>(instructTools);
    baseNLUTools.addAll(itkTools);
    //baseNLUTools.add(defaultEndConversationTool);
    baseNLUTools.add(defaultGoalInfoTool);
    baseNLUTools.add(textResponseTool);
    chatTemplates.put("nlu", new ChatTemplates(getNluSystemPrompt(), baseNLUTools));

    // Universal "focused" tool set:
    //  - Template specific targets (gathered at call-time based on entry corresponding to root action name - should be added by subclass)
    //  - Base LLM<->Diarc ITK targets - TODO: temporarily removed - should these be available or not? Only a specific subset?
    //  - relevant REPLY targets - TODO: temporarily removed, should these be allowed? Should there be a mechanism to define REPLY targets per focused context?
    //  - endConversation==cancelGoal
    //  - offContextSubmission - Should this be replaced with explicit suspend/cancelGoal targets?
    List<Tool> universalFocusedTools = new ArrayList<>();
    universalFocusedTools.add(focusedEndConversationTool);
    universalFocusedTools.add(new Tool(new ToolDefinition("suspendCurrentInteraction",
            "You pause your current task, which can be later resumed. An example of instructing this could be \"pause what you're doing.\"" +
                    "This may be instructed directly as shown in the example, or could be intended when the user wants to temporarily have a separate conversation on a different topic.",
            new ArrayList<>()),
            UtteranceType.INSTRUCT));
    universalFocusedTools.add(new Tool(new ToolDefinition("cancelCurrentInteraction",
            "You stop and fully discard your current task. An example of instructing this could be \"stop what you're doing.\"",
            new ArrayList<>()),
            UtteranceType.INSTRUCT));

    //universalFocusedTools.addAll(itkTools);
    //universalFocusedTools.add(defaultGoalInfoTool);
    //universalFocusedTools.add(offContextSubmissionTool); //Done in parse method

    // OffContext submission toolset: Everything from base context, but overriding:
    //  - getCurrentGoalInfo (to hide the "offContextSubmission" action / explicitly handle reasoning around the fact
    //                        that the spiritual "current goal" is what was superseded by this goal)
    //  - Specific endConversation tool - description includes notion of the "superseded" task this is interrupting
    universalFocusedTools.add(textResponseTool);
    List<Tool> offTopicTools = new ArrayList<>(instructTools);
    offTopicTools.add(offtopicEndConversationTool);
    offTopicTools.add(offtopicGoalInfoTool);
    offTopicTools.add(textResponseTool);
    chatTemplates.put("offContextSubmission", new ChatTemplates(getNluSystemPrompt(), offTopicTools)); //TODO: augment system prompt

    // Validation tool set:
    //  - Just reply forms, which are added dynamically (yes,no,abort)
    // Q: anything else needed? ITK? Or just always force validation questions to be addressed immediately
    chatTemplates.put("validationnlu", new ChatTemplates(getValidationNLUSystemPrompt(), new ArrayList<>()));

    // NLG tool sets: none
    chatTemplates.put("nlg", new ChatTemplates(getNlgSystemPrompt(), new ArrayList<>()));
    chatTemplates.put("validation", new ChatTemplates(getValidationNLGSystemPrompt(), new ArrayList<>()));

    // Focused context toolsets
    List<Tool> scheduleGoalTools = new ArrayList<>();
    scheduleGoalTools.add(new Tool(new ToolDefinition("scheduled",
            "Schedule the discussed task with all known timing information", List.of(
                new ToolParameterHelper("task","string",new ArrayList<>(),"The task to be performed. Please refer to the Action List in order to understand acceptable values and forms.",true),
                new ToolParameterHelper("start","string",new ArrayList<>(),"When to first execute the task. If start(immediately) is supplied as the value, that means the task will be submitted immediately upon validation. Otherwise, valid forms are a relative time represented by start(<number>,<timeunit>) or an absolute time represented by start(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss). For example, the relative time of 2 hours from now == start(2,HOURS). The absolute time of noon == start(12-00-00). You are only allowed to call this function and provide any value for this argument, empty string or not, if the user has explicitly told you when the task should start.",true),
                new ToolParameterHelper("period","string",new ArrayList<>(),"On what interval to repeat execution of the task. if period(nonrepeating) is supplied as the value, that means that the task will only be performed once and not repeated. Otherwise, valid forms a relative time represented by period(<number>,<timeunit>) or an absolute time represented by period(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss). For example, the relative time of 30 minutes from now == period(30,MINUTES). The absolute time of 6am == period(06-00-00). You are only allowed to call this function and provide any value for this argument, empty string or not, if the user has explicitly told you how frequently the task should repeat.",true),
                new ToolParameterHelper("end","string",new ArrayList<>(),"Valid only when period is also defined with a value other than period(nonrepeating): When to stop repeating execution the task. If end(indefinitely) is provided as the value, that means the task will continue repeating execution on the defined period until it is explicitly canceled by the user. Otherwise, valid forms are a relative time represented by end(<number>,<timeunit>) or an absolute time represented by end(date-format). <number> must take an Integer value. <timeunit> must be a value of the Java TimeUnit enum. <date-format> must be a DateFormat following HH-mm-ss or MM-dd-yyyy HH-mm-ss (military time for HH-mm-ss). For example, the relative time of in a quarter of an hour == end(15,MINUTES). The absolute time of 5:30pm == end(17-30-00). You are only allowed to call this function and provide any value for this argument, empty string or not, if the user has explicitly told you when the task should stop repeating.",true)
            )),
                    UtteranceType.INSTRUCT
            ));
    scheduleGoalTools.addAll(universalFocusedTools);
    chatTemplates.put("scheduleTask", new ChatTemplates(Prompts.getPrompt("socialava/scheduleGoal"), scheduleGoalTools));

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
    //List<String> patrolPaths = getPatrolPaths();
    List<String> videos = getVideoNames();
    List<String> images = getImageNames();
    //String patrolPathList = String.join(":path\n", patrolPaths);
    //if (!patrolPathList.isEmpty()) {
    //  patrolPathList += ":path\n";
    //}
    String locationList = String.join(":location\n", locations);
    if (!locationList.isEmpty()) {
      locationList += ":location\n";
    }
    String videoList = String.join(":video\n", videos);
    if (!videoList.isEmpty()) {
      videoList += ":video\n";
    }
    String imageList = String.join(":image\n", images);
    if (!videoList.isEmpty()) {
      imageList += ":image\n";
    }
    //vars.put("%patrolPathList", patrolPathList);
    vars.put("%locationList", locationList);
    vars.put("%videoList", videoList);
    vars.put("%exampleVideo", videos.isEmpty() ? "test" : videos.get(0));
    vars.put("%exampleImage", images.isEmpty() ? "test" : images.get(0));
    vars.put("%imageList", imageList);
    vars.put("%exampleLocation", locations.isEmpty() ? "test location" : locations.get(0));
    //promptVars.put("%examplePath", patrolPaths.isEmpty() ? "test path" : patrolPaths.get(0));
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
      promptVars.put("%worldState", getWorldStateString(agent));
      //TODO: only include goal summary as architecture itk action rather than putting in prompts explicitly?
      promptVars.put("%goals", getFullGoalsSummary(agent, false));
      promptVars.put("%questionList", getCurrentQuestionsString(agent));
      ChatCollection chatCollection = getChatCollection(user,agent);
      if (chatCollection != null) {
        ChatContext chatContext = chatCollection.currentChat;
        if (chatContext != null && chatContext.chatState == ChatState.VALIDATION) {
          //TODO: include tool information, not parsed semantics
          promptVars.put("%toolUpForValidationDescription",chatContext.utteranceUpForValidation.getSemantics().toString());
        }
      }
    } else {
      promptVars.put("%goals", getFullGoalsSummary(agent,false));
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
      // Pull correct template and update system message and tool targets based off current state
      if (chatContext.chatState == ChatState.VALIDATION) {
        systemTemplate = new Prompt(chatTemplates.get("validationnlu").systemMessageTemplate.getText());
        tools = new ArrayList<>();
        tools.add(textResponseTool.toolDefinition);
        // ValidationNLU has chat history as part of system message. Otherwise, content in the history will be parsed as validation responses.
        //  Creating a new chat just for validation, but maintaining proper history in Chat stored in this ChatContext in case of negative validation and continued interaction
        chat = new Chat();
        promptVars.put("%chatHistoryAsString", chatContext.history.getMessages().stream().map(m -> m.role + ": " + m.content).collect(Collectors.joining("\n")));
        chatContext.history.addUserMessage(utterance.getWordsAsString());
      } else {
        ChatTemplates chatTemplates = this.chatTemplates.get(chatContext.contextID);
        if (chatTemplates == null) {
          log.error("[parseUtteranceLLMService] could not find chat template for contextID {}", chatContext.contextID);
          return utterance;
        }
        systemTemplate = new Prompt(chatTemplates.systemMessageTemplate.getText());
        tools = new ArrayList<>(chatTemplates.toolDefinitions);
        chat = chatContext.history;
      }

      if (chatContext.chatType == ChatType.FOCUSED && chatContext.chatState != ChatState.VALIDATION) {
        // Add special offContextSubmission Tool if in a focused context
        //tools.add(offContextSubmissionTool.toolDefinition);
      } else {
        // TODO: Had written this way, but is there a reason to explicitly hide these tools when in a focused context?
        // Add all matching system question response tools based off of expected reply semantics
        tools.addAll(getQuestionResponseToolDefinitions(agentTeam));
      }

      //TODO: Error handling if chat is null

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
      //utterance.setSemantics(Factory.createPredicate("error",utterance.getAddressee().toUntypedString(),generateErrorResponse(false,user,agentTeam,true)));
      //utterance.setNeedsValidation(false);
      return utterance;
    } else {
      resetErrorCount(false, user, agentTeam);
    }

    log.info("[parseUtteranceLLMService] num tool calls: {}, text response: {}", response.toolCalls.size(), response.text);

    synchronized (currentChatCollections) {
      // Check for external parser notification between grabbing input lock and this synchronized block
      if (!chatCollection.shouldProcess(utterance)) {
        chatCollection.signalInputProcessed(utterance); //Failsafe - revisit input lock grab
        return utterance;
      }

      //TODO: handling multiple tool calls present
      //TODO: handling text response and tool call(s) present
      //// Handle Responses
      if (response.toolCalls.isEmpty()) {
        // Handle text response - always handled the same regardless of state (superfluous dialogue, extended parse attempt, extended validation)
        //    Add robot message to chat and return generateResponse semantics with no validation required
        return handleTextResponse(response, utterance, chatContext);
      } else {
        if (chatContext.chatState == ChatState.VALIDATION) {
          // Validation Tool Call - return appropriate reply semantics and modify chatCollection state as needed (possible
          //  to terminate and/or transfer between base and focused contexts in this case)
          return handleValidationResponse(response, utterance, chatCollection);
        } else {
          // Tool Call - extended parse recovery on invalid parse, or enter validation of valid parse
          return handleToolResponse(response, utterance, chatCollection);
        }
      }
    }
  }

  /**
   * Method for handling Completion responses containing Tool calls, 3 main cases:
   *  - Base: Return an Utterance populated with the Tool's underlying semantics. In the case of an external cache hit,
   *          the context will be discarded and handleValidatedToolResponse will be called from the corresponding dialogue history update.
   *  - LLM ITK - will return semantics for goal without validation (Underlying semantics should match a SKIPPENDING priority goal which calls {@link #addToolResponse(Symbol, Symbol, String)})
   *  - Invalid parse - Add a system error message to the Chat, call the completion again, and return a text response to the user with the result
   *  - *NEW* Stand-in tool(s) for text responses - Experimenting targeting a specific tool to give text responses rather than using
   *      handleTextResponse above with promising initial success. Allowing the LLM to target text responses or tool calls was giving it a
   *      hard time calling tools rather than just saying it performed the underlying functionality.
   */
  public Utterance handleToolResponse(Completion llmResponse, Utterance inputUtterance, ChatCollection chatCollection) {
    if (llmResponse.toolCalls.size() > 1) {
      //TODO
      log.warn("[handleToolResponse] Multiple tool calls returned -> TODO handle both. Only using first...");
    }
    ToolCall toolCall = llmResponse.toolCalls.get(0);
    log.info("[handleToolResponse] " + toolCall);

    ChatContext chatContext = chatCollection.currentChat;

    //// Thought we would have ideally wanted this in the history, but found that LLM would extract tool calls from
    ////  the history as targets, even if they were taken out of the system prompt based on a state change
    //chatContext.history.addRobotMessage("", List.of(toolCall));

    //// Check that tool response is valid
    ToolFunction functionCall = toolCall.function;
    Tool correspondingTool = null;
    try {
      if (chatContext.chatType == ChatType.FOCUSED) {
        if (functionCall.name.equals("offContextSubmission")) {
          correspondingTool = offContextSubmissionTool;
        } else {
          correspondingTool = chatTemplates.get(chatContext.contextID).tools.get(functionCall.name);
        }
      } else {
        if (chatContext.chatType == ChatType.OFFTOPIC) {
          correspondingTool = chatTemplates.get("offContextSubmission").tools.get(functionCall.name);
        } else {
          correspondingTool = chatTemplates.get("nlu").tools.get(functionCall.name);
        }
        if (correspondingTool == null) {
          correspondingTool = replyToolset.get(functionCall.name);
        }
      }
    } catch (NullPointerException e) {
      log.error("[handleToolResponse] error finding Tool definition for called tool found in Completion response", e);
    }

    // Internal LLM error - should there be LLM specific error responses in this case?
    if (correspondingTool == null) {
      handleErrorResponse(inputUtterance, true, true);
      return inputUtterance;
    }

    // Process the tool call and supplied arguments - output utterance will be null and baseToolResponse will be updated
    //  with error details in case any invalid response was supplied by the LLM
    StringBuilder baseToolResponse = new StringBuilder("You interpreted the user's command as ").append(toolCall).append(". This result contains missing or invalid information. Please succinctly let them know what you thought they were trying to instruct, including the task itself and any information which was supplied correctly, and summarize the following errors:\n");
    Utterance outputUtterance = processToolCall(functionCall, correspondingTool, inputUtterance, baseToolResponse);

    //// Handle processed response
    if (outputUtterance != null && outputUtterance.getSemantics() != null) {
      //Valid parse - proceed to validation

      // Experimental text response tool stand-in (described above)
      if (toolCall.function.name.equals("stillGatheringInformation")) {
        if (!outputUtterance.getSemantics().isPredicate() || ((Predicate)outputUtterance.getSemantics()).getArgs().size() < 2) {
          log.error("[handleToolResponse] malformed stillGatheringInformation response");
          handleErrorResponse(inputUtterance, true, true);
          return inputUtterance;
        }
        llmResponse.text = ((Predicate)outputUtterance.getSemantics()).get(1).toString();
        resetErrorCount(true, inputUtterance.getSpeaker().toUntyped(), inputUtterance.getAddressee().toUntyped());
        return handleTextResponse(llmResponse,inputUtterance,chatContext);
      }

      //// Domain specific postprocessing to correct common mistakes
      correctCommonNLUMistakes(outputUtterance);

      //// Universal postprocessing for accepted cases
      // TODO: Reevaluate if these are in fact conventions we want to universally accept / how we want to do these things
      String toolName = outputUtterance.getSemantics().getName();

      // "Off topic" Submission targeting suspend or cancel current goal - should be applied to the superseded dialogue
      //  goal rather than the currently active "offContextSubmission" wrapper
      // TODO: If using, make sure Validation NLG hides this distinction in this case - whether using a different action, providing examples
      if (chatContext.chatType == ChatType.OFFTOPIC) {
        if (toolName.equals("suspendSystemGoalsBlocking") || toolName.equals("cancelSystemGoals")) {
          if (chatCollection.suspendedChats.isEmpty()) {
            log.error("[handleToolResponse] targeted suspend or cancel in OFFTOPIC state, but no suspended focused context exists. Simply abandoning instruction");
            toolName = "endConversation";
          } else {
            String contextId = chatCollection.suspendedChats.get(0).contextID;
            Predicate pred = Factory.createPredicate(contextId, inputUtterance.getAddressee());
            long gid = -1;
            try {
              gid = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalId").argTypes(Predicate.class)).call(long.class, pred);
            } catch (TRADEException e) {
              log.error("[handleToolResponse] exception getting goal ID for suspended focused goal: {}", pred, e);
            }
            //// Not currently safe
            //if (gid == -1) {
            //  log.warn("[handleToolResponse] could not find goalId for suspended focused chat");
            //  //TODO; Any associated cleanup necessary?
            //  inputUtterance.setSemantics(Factory.createPredicate("error","internalErrorOccurred","couldNotStopDesiredGoal"));
            //  inputUtterance.setNeedsValidation(false);
            //  return inputUtterance;
            //} else {
              if (toolName.equals("suspendSystemGoalsBlocking")) {
                outputUtterance.setSemantics(Factory.createPredicate("suspendGoal", inputUtterance.getAddressee(), Factory.createSymbol(String.valueOf(gid))));
              } else {
                outputUtterance.setSemantics(Factory.createPredicate("cancelGoal", inputUtterance.getAddressee(), Factory.createSymbol(String.valueOf(gid))));
              }
            //}
          }
        }
      }

      // Temporary special wrappers - revisit when reintroducing "offContext" functionality
      if (chatContext.chatType != ChatType.BASE) {
        if (toolName.equals("endConversation") || toolName.equals("cancelCurrentInteraction")) {
          //TODO: make sure validation NLG handles this correctly
          outputUtterance.setSemantics(Factory.createPredicate("cancelSystemGoals", inputUtterance.getAddressee()));
        } else if (toolName.equals("suspendCurrentInteraction")) {
          outputUtterance.setSemantics(Factory.createPredicate("suspendSystemGoalsBlocking", inputUtterance.getAddressee()));
        }
      }

      // Proceed to validation if not architecture ITK call
      if (outputUtterance.getType() == UtteranceType.QUESTION) {
        outputUtterance.setType(UtteranceType.INSTRUCT);
        outputUtterance.setNeedsValidation(false);
      } else {
        if (outputUtterance.needsValidation()) {
          chatContext.chatState = ChatState.VALIDATION;
          chatContext.utteranceUpForValidation = outputUtterance; //copy?
        } else {
          // noConfirmation case - deferred until dialogue history update in case of external parser hit
          //handleValidatedToolCall(outputUtterance, chatCollection);
        }
      }

      resetErrorCount(true, inputUtterance.getSpeaker().toUntyped(), inputUtterance.getAddressee().toUntyped());
      return outputUtterance;
    } else {
      // Invalid parse - engage in recovery in the form of an extended dialogue interaction

      // Previously found that any (not to mention invalid) tool calls in the history could bias the LLM to retarget that format.
      //  Removing from history and instead supplying the information in the error message itself.
      // TODO: seems like the LLM doesn't react to any \"architecture\" role messages - see if we can explicitly prompt
      //  it to recognize diarc as an additional entity/user
      //chatContext.history.addRobotMessage("", llmResponse.toolCalls);
      //chatContext.history.addMessage("architecture", baseToolResponse.toString());
      chatContext.history.addMessage("tool", baseToolResponse.toString());

      // Re-call completion for text response
      Completion errorResponse = callChatCompletion(chatContext.history, List.of(textResponseTool.toolDefinition));
      if (errorResponse == null) {
        handleErrorResponse(inputUtterance, true, false);
        return inputUtterance;
      } else {
        if (errorResponse.toolCalls.isEmpty()) {
          resetErrorCount(true, inputUtterance.getSpeaker().toUntyped(), inputUtterance.getAddressee().toUntyped());
          return handleTextResponse(errorResponse, inputUtterance, chatContext);
        } else {
          return handleToolResponse(errorResponse,inputUtterance,chatCollection);
        }
      }
    }
  }

  /**
   * Method for handling Completion responses containing Text responses. Add directly as assistant message to history
   *  and return generateResponseFromString semantics with needsValidation set to false
   */
  public Utterance handleTextResponse(Completion llmResponse, Utterance inputUtterance, ChatContext chatContext) {
    String responseString = llmResponse.getText();

    // Updating response from dialogue history now - in case this parse result is not used
    //chatContext.history.addRobotMessage(responseString);
    Predicate generateResponsePred = Factory.createPredicate("generateResponseFromString",inputUtterance.getAddressee().toString(), inputUtterance.getSpeaker().toString(), "\""+responseString+"\"");
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
  public Utterance handleValidationResponse(Completion llmResponse, Utterance inputUtterance, ChatCollection chatCollection) {
    if (llmResponse.toolCalls.size() > 1) {
      log.warn("[handleValidationResponse] Multiple tool calls returned -> TODO handle both. Currently only using first...");
    }
    ToolCall toolCall = llmResponse.toolCalls.get(0);
    log.info("[handleValidationResponse] " + toolCall);

    // Tool -> semantics. Yes/No/Abort
    //  Context specific prompt - no special cases. Set utterance semantics and terminate chat if validated or aborted
    //  Base prompt - special cases based on whether context specific prompt exists or not
    Utterance.Builder output = new Utterance.Builder(inputUtterance);
    ChatContext chatContext = chatCollection.currentChat;

    // Experimental text response tool stand-in (described above)
    if (toolCall.function.name.equals("stillGatheringInformation")) {
      return handleToolResponse(llmResponse, inputUtterance, chatCollection);
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
      chatCollection.currentChat = null;
    }
    return output.build();
  }

  private void handleValidatedToolCall(Utterance validatedUtterance, ChatCollection chatCollection) {
    handleValidatedToolCall(validatedUtterance.getSemantics().getName(), chatCollection);
  }

  /**
   * Method to perform relevant side effects for a validated (or noConfirmation) tool call
   */
  private void handleValidatedToolCall(String toolName, ChatCollection chatCollection) {
    ChatContext chatContext = chatCollection.currentChat;

    // TODO: revisit offContextSubmission handling
    if (toolName.equals("offContextSubmission")) {
      //Make a copy for new submission context
      ChatContext copy = new ChatContext(chatContext);
      copy.submissionTime = System.currentTimeMillis();
      chatCollection.pendingFocusedChats.put(toolName, copy);
      chatContext.chatState = ChatState.PARSE;

      //TODO: test and iterate on this. What chat state do we want when entering offcontext submission case?
      //  Do we want the suspended conversation history to be in context? If so, in what form?
      //  Add architecture message explaining "offcontext" status? and/or summary of suspended context?
      //chatContext.history = new Chat();
      //chatContext.history.addMessage();
      copy.history.addMessage("tool", "The user has signalled they are attempting to give a new instruction, independent of the target of the original suspended interaction. If they have already supplied enough information about the new instruction to call a function below, please do so. Otherwise, please continue with the system task to figure out what the new instruction is.");

      //TODO: better way to do this w.r.t synchronization/safety?
      //Don't remove current state so it can be handled on suspend
    } else {
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

      //TODO: shouldn't be here - don't need to validate ITK, but hitting this in noConfirmation case now
      if (!itkActionNames.contains(toolName)) {
        // Temp workaround until reintroducing offContextSubmission - don't terminate context here if suspending a focused context
        if (!(chatCollection.currentChat.chatType == ChatType.FOCUSED && toolName.equals("suspendSystemGoalsBlocking"))) {
          // Wait on two-way lock so thread waiting on focused chat context to terminate can get any relevant result information
          chatCollection.currentChat.signalCompletion();
          chatCollection.currentChat = null;
        } else {
          // Else reset to parse state for when focused context is resumed
          chatCollection.currentChat.chatState = ChatState.PARSE;
        }
      }
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
        return chatContext.containsLLMParse; // exclude error responses?
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
    currentContext.history.addMessage("tool", "The user has just informed you that your previous function selection targeting " + semantics + " was not what they intended to instruct. Use the recent user messages to determine one of two ways to proceed: 1. If the user's message includes additional information clarifying their original intention, then you may attempt an updated function call targeting the new intention. 2. Otherwise, please continue with your original task by conversing with the user to resolve the misunderstanding.");

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
        promptVars.put("%chatHistoryAsString",chatContext.history.getMessages().stream().map(m -> m.role+": "+m.content).collect(Collectors.joining("\n")));
        if (chatContext.chatState == ChatState.VALIDATION) {
          chatTemplates = this.chatTemplates.get("validation");
        }
      }
    } else {
      promptVars.put("%chatHistoryAsString","<There is no relevant chat history>");
    }
    if (chatTemplates == null) {
      chatTemplates = this.chatTemplates.get("nlg");
    }

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

    // (Previously specific workaround for getting acceptable NLG for learned action instructions - revisit when adding learned actions back in)
    //TODO: ideally this check and override would occur somewhere else, but currently have learned action information here nicely
    //if (outgoingUtterance.getSemantics().getName().equals("mean")) {
    //  checkForLearnedActionInstructionValidation(outgoingUtterance);
    //}

    return utterance;
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
          //  TODO: see if performance is good enough just doing this, or if a tool response to the initial call saying
          //    to use the history towards the updated task explicitly works better
          chatContext.chatState = ChatState.PARSE;
          collection.signalInputProcessed();
          chatContext.contextID = contextIdentifier;
          collection.currentChat = chatContext;
          chat = chatContext.history;
          if (!contextIdentifier.equals("offContextSubmission")) {
            // Found issues with changing prompt but maintaining history in Message list - LLM interprets it as future input
            //   regardless of explanations in system prompt and will attempt to call tools based on that history. Including
            //  in system prompt itself and resetting message list
            promptVars.put("%focusedChatHistoryAsString", chat.getMessages().stream().map(m -> m.role + ": " + m.content).collect(Collectors.joining("\n")));
            chat.setMessages(new ArrayList<>());
          }
          //else TODO: should just have slightly modified prompt for offtopic which includes superseded goal info
          chat.setSystem(contextSpecificTemplate.systemMessageTemplate.apply(promptVars));

          // Update chat status
          if (contextIdentifier.equals("offContextSubmission")) {
            chatContext.chatType = ChatType.OFFTOPIC;
          } else {
            chatContext.chatType = ChatType.FOCUSED;

            // Check time elapsed since storage, if greater than N, add architecture message
            long timeSinceSubmission = (System.currentTimeMillis() - chatContext.submissionTime) / 1000;
            //TODO: what amount of time here? + Test if/how this actually modified response once everything else is settled
            if (timeSinceSubmission > 15) {
              chatContext.history.addMessage("tool", String.format("%d seconds have passed since the user gave you the previous instruction. In your next response, unless you are calling a function, please briefly remind the user that they previously instructed you to perform the current task.", timeSinceSubmission));
            }
          }
          chatContext.cid = cid;

          // Grab tools from corresponding template
          //tools = new ArrayList<>(contextSpecificTemplate.toolDefinitions);
          //tools.addAll(getQuestionResponseToolDefinitions(agentTeam));

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
            originalUtterance = handleTextResponse(response, originalUtterance, collection.currentChat);
          } else {
            // Direct tool call on transfer
            originalUtterance = handleToolResponse(response, originalUtterance, collection);
            //TODO: if this call returns the utterance unmodified, needs to be an error case
          }

          if (chatContext.getNumParseErrors() == 0 && externalErrorCount == 0) {
          //if (!originalUtterance.getSemantics().getName().equals("error")) {
            // Need to store utterance whose semantics we need to update in order for proper side effect to occur in the NLU pipeline
            contextTransferResponseUtterances.put(cid, originalUtterance);
          } else {
            originalUtterance.setType(UtteranceType.UNKNOWN); //Revisit whole commit - decide on error semantics or figure out better way to handle this
          }
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
        //TODO: is this now very much possible if the offcontext submission immediately goes through?
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
   * @return validated semantic result of this focused chat context - currently only relevant for offContextSubmission
   */
  @OnInterrupt(onCancelServiceCall = "interruptWaitOnFocusedChat(?user,?agentTeam,?cid)", onSuspendServiceCall = "interruptWaitOnFocusedChat(?user,?agentTeam,?cid)")
  @Action
  @TRADEService
  public Predicate waitOnFocusedChat(Symbol user, Symbol agentTeam, long cid) {
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
        return null;
      } else if (collection.currentChat == null || collection.currentChat.cid != cid) {
        //TODO: is this now very much possible if the offcontext submission immediately goes through?
        log.warn("[waitOnFocusedChat] no matching chat for pair and id {}/{} {}. This should only be possible if the initial instruction contained all relevant information", speaker, addressee, collection);
        return null;
      }
      chatContext = collection.currentChat;
    }

    // Wait for focused context to terminate
    Predicate resultOfFocusedContext = chatContext.waitForCompletion();

    Utterance utt = contextTransferResponseUtterances.remove(cid);
    //TODO: see if can do offContext cancel/suspendCurrentGoal transformation shenanigans here instead

    if (resultOfFocusedContext == null && utt != null) {
      // If we immediately called a populated template (noConfirmation) tool target, resultOfFocusedContext will be null,
      //  but not due to failure. Only if utt is also null did we actually fail in initial focused completion.
      //TODO: is this okay for offContextSubmission? And do same in above case "no matching chat for pair"
      resultOfFocusedContext = Factory.createPredicate("dummy","response");
    }

    log.info("[waitOnFocusedChat] out method {}/{} {}", user, agentTeam, cid);

    return resultOfFocusedContext;
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

  /**
   * Utility class for Map indexing using Predicates matching on signatures regardless of bound state
   * TODO: revisit
   */
  class PredicateSignature {
    Predicate predicate;

    public PredicateSignature(Predicate predicate) {
      this.predicate = predicate;
    }

    @Override
    public boolean equals(Object o) {
      if (o == null || getClass() != o.getClass()) return false;
      PredicateSignature that = (PredicateSignature) o;
      return that.predicate.instanceOf(predicate);
    }

    @Override
    public int hashCode() {
      return Objects.hashCode(predicate == null ? "" : predicate.getName());
    }
  }
  private final Lock goalSubmissionLock = new ReentrantLock();
  private final Map<PredicateSignature, Condition> goalSubmissionMap = Collections.synchronizedMap(new HashMap<>());

  /**
   * Wait for a goal matching the supplied goalPred's form to be submitted. Return that goal's gid (-1 if timed out)
   * Utility method used when handling multiple, disjoint side effects during transfer to a focused/offcontextsubmission.
   */
  @TRADEService
  public long waitForGoalSubmission(Predicate goalPred, long timeout) {
    if (goalPred == null) {
      log.error("[waitForGoalSubmission] null goalPred supplied");
      return -1;
    }

    long startTime = System.currentTimeMillis() - 1000;

    //Start thread to wait on notification of goal submission
    Thread waitForSubmissionThread = new Thread(() -> {
      TRADEServiceInfo statusNotificationHandler;
      TRADEServiceInfo unregisterService;
      try {
        TRADEServiceInfo registerService = TRADE.getAvailableService(new TRADEServiceConstraints().name("registerGoalManagerNotification").argTypes(TRADEServiceInfo.class, NotificationType.class));
        statusNotificationHandler = TRADE.getAvailableService(new TRADEServiceConstraints().name("handleGoalUpdate").argTypes(GoalInfo.class));
        registerService.call(void.class, statusNotificationHandler, NotificationType.GOAL_STATUS_CHANGE);
        unregisterService = TRADE.getAvailableService(new TRADEServiceConstraints().name("unregisterGoalManagerNotification").argTypes(TRADEServiceInfo.class, NotificationType.class));
      } catch (TRADEException e) {
        log.error("[waitForGoalSubmission] could not register for goal status updates", e);
        return;
      }

      Condition submissionCondition;
      goalSubmissionLock.lock();
      submissionCondition = goalSubmissionMap.get(new PredicateSignature(goalPred));
      if (submissionCondition == null) {
        submissionCondition = goalSubmissionLock.newCondition();
        goalSubmissionMap.put(new PredicateSignature(goalPred),submissionCondition);
      }
      try {
        submissionCondition.await(timeout, TimeUnit.MILLISECONDS);
      } catch (InterruptedException e) {
        log.error("[waitForGoalSubmission] exception waiting on goal submission", e);
      }
      try {
        unregisterService.call(void.class, statusNotificationHandler, NotificationType.DATABASE_ADDITION);
      } catch (TRADEException e) {
        log.error("[waitForGoalSubmission] could not unregister goal status update handler", e);
        return;
      }
      goalSubmissionLock.unlock();
    });
    waitForSubmissionThread.start();

    //Check current collections to see if the goal has already been submitted
    long gid = getSubmittedGoalId(goalPred, startTime);

    if (gid == -1) {
      //No -> wait on thread and the look for goal again
      try {
        waitForSubmissionThread.join();
      } catch (InterruptedException e) {
        log.error("[waitForGoalSubmission] exception joining on goal submission thread", e);
      }
      gid = getSubmittedGoalId(goalPred, startTime);
    }
    // Commented out this branch - need to let notification handler be unregistered. (will we ever practically have
    //    long enough timeout values where this is an issue?)
    //else {
    //  //Yes -> interrupt thread. Return gid
    //  waitForSubmissionThread.interrupt();
    //}
    return gid;
  }
  @TRADEService
  public void handleGoalUpdate(GoalInfo goalInfo) {
    goalSubmissionLock.lock();
    Symbol unboundGoal = goalInfo.unboundGoal;
    if (!unboundGoal.isPredicate()) {
      return;
    }
    Condition condition = goalSubmissionMap.remove(new PredicateSignature((Predicate) goalInfo.unboundGoal));
    if (condition != null) {
      condition.signalAll();
    }
    goalSubmissionLock.unlock();
  }
  protected long getSubmittedGoalId(Predicate goalPred, long startTime) {
    try {
      List<GoalInfo> goals = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalInfos").argTypes(Predicate.class)).call(List.class,goalPred);
      for (GoalInfo info : goals) {
        if (!(info.status.isTerminated() && info.start < startTime)) {
          return info.gid;
        }
      }
    } catch (TRADEException e) {
      log.error("[getSubmittedGoalId] exception while searching for matching goal", e);
    }
    return -1;
  }

  //// General Completion / ChatCollection Utilities ////

  private Completion callChatCompletion(Chat chat, List<ToolDefinition> tools) {
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
  }

  /**
   * Method to discard the current chat context. Currently only used when validation questions expire (should be used
   * for general nonactivity timeout as well)
   * TODO: test this is safe to call in all cases
   */
  @TRADEService
  public void endChatContext(Symbol user, Symbol agentTeam) {
    synchronized (currentChatCollections) {
      ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
      if (chatCollection != null) {
        chatCollection.releaseLocks();
        chatCollection.currentChat = null;
      }
    }
  }

  //TODO: need to be careful about when to copy vs pass by ref. (overall synchronization)
  public ChatCollection getChatCollection(Symbol user, Symbol agentTeam) {
    ChatCollection collection = currentChatCollections.get(Pair.of(getUntypedSymbol(user), getUntypedSymbol(agentTeam)));
    if (collection == null) {
      log.error("[getChatCollection] no chat collection found for {}/{} pair", user, agentTeam);
    }
    return collection;
  }

  @TRADEService
  public String callChatCompletionForTextResponse(Symbol user, Symbol agentTeam, long id) {
    ChatCollection collection;
    Chat chat;
    long originalChatContextid;
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
      chat = collection.currentChat.history;
      originalChatContextid = collection.currentChat.getUid();
    }

    Completion response = callChatCompletion(chat, List.of(textResponseTool.toolDefinition));

    if (response != null) {
      synchronized (currentChatCollections) {
        if (collection.currentChat == null || collection.currentChat.getUid() != originalChatContextid) {
          log.error("[callChatCompletionForTextResponse] chat context was overwritten during completion request");
          //TODO: do we still return the response in this case - or just move on?
          return "";
        }
        if (response.getText() != null && !response.getText().isEmpty()) {
          resetErrorCount(true, user, agentTeam);
          return response.getText();
        } else if (!response.toolCalls.isEmpty()) {
          ToolCall toolCall = response.toolCalls.get(0);
          if (toolCall.function.name.equals("stillGatheringInformation")) {
            Map<String, String> calledArguments = new Gson().fromJson(response.toolCalls.get(0).function.arguments, new TypeToken<LinkedTreeMap<String, String>>() {
            }.getType());
            calledArguments.values().removeIf(Objects::isNull);
            String ret = calledArguments.get("response");
            if (ret == null) {
              collection.currentChat.incrementNumParseErrors();
              return generateErrorResponse(true, collection.currentChat.getNumParseErrors(), false);
            } else {
              resetErrorCount(true, user, agentTeam);
             return ret;
            }
          } else {
            collection.currentChat.incrementNumParseErrors();
            return generateErrorResponse(true, collection.currentChat.getNumParseErrors(), false);
          }
        }
      }
    }

    externalErrorCount += 1;
    return generateErrorResponse(false,externalErrorCount,false);
  }

  //TODO: Should ITK calls be fully synchronized between instruction to this call here? Ideally, yes, but it would be
  //  very very hard to detect any exception in the external itk method which would cause this method to not be
  //  reached. For now, if a user submits another input which terminates the chat context before this method has a
  //  chance to be called, we just discard the response.
  /**
   * Method used by "social LLM" itk behaviors (llm <-> diarc itk) to add DIARC's response to an ITK request and then
   * call the Completion endpoint with the updated Chat for a text response.
   */
  //TODO: Relate to original tool call id if keeping in history
  @TRADEService
  public String addToolResponse(Symbol user, Symbol agentTeam, String toolResponse) {
    Symbol speaker = getUntypedSymbol(user);
    Symbol addressee = getUntypedSymbol(agentTeam);
    ChatCollection collection;
    ChatContext chatContext;
    Chat chat;
    long originalChatContextid;
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
        chatContext.history.addMessage("tool", toolResponse);
        chat = chatContext.history;
        originalChatContextid = chatContext.getUid();
      }
    }

    Completion response = callChatCompletion(chat, List.of(textResponseTool.toolDefinition));

    if (response != null) {
      synchronized (currentChatCollections) {
        if (collection.currentChat == null || collection.currentChat.getUid() != originalChatContextid) {
          log.error("[addToolResponse] chat context was overwritten during completion request");
          //TODO: do we still return the response in this case - or just move on?
          return "";
        }
        if (response.getText() != null && !response.getText().isEmpty()) {
          //collection.currentChat.history.addRobotMessage(response.getText());
          return response.getText();
        } else if (!response.toolCalls.isEmpty()) {
          ToolCall toolCall = response.toolCalls.get(0);
          if (toolCall.function.name.equals("stillGatheringInformation")) {
            Map<String, String> calledArguments = new Gson().fromJson(response.toolCalls.get(0).function.arguments, new TypeToken<LinkedTreeMap<String, String>>() {
            }.getType());
            calledArguments.values().removeIf(Objects::isNull);
            String ret = calledArguments.get("response");
            if (ret == null) {
              collection.currentChat.incrementNumParseErrors();
              return generateErrorResponse(true, collection.currentChat.getNumParseErrors(), false);
            } else {
              resetErrorCount(true, user, agentTeam);
              return ret;
            }
          } else {
            collection.currentChat.incrementNumParseErrors();
            return generateErrorResponse(true, collection.currentChat.getNumParseErrors(), false);
          }
        }
      }
    }

    externalErrorCount += 1;
    return generateErrorResponse(false,externalErrorCount,false);
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
      //chatContext.contextLock.release();
      //chatContext.signalCompletion();
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
      chatContext.history.addMessage("tool", "Your interaction with the user was paused for some amount of time while you both performed other actions. Please refamiliarize the user with the task you are performing and the current state of progress.");
      collection.currentChat = chatContext;

      //chatContext.waitToProcessInput(cid);
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
        collection.currentChat = null;
        //chatContext.contextLock.release();
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
          //chatContext.contextLock.release();
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
      //String questionString = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestion").argTypes(Symbol.class)).call(String.class,agent);
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
  public String getFullGoalsSummary(Symbol agent, boolean offContext) {
    List<Predicate> currentGoalPredicate = getCurrentGoals(agent);
    List<Predicate> pendingGoalPredicates = getPendingGoals(agent);
    List<Predicate> scheduledGoals = getScheduledGoals(agent);

    StringBuilder stringBuilder = new StringBuilder("At any time, you may have up to one current task, an arbitrary amount of upcoming tasks pending completion of the current task, and an arbitrary amount of scheduled tasks which will be submitted to the pending task queue at certain times. The current task may be active or temporarily suspended.\n");

    if (!currentGoalPredicate.isEmpty()) {
      if (offContext && !pendingGoalPredicates.isEmpty()) {
        //TODO: explicitly point to the superseded goal, not just the next one?
        stringBuilder.append("You are currently conversing with the user in order for them to submit a new instruction. This has temporarily suspended the \"current task\", which will continue upon completion of an instruction submission. That task is: ").append(pendingGoalPredicates.get(0)).append("\n");
      } else {
        stringBuilder.append("Your current task is: ").append(currentGoalPredicate.get(0).toString()).append("\n");
      }
    } else {
      stringBuilder.append("<You have no current task>\n");
    }

    stringBuilder.append("Your pending tasks are:\n");
    if (offContext && !pendingGoalPredicates.isEmpty()) {
      pendingGoalPredicates.remove(0);
    }
    if (pendingGoalPredicates.isEmpty()) {
      stringBuilder.append("<You have no pending tasks>\n");
    } else {
      for (Predicate pgPred : pendingGoalPredicates) {
        stringBuilder.append(pgPred).append("\n");
      }
    }

    if (scheduledGoals.isEmpty()) {
      stringBuilder.append("Your scheduled tasks are:\n");
      stringBuilder.append("<You have no scheduled tasks>\n");
    } else {
      stringBuilder.append("Scheduled goals may contain the following information in order: The underlying task to be submitted, the start time to begin submitting the underlying task, the interval at which to keep resubmitting the task, and the end time to stop resubmitting the task. A missing period and end time indicates that the task will only be submitted once. Your scheduled tasks are:\n");
      for (Predicate sg : scheduledGoals) {
        stringBuilder.append(sg).append("\n");
      }
    }
    return stringBuilder.toString();
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
    //if (output.isEmpty()) {
    //  output = "<There are no current outstanding questions>";
    //}
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
    log.warn("[SocialAvaLLMServiceProvider] TODO: IMPLEMENT LEARNED ACTION -> TOOL TARGET TRANSLATION");
  }

  @Override
  public void handleDatabaseAddition(ActionDBEntry entry) throws TRADEException {
    super.handleDatabaseAddition(entry);
    log.warn("[SocialAvaLLMServiceProvider.handleDatabaseAddition] TODO: IMPLEMENT LEARNED ACTION -> TOOL TARGET TRANSLATION");
  }

  @Override
  public void handleDatabaseRemoval(ActionDBEntry entry) throws TRADEException {
    super.handleDatabaseRemoval(entry);
    log.warn("[SocialAvaLLMServiceProvider.handleDatabaseRemoval] TODO: Remove corresponding tool target when learned action is forgotten");
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
         if (utterance.getSemantics() == null || !utterance.getSemantics().isPredicate()) {
           return;
         }
        //NLU
        Symbol agentTeam = utterance.getAddressee().toUntyped();
        Symbol user = utterance.getSpeaker().toUntyped();
        ChatCollection chatCollection = getChatCollection(user, agentTeam);
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

          Predicate semantics = (Predicate) utterance.getSemantics();
          // If negative validation response, don't unlock until reattemptParse completes
          if ((semantics.getName().equals("val") || semantics.getName().equals("validated")) && semantics.get(0).getName().equals("no")) {
            return;
          }

          // TODO: How to detect all ITK cases to know whether to keep result in the history? Currently not using the
          //  itk pipeline due to issues interacting with EM queueing. Can at least match against any LLM itk tool..
          // Perform necessary side effects to current chat. 2 cases handled collectively here:
          //   External parser hit for an action instruction - discard chat context holding any side effects from unused LLM parse
          //   LLM noConfirmation case - handle side effects as if a validated instruction just occurred
          if (utterance.getType() == UtteranceType.INSTRUCT && !utterance.needsValidation()) {
            boolean preserveHistory = false;

            // (noConfirmation LLM cases only)
            if (utterance.getSemantics() != null && utterance.getSemantics().isPredicate()) {
              if (semantics.getArgs().size() > 1) {
                String command = semantics.get(1).getName();
                if (command.equals("generateResponseFromString") || itkActionNames.contains(command)) {
                  // TODO: should this check be for all SKIPPENDING?
                  // Remove history unless LLM text response or ITK call
                  preserveHistory = true;
                } else if (chatContext.chatType == ChatType.FOCUSED && command.equals("suspendSystemGoalsBlocking")) {
                  // Temp workaround until reintroducing offContextSubmission - don't terminate context here if suspending a focused context
                  preserveHistory = true;
                } else {
                  if (command.equals("offContextSubmission")) {
                    // TODO: can this be moved somewhere else
                    // Moved to suspended context on suspend
                    preserveHistory = true;
                    chatContext.utteranceUpForValidation = utterance;
                  }
                  // For LLM targets under noConfirmation - perform special side effects for "focused" contexts
                  handleValidatedToolCall(command, chatCollection);
                }
              }
            }

            // Clear history unless noConfirmation LLM target dictates it should be retained
            if (!preserveHistory) {
              endChatContext(user, agentTeam);
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
            chatContext.history.addRobotMessage(utterance.getWordsAsString());
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
            chatCollection.currentChat = null;
          }
        }
      }
    }
  }

  //TODO: should postprocessing errors here increment "internalErrorCount" even if returning their own specialized error message?
  /**
   * Process an LLM Tool call response by matching it against existing Tool Definitions.
   * Return:
   *   Fully populated utterance with information extracted from functionCall
   *   Null on any error case - errorMessage should be updated in this case
   */
  private Utterance processToolCall(ToolFunction functionCall, Tool tool, Utterance inputUtterance, StringBuilder errorMessage) {
    Map<String, String> calledArguments = new Gson().fromJson(functionCall.arguments, new TypeToken<LinkedTreeMap<String, String>>() {
    }.getType());
    calledArguments.values().removeIf(Objects::isNull);
    boolean validParse = true;

    ToolDefinition toolDefinition = tool.toolDefinition;

    // Check all required arguments exist
    ToolParameters targetParameters = toolDefinition.function.parameters;
    for (String paramName : targetParameters.getProperties().keySet()) {
      if (targetParameters.getRequired().contains(paramName)) {
        if (!calledArguments.containsKey(paramName)) {
          validParse = false;
          errorMessage.append("- The user's instruction was missing a value for the required \"").append(paramName).append("\" argument\n");
        }
      }
    }

    // Add actor to output semantics
    Map<Symbol, Variable> varMap = new LinkedHashMap<>();
    List<Symbol> processedArguments = new ArrayList<>();
    //TODO: QUESTION should not be in this case once ITK pipeline is fixed - and/or make distinction between going through
    //      the itk pipeline to have it generate its own response vs calling the underlying action to populate a tool response
    if (tool.utteranceType == UtteranceType.INSTRUCT || tool.utteranceType == UtteranceType.QUESTION) {
      String actorName = inputUtterance.getAddressee().getName();
      processedArguments.add(Factory.createSymbol(actorName));
    }
    Utterance.Builder output = new Utterance.Builder(inputUtterance);

    // Validate Tool call's supplied arguments
    for (String argName : calledArguments.keySet()) {
      Symbol processedArg;
      ToolParameter targetParam = targetParameters.getProperties().get(argName);
      if (targetParam == null) {
        // An argument was supplied which isn't present in the target set
        //  (This is more just an LLM issue? Do we do anything in this case, or just discard the information?)
        log.warn("[processToolCall] argument supplied by LLM in tool call which doesn't exist in tool definition. Discarding.");
      } else {
        Symbol argValue = Factory.createFOL(calledArguments.get(argName));
        // If there is an explicit possible_values set, make sure the argument is present
        if (!targetParam.getPossible_values().isEmpty()) {
          if (!targetParam.getPossible_values().contains(calledArguments.get(argName))) {
            validParse = false;
            //TODO: include type information and use in error message
            errorMessage.append("- The user's instruction contained an invalid value for the \"").append(argName).append("\" argument\n");
            continue;
          }
          // Replace reference descriptor with variable if type contained in refTypes
          //TODO: can probably refactor to eliminate duplicate code
          if (refTypes.contains(argValue.getType())) {
            if (!varMap.containsKey(argValue)) {
              Variable var = Factory.createVariable("VAR" + varMap.size() + ":" + argValue.getType());
              varMap.put(argValue, var);

              output.addSupplementalSemantics(Factory.createPredicate(argValue.toUntypedString(), var));
              Symbol tier = Factory.createSymbol("DEFINITE");
              output.addTierAssignment(var, tier);

              processedArg = var;
            } else {
              processedArg = varMap.get(argValue);
            }
          } else {
            processedArg = argValue;
          }
        } else {
          // Unrestricted argument (at least explicitly), make sure it is otherwise valid (populated, typed correctly, etc.)
          boolean unpopulatedArgSafe = toolDefinition.function.name.equals("reportIfCanPerform");
          processedArg = processToolCallArgument(argName, argValue, errorMessage, varMap, output, false,unpopulatedArgSafe);
          if (processedArg == null) {
            validParse = false;
            continue;
          }
        }
        processedArguments.add(processedArg);
      }
    }

    // Return appropriate object
    if (validParse) {
      Predicate semantics = Factory.createPredicate(toolDefinition.function.name, processedArguments);
      output.setSemantics(semantics);
      output.setUtteranceType(tool.utteranceType);
      return output.build();
    } else {
      return null;
    }
  }
  private Symbol processToolCallArgument(String argName, Symbol arg, StringBuilder errorMessage, Map<Symbol,Variable> varMap, Utterance.Builder output, boolean inner, boolean unpopulatedArgSafe) {
    if (arg.isPredicate()) {
      List<Symbol> processedInnerArguments = new ArrayList<>();
      if (arg.getName().equals("reportIfCanPerform")) {
        unpopulatedArgSafe = true;
      }
      for (Symbol innerArg : ((Predicate)arg).getArgs()) {
        Symbol processedInnerArg = processToolCallArgument(argName, innerArg, errorMessage, varMap, output, true, unpopulatedArgSafe);
        processedInnerArguments.add(processedInnerArg);
      }
      if (processedInnerArguments.contains(null)) {
        return null;
      }
      return Factory.createPredicate(arg.getName(), processedInnerArguments);
    } else {
      String argString = arg.toString();

      // Reject any unpopulated arguments returned by the LLM, unless the semantics specifically allow for that
      if (unpopulatedArgSafe && argString.startsWith("?")) {
        return Factory.createSymbol(argString.substring(1).split(":")[0]);
      } else if (argString.startsWith("?") || (argString.length() == 1 && Character.isUpperCase(argString.charAt(0)))) {
        log.warn("[parseNLUCompletionResponse] NLU parse has unpopulated argument present, failing: {}", argString);
        if (inner) {
          //TODO: Is this case generally another tool call in some form? Could explicitly try to find corresponding ToolDefinition here as well?
          errorMessage.append("- The user's supplied value for the \"").append(argName).append("\" argument was missing information about the ").append(argString).append(" detail\n");
        } else {
          errorMessage.append("- The user's instruction was missing a value for the required \"").append(argName).append("\" argument\n");
        }
        return null;
      }

      Symbol argSymbol = Factory.createFOL(argString);
      // Replace reference symbol with Variable and add supplemental semantics
      if (argSymbol != null && (refTypes.contains(argSymbol.getType()))) {
        if (!varMap.containsKey(argSymbol)) {
          Variable var = Factory.createVariable("VAR" + varMap.size() + ":" + argSymbol.getType());
          varMap.put(argSymbol, var);

          output.addSupplementalSemantics(Factory.createPredicate(argSymbol.toUntypedString(), var));
          Symbol tier = Factory.createSymbol("DEFINITE");
          output.addTierAssignment(var, tier);

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

  Predicate replaceReferencesInSemantics(Predicate semantics) {
    List<Symbol> transformedArgs = new ArrayList<>();
    for (Symbol arg: semantics.getArgs()) {
      if (arg.isPredicate()) {
        transformedArgs.add(replaceReferencesInSemantics((Predicate) arg));
      } else {
        if (refTypes.contains(arg.getType())) {
          Map<Symbol, List<Term>> refs = new HashMap<>();
          try {
            refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateRE").argTypes(Symbol.class)).call(Map.class, arg);
            if (refs.get(arg).size() == 1) {
              transformedArgs.add(Factory.createSymbol(refs.get(arg).get(0).getName(),refs.get(arg).get(0).get(0).getType()));
            }
          } catch (TRADEException e) {
            log.warn("[generateREG]", e);
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
      int internalErrorCount = 1;
      ChatContext chatContext = null;
      ChatCollection chatCollection = getChatCollection(user.toUntyped(), agentTeam.toUntyped());
      if (chatCollection != null) {
        chatContext = chatCollection.currentChat;
        if (chatContext != null) {
          chatContext.incrementNumParseErrors();
          internalErrorCount = chatContext.getNumParseErrors();
        }
      }

      if (internalErrorCount >= 3) {
        endChatContext(user, agentTeam);
        chatContext.resetNumParseErrors();
      }
      errorMessage = generateErrorResponse(internal, internalErrorCount, false);

      //if (internalErrorCount == 1) {
      //  //utterance.setWords(generateErrorResponse(internal,internalErrorCount,false));
      //} else if (internalErrorCount == 2) {
      //  //utterance.setWords(generateErrorResponse(internal,internalErrorCount,false));
      //} else {
      //  endChatContext(user, agentTeam);
      //  chatContext.resetNumParseErrors();
      //  //utterance.setWords(generateErrorResponse(internal,internalErrorCount,false));
      //}
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

//        if (!restrictedMode) {
//          try {
//            TRADE.getAvailableService(new TRADEServiceConstraints().name("setHybridParserRestrictedMode").argTypes(Boolean.class)).call(void.class, true);
//            restrictedMode = true;
//            if (parse) {
//              utterance.setSemantics(Factory.createPredicate("error", agentTeam.toUntypedString(), "and(sorryIAmUnableToConnectToNecessaryServices,enteringRestrictedInteractionMode,pleaseTryStartingOver)"));
//            }
//            utterance.setWords("Sorry, I am unable to connect to necessary services. Entering restricted interaction mode. Please try starting over.");
//          } catch (TRADEException e) {
//            log.info("[generateErrorResponse] could not reach Hybrid parser to set restricted mode. Assuming we are only using the LLM parser", e);
//            if (parse) {
//              utterance.setSemantics(Factory.createPredicate("error", agentTeam.toUntypedString(), "and(sorryIAmUnableToConnectToNecessaryServices,pleaseReconnectOrReachOutToSupport)"));
//            }
//            utterance.setWords("Sorry, I am unable to connect to necessary services. Please check your internet connection or reach out to support.");
//          }
//        }
      } else {
        errorMessage = generateErrorResponse(internal, externalErrorCount, false);

        //if (parse) {
        //  utterance.setSemantics(Factory.createPredicate("error",agentTeam.toUntypedString(),generateErrorResponse(internal, externalErrorCount, true)));
        //}
        //utterance.setWords(generateErrorResponse(internal, externalErrorCount, false));
      }
    }
    if (parse) {
      utterance.setSemantics(Factory.createPredicate("generateResponseFromString", agentTeam.toUntypedString(), user.toUntypedString(),"\""+errorMessage+"\""));
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

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// Should be domain specific

  //TODO: explicitly handle error cases if finding they are still possible
  private void correctCommonNLUMistakes(Utterance utterance) {
    Symbol actor = utterance.getAddressee();
    try {
      String command = utterance.getSemantics().getName();
      if (command.equals("taskPredicate")) {
        //Not currently prompted to do this
        if (utterance.getSemantics().hasArgs()) {
          Symbol innerArg = ((Term)utterance.getSemantics()).get(0);
          List<Symbol> argsWithActor = new ArrayList<>();
          argsWithActor.add(actor);
          if (innerArg.isTerm()) {
            argsWithActor.addAll(((Term) ((Term) utterance.getSemantics()).get(0)).getArgs());
          }
          utterance.setSemantics(Factory.createPredicate(command, Factory.createPredicate(innerArg.getName(), argsWithActor)));
        }
      } else if (command.equals("forgetTaughtAction") || command.equals("reportIfCanPerform") || command.equals("describeActionSteps") || command.equals("urgentPriority")) {
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
      } else if (command.equals("playVideo")) {
        // Special format returned by LLM for different parameterization of playing videos- make sure output is
        //    valid and coerce into proper specific action semantic form.
        Predicate semantics = (Predicate) utterance.getSemantics();

        List<Symbol> args = semantics.getArgs();
        if (args.size() == 1) {
          utterance.setSemantics(Factory.createPredicate("playNextVideo", actor));
        } else if (args.size() == 2) {
          if (args.get(1).getType().equals("video")) {
            utterance.setSemantics(Factory.createPredicate("playSpecificVideo", actor, args.get(1).toUntyped()));
          } else if (args.get(1).getType().equals("location")) {
            utterance.setSemantics(Factory.createPredicate("playNextVideoAtLocation", actor, args.get(1)));
          } else {
            //(This shouldn't be possible anymore - if it is, it means we don't have types in possible_values Tool lists)
            //TODO: could try to infer type based off of literal. Do this if the LLM is found to make this mistake
            log.error("[correctCommonNLUMistakes] single argument supplied without a type - not sure which optional argument was supplied");
          }
        } else if (args.size() == 3) {
          utterance.setSemantics(Factory.createPredicate("playSpecificVideoAtLocation", actor, args.get(1).toUntyped(), args.get(2)));
        } //else (shouldn't be possible anymore)
      } else if (command.equals("displayImage")) {
        // Special format returned by LLM for different parameterization of displaying images - make sure output is
        //    valid and coerce into proper specific action semantic form.
        Predicate semantics = (Predicate) utterance.getSemantics();

        // Needs to have all arguments defined (actor, video name, location, duration)
        List<Symbol> args = semantics.getArgs();
        if (args.size() == 2) {
          utterance.setSemantics(Factory.createPredicate("displayNamedImage",actor, args.get(1).toUntyped()));
        } else if (args.size() == 3) {
          //TODO: Be more careful with type/value checking if LLM is found to make mistakes here
          if (args.get(2).getType().equals("location")) {
            utterance.setSemantics(Factory.createPredicate("displayNamedImageAtLocation",actor,args.get(1).toUntyped(),args.get(2)));
          } else {
            utterance.setSemantics(Factory.createPredicate("displayNamedImageForDuration",actor,args.get(1).toUntyped(),args.get(3)));
          }
        } else if (args.size() == 4) {
          utterance.setSemantics(Factory.createPredicate("displayNamedImageForDurationAtLocation",actor,args.get(1).toUntyped(),args.get(2), args.get(3)));
        } //else (shouldn't be possible anymore)
      } else if (command.equals("scheduled")) {
        List<Symbol> args = new ArrayList<>();
        for (Symbol arg : ((Predicate)utterance.getSemantics()).getArgs()) {
          //Need to remove actor from scheduled predicate - will be filtered out by default
          if (arg.isPredicate()) {
            Predicate argPred = (Predicate) arg;
            if (arg.getName().equals("start")) {
              if (argPred.hasArgs() && !argPred.get(0).getName().equals("immediately")) {
                args.add(arg);
              }
            } else if (arg.getName().equals("period")) {
              if (argPred.hasArgs() && !argPred.get(0).getName().equals("nonrepeating")) {
                args.add(arg);
              }
            } else if (arg.getName().equals("end")) {
              if (argPred.hasArgs() && !argPred.get(0).getName().equals("indefinitely")) {
                args.add(arg);
              }
            } else {
              //Need to add actor into task argument
              List<Symbol> innerArgs = ((Predicate)arg).getArgs();
              innerArgs.add(0,actor);
              args.add(Factory.createPredicate(arg.getName(),innerArgs));
            }
          }
        }
        utterance.setSemantics(Factory.createPredicate(command,args));
      }
    }
    catch (Exception e) {
      log.warn("[correctCommonNLUMistakes] unexpected semantics format {}", utterance.getSemantics());
      //TODO: explicitly fail with a specific message here? Or just return unmodified utterance?
    }
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
      return names.stream().map(vid->vid+":video").toList();
    } catch (TRADEException e) {
      log.error("[getVideoNames]", e);
    }
    return new ArrayList<>();
  }

  private List<String> getImageNames() {
    try {
      List<String> names = TRADE.getAvailableService(new TRADEServiceConstraints().name("getImageNames").argTypes(Boolean.class)).call(List.class,true);
      return names.stream().map(img->img+":image").toList();
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
      if (tsi != null) {
        refs = tsi.call(Map.class);
      }
      if (refs != null) {
        return refs.values().stream().map(ref -> {
          if (!ref.properties.isEmpty()) {
            return ref.properties.get(0).getName() + ":location";
          } else {
            return "";
          }
        }).toList();
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
        case "getCurrentPendingScheduledGoalInformation":
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
      argumentsString = toolFunction.parameters.getProperties().isEmpty() ? "None" : "\n" +
              toolFunction.parameters.getProperties().entrySet().stream().map(entry ->
                      entry.getKey() + " (" + (required.contains(entry.getKey()) ? "necessary" : "optional") + "): "
                              + entry.getValue().getDescription()
                              +  " (target set: " + entry.getValue().getPossible_values() + ")"
                      ).collect(Collectors.joining("\n"));
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
            "You go to the supplied destination location. An example of instructing this could be \"please navigate to %exampleLocation\", which would map to goToLocation(%exampleLocation).",
            List.of(new ToolParameterHelper("destination", "string", locationNames, "The destination location to go to", true))
    );
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("dock",
            "You \"dock\" at the supplied dock destination (meaning you idly charge on your charging port). An example of instructing this could be \"dock at %exampleLocation\", which would map to dock(%exampleLocation).",
            List.of(new ToolParameterHelper("destination", "string", locationNames, "The name of the dock tag", true))
    );
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("follow",
            "You follow the user. An example of instructing this could be \"follow me.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("say",
            "You say ?words. An example of instructing this could be \"say hello, it's me.\", which would map to say(hello, it's me)",
            List.of(new ToolParameterHelper("words", "string", new ArrayList(), "The phrase to be said", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("stopFollow",
            "You stop following the user. An example of instructing this could be \"stop following me.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("escortToLocation",
            "You lead someone from your current location to a different one. An example of instructing this could be \"bring Owen to %exampleLocation.\", which would map to escortToLocation(owen,%exampleLocation)",
            List.of(new ToolParameterHelper("name", "string", new ArrayList(), "Name of the entity to escort", true),
                    new ToolParameterHelper("destination", "string", locationNames, "The destination location to escort to", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("escortFromLocationToLocation",
            "You meet someone at the specific start location and then lead them to the destination location. An example of instructing this could be \"bring our guest from %exampleLocation to the dock.\", which would map to escortFromLocationToLocation(guest,%exampleLocation,dock:location)",
            List.of(new ToolParameterHelper("name", "string", new ArrayList(), "Name of the entity to escort", true),
                    new ToolParameterHelper("start", "location", new ArrayList(), "The start location to escort from. This should be where ?name can be found", true),
                    new ToolParameterHelper("destination", "location", new ArrayList(), "The destination location to escort to", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("saveLocation",
            "You save your current location under the supplied name. An example of instructing this could be \"save this area as location one.\", which would map to saveLocation(location one)",
            List.of(new ToolParameterHelper("name", "string", new ArrayList(), "the label for the location to save", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //definition = new ToolDefinition("saveLocationsToFile()",
    //        "You backup all your currently saved locations",
    //        "backup your current locations",
    //        "saveLocationsToFile()", 10);
    //toolDefinitions.add(definition);
    definition = new ToolDefinition("sit",
            "You lower your screen. An example of instructing this could be \"sit down.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("stand",
            "You raise your screen. An example of instructing this could be \"stand up.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    //definition = new ToolDefinition("deliver(?objectName, ?location:location)",
    //        "You deliver ?objectName to location ?location:location",
    //        "bring the mug to %exampleLocation",
    //        "deliver(mug,%exampleLocation)", 10);
    //toolDefinitions.add(definition);
    definition = new ToolDefinition("goHome",
            "You go to your default charging dock and charge. An example of instructing this could be \"go back to charge.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("scheduleTask",
            "You begin a dialogue interaction with the user to schedule future or regularly repeating execution of a task. An example of instructing this could be \"I want to schedule a goal.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("modifyScheduledGoal",
            "You begin a dialogue interaction with the user to modify the timing of a future already scheduled task. An example of instructing this could be \"I want to change the timing of an action.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("cancelSystemGoals",
            "You stop and fully discard your current task. An example of instructing this could be \"stop your current task.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("suspendSystemGoalsBlocking",
            "You pause your current task, which can be later resumed. An example of instructing this could be \"pause what you're doing.\"",
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
    definition = new ToolDefinition("cancelScheduledGoal",
            "You begin a dialogue interaction with the user to cancel a particular scheduled goal. An example of instructing this could be \"I want to cancel a scheduled task.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("learnNewTask",
            "You begin a dialogue interaction with the user to learn the steps of a new instructable action. An example of instructing this could be \"i will teach you a new task.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("endActionLearning",
            "You conclude learning the steps of a new instructable action. Examples of instructing this could be \"those are the steps of the task\" or \"im done\".",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("forgetTaughtAction",
            "You forget a previously taught action represented by the supplied actionPredicate. actionPredicate should be assigned a populated instance of another tool call. An example of instructing this could be \"forget how to go to %exampleLocation.\", which would map to forgetTaughtAction(goToLocation(%exampleLocation))",
            List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "The populated tool call representing the action to forget", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("learnTourAction",
            "You begin a dialogue interaction with the user where they teach you how to perform a new type of tour (navigating between a sequence of locations while performing specific actions at each one). An example of instructing this could be \"I want to teach you how to perform a tour.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("updateKioskTimeout",
            "You begin a dialogue interaction with the user to update the configured kiosk timeout. An example of instructing this could be \"I would like to update the kiosk timeout.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("setKioskTimeout",
            "You update the configured kiosk timeout. An example of instructing this could be \"Set the kiosk timeout to 30 minutes.\", which would map to setKioskTimeout(30,minutes)",
            List.of(new ToolParameterHelper("duration", "integer", new ArrayList(), "The number of timeunits defining the timeout", true),
                    new ToolParameterHelper("timeunit", "string", Arrays.stream(TimeUnit.values()).map(Objects::toString).toList(), "The timeunit of the duration to set ", true)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("setKioskModeEnabled",
            "You set whether or not kiosk mode is enabled. An example of instructing this could be \"turn on kiosk mode.\", which would map to setKioskModeEnabled(true)",
            List.of(new ToolParameterHelper("enabled", "boolean", new ArrayList<>(), "the truth value of kiosk mode being enabled", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("updateKioskAction",
            "You begin a dialogue interaction with the user to update the configured kiosk action. An example of instructing this could be \"I want to change the kiosk action.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("setVideoVolume",
            "You set the volume of videos played on your screen. An example of instructing this could be \"Set the video volume to max.\", which would map to setVideoVolume(100)",
            List.of(new ToolParameterHelper("volume", "integer", new ArrayList(), "The volume to set the video media player to. Acceptable values are whole numbers between 0 and 100", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("waitForDuration",
            "You wait idly for a number of seconds. An example of instructing this could be \"Then wait for a minute.\", which would map to waitForDuration(60)",
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
    definition = new ToolDefinition("playVideo",
            "You begin playback of a video from your saved collection at some location.",
            List.of(new ToolParameterHelper("videoName", "string", getVideoNames(), "The video to play. Not supplying this argument means that the next video in your saved collection will be played", false),
                    new ToolParameterHelper("location", "string", locationNames, "Where to play the video. Not supplying this argument means that you will play the video at your current location.", false)
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
    definition = new ToolDefinition("displayImage",
            "You display an image from your saved collection at some location for a certain duration.",
            List.of(new ToolParameterHelper("imageName", "string", getImageNames(), "The image to display", true),
                    new ToolParameterHelper("location", "string", locationNames, "Where to display the image. Not supplying this argument means the image will be displayed at your current location", false),
                    new ToolParameterHelper("duration", "integer", new ArrayList<>(), "How long to display the image for. Not supplying this argument means you will display the image indefinitely", false)
            ));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("urgentPriority",
            "You submit an action with urgent priority. This means that this action will interrupt execution of any other currently submitted actions and be immediately prioritized. An example of instructing this could be \"go to %exampleLocation right now.\", which would map to urgentPriority(goToLocation(%exampleLocation))",
            List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "The action to submit with urgent priority. This should be filled with a fully populated instance of another tool call.", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("cancelTaskInQueueByIndex",
            "You cancel and remove a task in the task queue specified by its index. An example of instructing this could be \"Remove the first task from the queue.\", which would map to cancelTaskInQueueByIndex(0)",
            List.of(new ToolParameterHelper("index", "integer", new ArrayList(), "The 0-based index of the task in the queue to remove", true)));
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("enableAdminMode",
            "Allow certain actions which require administrator privileges to be instructed through voice directly to the robot.. An example of instructing this could be \"enable admin mode.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("disableAdminMode",
            "Reject certain actions which require administrator privileges from being instructed through voice directly to the robot.. An example of instructing this could be \"disable admin mode.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("switchDisplayToTelepresence",
            "Change the output on your display to the telepresence stream. An example of instructing this could be \"start telepresence.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));
    definition = new ToolDefinition("switchDisplayToTRACS",
            "Change the output on your display to what is shown by TRACS, the default setting. An example of instructing this could be \"return to TRACS.\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));

    //TODO: need to update this with new social extended interaction structure - Do we still even need this as an action? Or should be rephrased to repeatSystemQuestion
    definition = new ToolDefinition("repeatLastQuestion",
            "You repeat the last question you asked, which the human still needs to answer. examples of instructing this could be \"could you repeat that\" or \"what is the question\"",
            new ArrayList());
    instructTools.add(new Tool(definition, UtteranceType.INSTRUCT));


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
            "You report the current battery level of the robot as a percentage. An example of instructing this could be \"how much charge do you have.\"",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    definition = new ToolDefinition("getCurrentLocationString",
            "You report what location you believe you are currently at. An example of instructing this could be \"what is your current location.\"",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    definition = new ToolDefinition("getAllKnownLocations",
            "You list all currently saved locations. An example of instructing this could be \"what are your saved locations.\"",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    definition = new ToolDefinition("getTaughtActions",
            "You list all of the actions which have been taught to you by a user. An example of instructing this could be \"What actions have you been taught.\"",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    definition = new ToolDefinition("getIfCanPerform",
            "You report whether you know how to perform the action represented by the supplied actionPredicate. actionPredicate should be assigned a populated instance of another tool call. An example of instructing this could be \"do you know how to escort a patient from %exampleLocation to the dock.\", which would map to reportIfCanPerform(escortFromLocationToLocation(guest,%exampleLocation,dock:location))",
            List.of(new ToolParameterHelper("actionPredicate", "string", new ArrayList(), "The populated tool call representing the action queried about", true)));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
//    definition = new ToolDefinition("reportInstructableActions",
//            "You report the list of actions a user is able to instruct you through natural language to perform. An example of instructing this could be \"what can you be instructed to do.\"",
//            new ArrayList());
//    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    /// /////////////////////STANDIN ///////////////////////////////////////////
    /// /////////////////////STANDIN ///////////////////////////////////////////
    /// /////////////////////STANDIN ///////////////////////////////////////////
    definition = new ToolDefinition("getPredefinedActions",
            "You report the list of actions a user is able to instruct you through natural language to perform. An example of instructing this could be \"what can you be instructed to do.\"",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    /// /////////////////////STANDIN ///////////////////////////////////////////
    /// /////////////////////STANDIN ///////////////////////////////////////////
    /// /////////////////////STANDIN ///////////////////////////////////////////
    definition = new ToolDefinition("getActionSteps",
            "You describe an action by listing out the steps which would be performed when it is executed. ?action can be replaced by a populated instance of another tool OR be just the name of another tool if no specific arguments are supplied. An example of instructing this could be \"How would you escort someone.\", which would map to describeActionSteps(escort)",
            List.of(new ToolParameterHelper("action", "string", new ArrayList(), "The tool whose execution steps to describe", true)));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    definition = new ToolDefinition("getKioskSettings",
            "You report the current configuration of kiosk mode; Whether it is enabled and the currently configured kiosk timeout and action. An example of instructing this could be \"what is your kiosk configuration.\"",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    definition = new ToolDefinition("getVideoVolume",
            "You report the current volume videos are set to play at. An example of instructing this could be \"what is the video volume set to.\"",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    definition = new ToolDefinition("getIsVideoSaved",
            "You answer whether you have a video saved under the supplied name. An example of instructing this could be \"Do you have the %exampleVideo video saved.\", which would map to reportIsVideoSaved(?videoName)",
            List.of(new ToolParameterHelper("videoName", "string", new ArrayList<>(), "The video being queried about", true)));
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    definition = new ToolDefinition("getSavedVideos",
            "You list out the collection of videos you currently have saved. An example of instructing this could be \"what videos can you play.\"",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));
    definition = new ToolDefinition("getSavedImages",
            "You list out the collection of images you currently have saved. An example of instructing this could be \"what images can you display.\"",
            new ArrayList());
    itkTools.add(new Tool(definition, UtteranceType.QUESTION));

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
            List.of(new ToolParameterHelper("response","string",List.of("yes","no"),"The response value should be yes when the user has validated your interpretation of the their instruction, the response value should be no when the user does not validate the parsed meaning of their instruction",true)));
    replyTools.put("validated",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("val(X)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition("location",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION by specifying a certain saved location",
            List.of(new ToolParameterHelper("name","string",locationNames,"The specified location. \"none\" is the one special value, indicating the user is done supplying location names.",true)));
    replyTools.put("location",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("location(X:location)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition("taskPredicate",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of another tool call. An example of instructing this could be \"the task is to go to %exampleLocation\", which would map to taskPredicate(goToLocation(%exampleLocation))",
            List.of(new ToolParameterHelper("toolCall","string",new ArrayList<>(),"This argument should be replaced by definition of another tool call and populated arguments from the previous examples",true)));
    replyTools.put("taskPredicate",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("taskPredicate(X)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition("relativeTime",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of a time relative to now. An example of instructing this could be \"in two hours\", which would map to relativeTime(2,HOURS)",
            List.of(new ToolParameterHelper("duration","number",List.of(),"an integer, the duration of the specified time",true),
                    new ToolParameterHelper("timeUnit","string", Arrays.stream(TimeUnit.values()).map(TimeUnit::toString).toList(),"the duration's corresponding timeunit",true)
            ));
    replyTools.put("relativeTime",new Tool(definition,UtteranceType.REPLY));
    //replyTools.put(Factory.createPredicate("relativeTime(X,Y)"), new Tool(definition,UtteranceType.REPLY));
    definition = new ToolDefinition("absoluteTime",
            "You inform the system that the user has answered its current OUTSTANDING QUESTION through specification of an absolute time. An example of instructing this could be \"half past noon\", which would map to absoluteTime(12:30,pm)",
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
