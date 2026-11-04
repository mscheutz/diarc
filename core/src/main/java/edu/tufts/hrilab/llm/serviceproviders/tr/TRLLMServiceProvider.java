package edu.tufts.hrilab.llm.serviceproviders.tr;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.GoalFilter;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.serviceproviders.BasicLLMServiceProvider;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

//TODO: remaining information which can potentially be pulled up from subclasses (and perhaps into base service provider):
//        Semantic target entries: Class to instantiate/format entries + prompt structure/presentation
//        Consultant scraping of registered types for ref lists / presentation in prompt
//        Current/Pending/Scheduled goals
//        Current Question list
//      ^To make pulling all of this logic into here actually useful, all domain specific prompts would have to be
//        updated to conform to a specific structure using all of this information. Do not presently have time for this.
public abstract class TRLLMServiceProvider extends BasicLLMServiceProvider {
  protected String language = "English";
  protected boolean processMatchingNLUParseLiteralsAsRefs = false;

  protected String learnedActionsString = "";
  protected List<String> learnedActionNames = new ArrayList<>();

  /**
   * One-shot completion requests consist of a specific system message containing instructions and a user message
   *  containing the contents of a specific request. Current distinctions are all NLU cases, general NLG, and specific
   *  validation NLG
   */
  Prompt nluSystemPrompt;
  Prompt nluUserPrompt;
  Prompt nlgSystemPrompt;
  Prompt nlgUserPrompt;
  Prompt validationSystemPrompt;
  Prompt validationUserPrompt;
  protected abstract Prompt getNluSystemPrompt();
  protected abstract Prompt getNluUserPrompt();
  protected abstract Prompt getNlgSystemPrompt();
  protected abstract Prompt getNlgUserPrompt();
  protected abstract Prompt getValidationSystemPrompt();
  protected abstract Prompt getValidationUserPrompt();

  /**
   * Return prompt variables relevant for the associated Prompt to be included in an upcoming request. Refer to the
   *  Prompt class and apply method if this notion is unfamiliar to you. These details may be generated on each request,
   *  or constantly updated in the background and provided to requests from this method.
   *  Please explicitly incorporate, overwrite, or discard information supplied in the passed in promptVars.
   * @param utterance - The input/output utterance associated with this request
   * @param promptVars - Default prompt variables containing common system information gathered in LLMServiceProvider
   * @param system - Whether these variables are being applied to the system message or user message
   * @param parsing - Whether this call is part of an NLU or NLG request
   * @param validation - (NLG Only) whether this call is associated with a system validation question
   * @return The variables to apply to the associated prompt in an upcoming request
   */
  protected abstract Map<String,String> populatePromptVars(Utterance utterance, Map<String,String> promptVars, boolean system, boolean parsing, boolean validation);


  /**
   * List of types handled by consultants in the system - when encountering symbols with these types during
   *  NLG preprocessing/NLU postprocessing, replaces references with descriptors/typed descriptors with variables.
   */
  //TODO: pull from consultants automatically
  List<String> refTypes;
  protected abstract List<String> getRefTypes();

  /**
   * Domain specific postprocessing on NLU completions. Simply return supplied utterance if you have no domain specific
   *  postprocessing.
   */
  protected abstract Utterance correctCommonNLUMistakes(Utterance utterance);

  //TODO: Should this be updated? Currently even any nonrelated argument whose value happens to match a known entities
  // chosen descriptor will be processed as a reference to that entity.
  /**
   * Used during NLU postprocessing - Check if the supplied descriptor matches any known references in the system.
   * Return the associated reference's type (the descriptor will be typed replaced with a variable to later be dereferenced)
   * Return an empty string if no match is found or you do not desire this functionality.
   */
  //Check if descriptor matches the descriptor of an existing reference, if so return the reference type
  protected abstract String getDescriptorType(String agentName, String descriptor);

  @Override
  protected void init() {
    super.init();
    refTypes = getRefTypes();
    nluSystemPrompt = getNluSystemPrompt();
    nluUserPrompt = getNlgUserPrompt();
    nlgSystemPrompt = getNlgSystemPrompt();
    nlgUserPrompt = getNluUserPrompt();
    validationSystemPrompt = getValidationSystemPrompt();
    validationUserPrompt = getValidationUserPrompt();
    trackDialogueHistory = true;
    trackLearnedActions = true;
  }

  @Override
  protected Chat generateNLUChat(Utterance utterance) {
    // Create fresh chat with no explicit history
    Chat chat = new Chat();
    Map<String,String> promptVars = new LinkedHashMap<>();

    // Add conversation history string to prompt variables
    if (trackDialogueHistory) {
      Symbol agentTeam = getUntypedSymbol(utterance.getAddressee());
      List<Utterance> conversationHistory;
      if (!agentHierarchy.containsKey(agentTeam)) {
        log.error("[generateNLUChat] System agent found in incoming utterance not found in agent hierarchy");
        conversationHistory = new ArrayList<>();
      } else {
        conversationHistory = getConversationHistory(utterance.getSpeaker(), utterance.getAddressee());
      }
      String conversationHistoryString = getConversationHistoryString(conversationHistory);
      promptVars.put("%conversationHistory", conversationHistoryString);
    }

    // Populate and set system message
    promptVars = populatePromptVars(utterance, promptVars, true, true, false);
    chat.setSystem(nluSystemPrompt.apply(promptVars));
    promptVars.clear();

    // Populate and set user message
    promptVars = populatePromptVars(utterance, promptVars, false, true, false);
    chat.addUserMessage(nluUserPrompt.apply(promptVars));

    return chat;
  }

  @Override
  protected Chat generateNLGChat(Utterance utterance) {
    // Create fresh chat with no explicit history
    Chat chat = new Chat();
    Map<String,String> promptVars = new LinkedHashMap<>();

    // Add conversation history string to prompt variables
    if (trackDialogueHistory) {
      Symbol agentTeam = getUntypedSymbol(utterance.getSpeaker());
      List<Utterance> conversationHistory;
      if (!agentHierarchy.containsKey(agentTeam)) {
        log.error("[generateNLUChat] System agent found in incoming utterance not found in agent hierarchy");
        conversationHistory = new ArrayList<>();
      } else {
        conversationHistory = getConversationHistory(utterance.getAddressee(), utterance.getSpeaker());
      }
      String conversationHistoryString = getConversationHistoryString(conversationHistory);
      promptVars.put("%conversationHistory", conversationHistoryString);
    }

    // Intercept Validation Question and use specific validation prompt
    boolean validation = false;
    Predicate semantics;
    List<Term> boundSemantics = utterance.getBoundSemantics();
    if (boundSemantics.isEmpty()) {
      semantics = (Predicate) utterance.getSemantics();
    } else {
      semantics = (Predicate) boundSemantics.get(0);
    }
    if (semantics.getName().equals("mean")) {
      validation = true;
    }
    // Replace references with sane descriptors in semantics before handing off to LLM
    utterance = new Utterance.Builder(utterance).build();
    semantics = replaceReferencesInSemantics(semantics);
    utterance.setSemantics(semantics);


    // Populate and set system message
    promptVars = populatePromptVars(utterance, promptVars, true, true, validation);
    if (validation) {
      chat.setSystem(validationSystemPrompt.apply(promptVars));
    } else {
      chat.setSystem(nlgSystemPrompt.apply(promptVars));
    }
    promptVars.clear();
    // Populate and set user message
    promptVars = populatePromptVars(utterance, promptVars, false, true, validation);
    if (validation) {
      chat.addUserMessage(nlgUserPrompt.apply(promptVars));
    } else {
      chat.addUserMessage(validationUserPrompt.apply(promptVars));
    }

    return chat;
  }

  /**
   * Reach out to consultants to replace any references with their corresponding descriptor - only performed for types
   *  defined in refTypes class variable (populated by abstract method getRefTypes)
   */
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

  @Override
  public Utterance parseNLUCompletionResponse(Utterance incomingUtterance, Completion llmResponse) {
    try {
      // Default error response
      Utterance.Builder failure = new Utterance.Builder();
      failure.setSpeaker(incomingUtterance.getSpeaker());
      failure.setAddressee(incomingUtterance.getAddressee());
      failure.setSemantics(Factory.createPredicate("error("+incomingUtterance.getAddressee()+",and(report,not(understand(" + incomingUtterance.getAddressee() + ",that)),ask(pleaseTryAgain))"));
      failure.setWords(incomingUtterance.getWords());

      Utterance.Builder output = new Utterance.Builder(incomingUtterance);
      log.info("[parseNLUCompletionResponse] " + llmResponse.getText());
      String responseString = llmResponse.getText();

      // Extract semantics
      Pattern semanticsPattern = Pattern.compile(".*\"?Semantics\"?\\s*:\\s*\"?(.*)\"?,\\s*\"?Input Type\"?:.*", Pattern.DOTALL);
      Matcher m = semanticsPattern.matcher(responseString);
      if (!m.matches()) {
        log.error("[parseNLUCompletionResponse] cannot find Semantics or Input Type result in LLM response");
        return failure.build();
      }
      Predicate semantics = Factory.createPredicate(m.group(m.groupCount()));

      // Extract Utterance type
      Pattern typePattern = Pattern.compile(".*\"?Input Type\"?\\s*:\\s*\"?(.*?)\"?,\\s*\"?Confidence\"?:.*", Pattern.DOTALL);
      m = typePattern.matcher(responseString);
      if (!m.matches()) {
        log.error("[parseNLUCompletionResponse] cannot find Input Type result in LLM response");
        return failure.build();
      }
      UtteranceType utteranceType;
      try {
        utteranceType = UtteranceType.valueOf(m.group(m.groupCount()).trim());
      } catch (IllegalArgumentException e) {
        log.error("[parseNLUCompletionResponse] generated Input Type {} invalid", m.group(m.groupCount()));
        return failure.build();
      }

      // Extract confidence measure - fail if low confidence
      Pattern confidencePattern = Pattern.compile(".*\"?Confidence\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL);
      m = confidencePattern.matcher(responseString);
      if (!m.matches()) {
        log.error("[parseNLUCompletionResponse] cannot find Confidence result in LLM response");
        return failure.build();
      }
      String confidenceString = m.group(m.groupCount()).trim();
      int confidence = Integer.parseInt(confidenceString);
      if (confidence < 5) {
        log.info("[parseNLUCompletionResponse] llm indicated parse result as low confidence ({})", confidence);
        //TODO: Send architecture message to LLM explaining the issue
        return failure.build();
      }

      output.setUtteranceType(utteranceType);

      // Process parsed semantics - check for unpopulated arguments and replace reference descriptors with variables
      Map<Symbol, Variable> varMap = new LinkedHashMap<>();
      List<Symbol> processedArguments = new ArrayList<>();
      if (utteranceType == UtteranceType.INSTRUCT) {
        String actorName = incomingUtterance.getAddressee().getName();
        processedArguments.add(Factory.createSymbol(actorName));
      }
      processedArguments = processSemantics(semantics, processedArguments, varMap, output, incomingUtterance.getAddressee().getName());
      if (processedArguments == null) {
        return failure.build();
      }
      Predicate outputSemantics = Factory.createPredicate(semantics.getName(), processedArguments);
      output.setSemantics(outputSemantics);

      // Perform any domain specific postprocessing
      Utterance result = correctCommonNLUMistakes(output.build());
      if (result != null) {
        return result;
      } else {
        return failure.build();
      }
    } catch (Exception e) {
      log.warn("[parseNLUCompletionResponse]",e);
      return incomingUtterance;
    }
  }

  //TODO: need to update if intended to allow unpopulated variables in the form of ?arg to exist
  /**
   * Return processed arguments for supplied semantics returned by Completion request
   *  - fails if any arguments are unpopulated variables
   *  - replaces values matching descriptors for entities with types defined in refTypes with variables for RR
   */
  protected List<Symbol> processSemantics(Predicate semantics, List<Symbol> processedArguments, Map<Symbol,Variable> varMap, Utterance.Builder output, String actor) {
    return processSemantics(semantics,processedArguments,varMap, output, actor, false);
  }
  private List<Symbol> processSemantics(Predicate semantics, List<Symbol> processedArguments, Map<Symbol,Variable> varMap, Utterance.Builder output, String actor, boolean unpopulatedArgSafe) {
      if (semantics.getName().equals("reportIfCanPerform")) {
          unpopulatedArgSafe = true;
      }
    for (Symbol arg: semantics.getArgs()) {
      if (arg.isPredicate()) {
            List<Symbol> innerArgs = processSemantics((Predicate)arg,new ArrayList<>(),varMap,output,actor,unpopulatedArgSafe);
        if (innerArgs == null) {
          return null;
        }
        processedArguments.add(Factory.createPredicate(arg.getName(), innerArgs));
      } else {
        String argString = arg.toString();

        // Reject any unpopulated arguments returned by the LLM, unless the semantics specifically allow for that
        if (unpopulatedArgSafe && argString.startsWith("?")) {
            processedArguments.add(Factory.createSymbol(argString.substring(1).split(":")[0]));
            continue;
        } else if (argString.startsWith("?") || (argString.length() == 1 && Character.isUpperCase(argString.charAt(0)))) {
            log.warn("[parseNLUCompletionResponse] NLU parse has unpopulated argument present, failing: {}", argString);
            return null;
        }

        Symbol argSymbol;
        if (processMatchingNLUParseLiteralsAsRefs) {
          //Append types to arguments to actions which are references
          //TODO: this currently will process any literal matching any reference's descriptor as that reference.
          //      Either do this another safer way, or have the LLM return typed references
          String descriptorType = getDescriptorType(actor,argString);
          if (descriptorType.isEmpty()) {
            argSymbol = Factory.createFOL(argString);
          } else {
            argSymbol = Factory.createSymbol(argString, descriptorType);
          }
        } else {
          argSymbol = Factory.createFOL(argString);
        }

        if (argSymbol != null && (refTypes.contains(argSymbol.getType()))) {
          if (!varMap.containsKey(argSymbol)) {
            Variable var = Factory.createVariable("VAR" + varMap.size() + ":" + argSymbol.getType());
            varMap.put(argSymbol, var);

            output.addSupplementalSemantics(Factory.createPredicate(argSymbol.toUntypedString(), var));
            Symbol tier = Factory.createSymbol("DEFINITE");
            output.addTierAssignment(var, tier);

            processedArguments.add(var);
          } else {
            processedArguments.add(varMap.get(argSymbol));
          }
        } else {
          //TODO: this isn't always the case, if we want to allow variables to be generated by the LLM this needs to change and the prompt needs to be updated
          //Domain-specific safeguard to prevent the LLM from capitalizing arguments
          //If argument is a wildcard, don't want it to be capitalized by the LLM just because it recognizes it as a proper noun
          if (Character.isUpperCase(argString.charAt(0)) && argString.length() > 1) {
            try {
                TimeUnit.valueOf(argString);
                processedArguments.add(Factory.createSymbol(argString));
            } catch (IllegalArgumentException ignored) {
                String lowercaseArg = argString;
                lowercaseArg = Character.toLowerCase(lowercaseArg.charAt(0)) + lowercaseArg.substring(1);
                processedArguments.add(Factory.createSymbol(lowercaseArg));
            }
          } else {
            processedArguments.add(Factory.createSymbol(argString));
          }
        }
      }
    }
    return processedArguments;
  }

  @Override
  public Utterance parseNLGCompletionResponse(Utterance outgoingUtterance, Completion llmResponse) {
    log.info("[parseNLGCompletionResponse] " + llmResponse.getText());
    String responseString = llmResponse.getText().trim();

    Pattern outputPattern = Pattern.compile(".*\"?translation\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL);
    Matcher m = outputPattern.matcher(responseString);
    if (!m.matches()) {
      log.error("[parseNLGCompletionResponse] cannot find Output field in LLM response");
      return null;
    }

    if (outgoingUtterance.getSemantics().getName().equals("mean")) {
      //TODO: Do we need and/or want this? Validation NLG for learned actions - have no examples for these actions,
      //      so force standard generation? Ideally this check and override would occur somewhere else, but currently
      //      have learned action information here nicely
      Predicate semantics;
      List<Term> boundSemantics = outgoingUtterance.getBoundSemantics();
      if (boundSemantics.isEmpty()) {
        semantics = (Predicate) outgoingUtterance.getSemantics();
      } else {
        semantics = (Predicate) boundSemantics.get(0);
      }
      List<Symbol> args = semantics.getArgs();
      if (args.size() > 1) {
        String name = args.get(1).getName();
        if (learnedActionNames.contains(name)) {
          outgoingUtterance.setWords(String.format("Did you mean to ask me to perform the \"%s\" action?",String.join(" ", name.split("_"))));
        }
      }
    } else {
      String output = m.group(m.groupCount());
      outgoingUtterance.setWords(List.of(output.split(" ")));
    }

    return outgoingUtterance;
  }

  // Common utilities //

  protected String getConversationHistoryString(List<Utterance> conversationHistory) {
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

  // (Example now in SocialTRServiceProvider)
  //TODO: Extend to include pending and scheduled goals + all current goals - not just most recent
  protected List<Predicate> getCurrentGoals(Symbol agent) {
    try {
      List<Predicate> currentGoalPredicate = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalPredicatesForActor").argTypes(GoalFilter.class, Symbol.class))
              .call(List.class, GoalFilter.SYSTEM, agent);
      return currentGoalPredicate;
    } catch (TRADEException e) {
      log.error("[getWorldStateString] error getting current goal for agent", e);
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

  //TODO: does this need to be extended to agent pairs?
  protected String getCurrentQuestionsString(Symbol agent) {
    String output = "";
    try {
      String questionString = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestion").argTypes(Symbol.class)).call(String.class,agent);
      List<Term> expectedResponseForms = TRADE.getAvailableService(new TRADEServiceConstraints().name("getLastQuestionResponseForm").argTypes(Symbol.class)).call(List.class,agent);
      if (!expectedResponseForms.isEmpty()) {
        output = questionString + "\nEXPECTED REPLY FORM:\n" + String.join("\n",expectedResponseForms.stream().map(Term::toString).toList());
      }
    } catch (TRADEException e) {
      log.error("[getCurrentQuestionsString] error getting last question for agent");
    }
    if (output.isEmpty()) {
      output = "<There are no current outstanding questions>";
    }
    return output;
  }

  @Override
  protected void initLearnedActions() {
    super.initLearnedActions();
    for (ActionDBEntry entry : learnedActions) {
      Predicate signature = entry.getSignature(false);
      learnedActionsString += String.format("%s\n#\n", signature);
      learnedActionNames.add(signature.getName());
    }
  }

  @Override
  public void handleDatabaseAddition(ActionDBEntry entry) throws TRADEException {
    super.handleDatabaseAddition(entry);
    if (entry.isScript() && entry.getDBFile() == null && !entry.getName().equals("planned")) {
      log.info("[handleDatabaseAddition] have learned action {}", entry.getName());
      Predicate pred = entry.getSignature(false);
      learnedActionsString += String.format("%s\n#\n", pred);
      learnedActionNames.add(pred.getName());
    }
  }

  @Override
  public void handleDatabaseRemoval(ActionDBEntry entry) throws TRADEException {
    super.handleDatabaseRemoval(entry);
    Predicate pred = entry.getSignature(false);
    learnedActionsString = learnedActionsString.replace(String.format("%s\n#\n", pred),"");
    learnedActionNames.remove(pred.getName());
  }

  // Testing Code //
  @Action
  @TRADEService
  public void testLLMNLGWithString(String semantics) {
    Chat chat = new Chat();
    List<Utterance> conversationHistory = new ArrayList<>();
    Predicate semanticsPred = Factory.createPredicate(semantics);
    Utterance utterance = new Utterance.Builder().setSemantics(semanticsPred).setIsInputUtterance(false).setUtteranceType(UtteranceType.REPLY).setSpeaker(Factory.createSymbol("robotone")).setListener(Factory.createSymbol("pete")).build();
    chat = generateNLGChat(utterance);

    // Call LLM to get Completion
    Completion response;
    try {
      response = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Chat.class)).call(Completion.class, chat);
    } catch (TRADEException e) {
      log.error("[testLLMNLGWithString] Error performing chat completion", e);
      return;
    }
    utterance = parseNLGCompletionResponse(utterance, response);
    log.info("[testLLMNLGWithString] {}",utterance.getWordsAsString());

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("sayText").argTypes(String.class)).call(Void.class,utterance.getWordsAsString());
    } catch (TRADEException e) {
      log.error("[testLLMNLGWithString] Error calling sayText", e);
    }
  }
}
