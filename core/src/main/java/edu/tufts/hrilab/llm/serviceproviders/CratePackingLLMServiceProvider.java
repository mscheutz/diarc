/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.serviceproviders;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.LLMServiceProvider;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import org.apache.commons.cli.CommandLine;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

//TODO: Now copying a lot of shared templating across service providers, probably worth making another subclass layer
public class CratePackingLLMServiceProvider extends LLMServiceProvider {
  Prompt nluSystemPrompt = Prompts.getPrompt("cratepacking/nlu/system");
  Prompt nluUserPrompt = Prompts.getPrompt("cratepacking/nlu/user");
  Prompt nlgSystemPrompt = Prompts.getPrompt("cratepacking/nlg/system");
  Prompt nlgUserPrompt = Prompts.getPrompt("cratepacking/nlg/user");
  Prompt validationSystemPrompt = Prompts.getPrompt("cratepacking/validation/system");
  Prompt validationUserPrompt = Prompts.getPrompt("cratepacking/validation/user");
  private boolean usingWithParserTest = false;
  //TODO: All of this information should potentially be spread out to different parts of the system and gathered here
  //  on startup rather than hardcoded. E.g. descriptions from ActionDBEntries (asl pre-existing and new java annotations),
  //  NL examples from various parsers, selection of exposed actions from config, ... This is a fairly large architectural
  //  decision to discuss and therefore will need to wait until after first deployment.
  private class LLMActionEntry {
    String signature;
    String description;
    LLMNLExample example;

    LLMActionEntry(String signature, String description, LLMNLExample example) {
      this.signature = signature;
      this.description = description;
      this.example = example;
    }

    LLMActionEntry(String signature, String description, String exampleInstruction, String exampleTranslation, int exampleConfidence) {
      this.signature = signature;
      this.description = description;
      this.example = new LLMNLExample(exampleInstruction, exampleTranslation, exampleConfidence);
    }

    public String toString(boolean nlu) {
      String signatureString = signature;
      if (!nlu) {
        String[] parts = signatureString.split("\\(",2);
        signatureString = parts[0] + "(robotone";
        if (parts[1].equals(")")) {
          signatureString += parts[1];
        } else {
          signatureString += "," + parts[1];
        }
      }
      String output = signatureString + "\n" +
              "Description: " + description + "\n";
      if (nlu) {
        output +=
                "Example Translation: \"" +
                        example.instruction +
                        "\" -> {\"Semantics\":\"" +
                        example.semantics +
                        "\",\"Input Type\":\"INSTRUCT\",\"Confidence\":\"" +
                        example.confidence +
                        "\"}\n";
      }
      output += "#\n";
      return output;
    }
  }
  private class LLMNLExample {
    String instruction;
    String semantics;
    int confidence;

    LLMNLExample(String instruction, String semantics, int confidence) {
      this.instruction = instruction;
      this.semantics = semantics;
      this.confidence = confidence;
    }
  }
  private List<LLMActionEntry> exposedActionList = new ArrayList<>();

  @Override
  protected void init() {
    super.init();
    generateExposedActionList();
  }

  @Override
  protected void parseArgs (CommandLine cmdLine) {
    super.parseArgs(cmdLine);
    if (cmdLine.hasOption("japanese")) {
      nluSystemPrompt = Prompts.getPrompt("cratepacking/nlu/system");
      nluUserPrompt = Prompts.getPrompt("cratepacking/nlu/japanese/user");
      nlgSystemPrompt = Prompts.getPrompt("cratepacking/nlg/system");
      nlgUserPrompt = Prompts.getPrompt("cratepacking/nlg/japanese/user");
      validationUserPrompt = Prompts.getPrompt("cratepacking/nlg/japanese/validation/user");
      validationSystemPrompt = Prompts.getPrompt("cratepacking/nlg/japanese/validation/system");
    }
  }

  @Override
  public Utterance parseNLUCompletionResponse(Utterance incomingUtterance, Completion llmResponse) {
    try {
      Utterance.Builder output = new Utterance.Builder(incomingUtterance);
      log.info(llmResponse.getText());
      String responseString = llmResponse.getText();

      Pattern semanticsPattern = Pattern.compile(".*\"?Semantics\"?\\s*:\\s*\"?(.*)\"?,\\s*\"?Input Type\"?:.*", Pattern.DOTALL);
      Matcher m = semanticsPattern.matcher(responseString);
      if (!m.matches()) {
        log.error("[parseNLUCompletionResponse] cannot find Semantics result in LLM response");
        return incomingUtterance;
      }
      Predicate semantics = Factory.createPredicate(m.group(m.groupCount()));

      Pattern typePattern = Pattern.compile(".*\"?Input Type\"?\\s*:\\s*\"?(.*?)\"?,\\s*\"?Confidence\"?:.*", Pattern.DOTALL);
      m = typePattern.matcher(responseString);
      if (!m.matches()) {
        log.error("[parseNLUCompletionResponse] cannot find Input Type result in LLM response");
        return incomingUtterance;
      }
      UtteranceType utteranceType;
      try {
        utteranceType = UtteranceType.valueOf(m.group(m.groupCount()).trim());
      } catch (IllegalArgumentException e) {
        log.error("[parseNLUCompletionResponse] generated Input Type {} invalid", m.group(m.groupCount()));
        return incomingUtterance;
      }

      Pattern confidencePattern = Pattern.compile(".*\"?Confidence\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL);
      m = confidencePattern.matcher(responseString);
      if (!m.matches()) {
        log.error("[parseNLUCompletionResponse] cannot find Confidence result in LLM response");
        return incomingUtterance;
      }
      String confidenceString = m.group(m.groupCount()).trim();
      int confidence = Integer.parseInt(confidenceString);
      if (confidence < 5) {
        log.info("[parseNLUCompletionResponse] llm indicated parse result as low confidence ({})", confidence);
        //TODO: start conversation and recovery dialogue?
        //      Cache result so that script can be entered based on utterance/agent/something?
        return incomingUtterance;
      }

      output.setUtteranceType(utteranceType);

      Map<Symbol, Variable> varMap = new LinkedHashMap<>();
      List<Symbol> processedArguments = new ArrayList<>();
      if (utteranceType == UtteranceType.INSTRUCT) {
        String actorName = incomingUtterance.getAddressee().getName();
        processedArguments.add(Factory.createSymbol(actorName));
      }
      processedArguments = processSemantics(semantics, processedArguments, varMap, output, incomingUtterance.getAddressee().getName());
      if (processedArguments == null) {
        return incomingUtterance;
      }
      output.setSemantics(Factory.createPredicate(semantics.getName(), processedArguments));
      output.setNeedsValidation(true);

      Utterance result = output.build();
      boolean parsed = correctCommonNLUMistakes(result);
      if (parsed) {
        return result;
      } else {
        return incomingUtterance;
      }
    } catch (Exception e) {
      log.warn("[parseNLUCompletionResponse]",e);
      return incomingUtterance;
    }
  }

  //TODO: fix overlapping cases - also in cache:
  //      case where sizeInMM conflicts with direct setLength, etc based on whether a matching question exists
  //      case where sizeInMM conflicts with amount when no units are given
  private boolean correctCommonNLUMistakes(Utterance utterance) {
    String actorName = utterance.getAddressee().getName();
    try {
      String command = utterance.getSemantics().getName();
      if (command.startsWith("describe")) {
        utterance.setType(UtteranceType.INSTRUCT);
        List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
        args.add(0, Factory.createSymbol(actorName));
        utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
      } else if (command.equals("knowHowTo")) {
        List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
        args.add(0, Factory.createSymbol(actorName));
        Term knownPredicate = (Term) args.remove(1);
        List<Symbol> innerArgs = knownPredicate.getArgs();
        innerArgs.add(0,Factory.createSymbol(actorName));
        args.add(Factory.createPredicate(knownPredicate.getName(), innerArgs));
        utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
      }
    } catch (Exception e) {
      log.warn("[correctCommonNLUMistakes] unexpected semantics format");
      return false;
    }
    return true;
  }

  private List<Symbol> processSemantics(Predicate semantics, List<Symbol> processedArguments, Map<Symbol,Variable> varMap, Utterance.Builder output, String actor) {
    for (Symbol arg: semantics.getArgs()) {
      if (arg.isPredicate()) {
        List<Symbol> innerArgs = processSemantics((Predicate)arg,new ArrayList<>(),varMap,output,actor);
        if (innerArgs == null) {
          return null;
        }
        processedArguments.add(Factory.createPredicate(arg.getName(), innerArgs));
      } else {
        String argString = arg.toString();
        if (argString.startsWith("?")) {
          log.warn("[parseNLUCompletionResponse] NLU parse has unpopulated argument present, failing: {}", argString);
          return null;
        }
        Symbol argSymbol;
        String descriptorType = getDescriptorType(actor,argString);
        if (descriptorType.isEmpty()) {
          argSymbol = Factory.createFOL(argString);
        } else {
          //v2 only
          argSymbol = Factory.createSymbol(argString, descriptorType);
        }

        //Should generally be any reference, just know in this case all references will be locations
        if (argSymbol != null && (argSymbol.getType().equals("pose"))) {
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
            String lowercaseArg = argString;
            lowercaseArg = Character.toLowerCase(lowercaseArg.charAt(0)) + lowercaseArg.substring(1);
            processedArguments.add(Factory.createSymbol(lowercaseArg));
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
    log.info(llmResponse.getText());
    String responseString = llmResponse.getText().trim();

    Pattern outputPattern = Pattern.compile(".*\"?translation\"?\\s*:\\s*\"?(.*?)\"?\\s*}.*", Pattern.DOTALL);
    Matcher m = outputPattern.matcher(responseString);
    if (!m.matches()) {
      log.error("[parseNLGCompletionResponse] cannot find Output field in LLM response");
      return null;
    }
    String output = m.group(m.groupCount());
    outgoingUtterance.setWords(List.of(output.split(" ")));

    return outgoingUtterance;
  }

  @Override
  protected Chat generateNLGChat(Utterance utterance, List<Utterance> conversationHistory) {
    Chat chat = new Chat();

    //Intercept Validation Question and use specific validation prompt
    String userMessage;
    Predicate semantics;
    List<Term> boundSemantics = utterance.getBoundSemantics();
    if (boundSemantics.isEmpty()) {
      semantics = (Predicate) utterance.getSemantics();
    } else {
      semantics = (Predicate) boundSemantics.get(0);
    }
    List<String> englishPoses = getPoses(utterance.getSpeaker().getName(),true);
    List<String> japanesePoses = getPoses(utterance.getSpeaker().getName(),false);

    Map<String,String> promptVars = new LinkedHashMap<>();
    utterance = new Utterance.Builder(utterance).build();
    //Doesn't include suppSemantics
    semantics = replaceReferencesInSemantics(semantics);
    utterance.setSemantics(semantics);
    promptVars.put("%utterance", utterance.toString());
    if (semantics.getName().equals("mean")) {
      userMessage = validationUserPrompt.apply(promptVars);
    } else {
      userMessage = nlgUserPrompt.apply(promptVars);
    }
    chat.addUserMessage(userMessage);

    promptVars = new LinkedHashMap<>();
    promptVars.put("%addressee", utterance.getAddressee().getName());
    promptVars.put("%conversationHistory", getConversationHistoryString(conversationHistory));
    promptVars.put("%actionList", generateLLMActionListString(false));
    promptVars.put("%language", language);
    promptVars.put("%locationList", String.join("\n", englishPoses));
    promptVars.put("%japanesePoseList", String.join("\n", japanesePoses));
    if (semantics.getName().equals("mean")) {
      chat.setSystem(validationSystemPrompt.apply(promptVars));
    } else {
      chat.setSystem(nlgSystemPrompt.apply(promptVars));
    }

    log.info("generateNLGChat called for: {}", utterance);

    return chat;
  }

  Predicate replaceReferencesInSemantics(Predicate semantics) {
    List<Symbol> transformedArgs = new ArrayList<>();
    for (Symbol arg: semantics.getArgs()) {
      if (arg.isPredicate()) {
        transformedArgs.add(replaceReferencesInSemantics((Predicate) arg));
      } else {
        if (arg.getType().equals("pose")) {
          Map<Symbol, List<Term>> refs = new HashMap<>();
          try {
            refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateRE").argTypes(Symbol.class)).call(Map.class, arg);
            if (refs.get(arg).size() == 1) {
              transformedArgs.add(Factory.createSymbol(refs.get(arg).get(0).getName(),refs.get(arg).get(0).get(0).getType()));
            }
            else if (usingWithParserTest){
              transformedArgs.add(arg);
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
  protected Chat generateNLUChat(Utterance utterance, Chat currentChat, List<Utterance> conversationHistory){
    if (currentChat == null) {
      currentChat = new Chat();
    }

    Map<String,String> promptVars = new LinkedHashMap<>();
    promptVars.put("%actionList", generateLLMActionListString(true));
    promptVars.put("%conversationHistory", getConversationHistoryString(conversationHistory));
    promptVars.put("%goals", getGoalString(utterance.getAddressee()));
    promptVars.put("%questionList", getCurrentQuestionsString(utterance.getAddressee()));
    List<String> englishPoses = getPoses(utterance.getAddressee().getName(),true);
    List<String> japanesePoses = getPoses(utterance.getAddressee().getName(),false);
    promptVars.put("%locationList", String.join("\n", englishPoses));
    promptVars.put("%japanesePoseList", String.join("\n", japanesePoses));
    //TODO: if have relevant world state, reinclude
    //promptVars.put("%worldState", getWorldStateString(utterance.getAddressee()));
    currentChat.setSystem(nluSystemPrompt.apply(promptVars));
    promptVars.clear();
    promptVars.put("%utterance", utterance.getWordsAsString());
    currentChat.addUserMessage(nluUserPrompt.apply(promptVars));

    return currentChat;
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

  private String getGoalString(Symbol agent) {
    //TODO: also add pending and scheduled?
    String currentGoalString = "<The robot has no current task>\n";
    try {
      List<Predicate> currentGoalPredicate = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSystemGoalsPredicates").argTypes(Symbol.class)).call(List.class,agent);
      if (!currentGoalPredicate.isEmpty()) {
        currentGoalString = currentGoalPredicate.get(0).toString() + "\n";
      }
    } catch (TRADEException e) {
      log.error("[getWorldStateString] error getting current goal for agent", e);
    }
    return currentGoalString;
  }

  private String getCurrentQuestionsString(Symbol agent) {
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

  private List<String> getPoses(String agentName, boolean english) {
    try {
      List<String> propertyNames = new ArrayList<>();
      if (english) {
        TRADEServiceInfo tsi = getServiceForAgent(agentName,"getEnglishPoseDescriptors");
        if (tsi != null) {
          propertyNames = tsi.call(List.class);
        }
        else if (usingWithParserTest){
          return Arrays.asList("location one", "location two", "location three", "location four", "location five");
        }
      } else {
        TRADEServiceInfo tsi = getServiceForAgent(agentName,"getJapanesePoseDescriptors");
        if (tsi != null) {
          propertyNames = tsi.call(List.class);
        }
        else if (usingWithParserTest){
          return Arrays.asList("場所1","場所2","場所3","場所4","場所5");
        }
      }
      return propertyNames;
    } catch (TRADEException e) {
      log.error("[getPoseListString]", e);
    }

    return new ArrayList<>();
  }

  private String getDescriptorType(String agentName, String descriptor) {
    List<String> propertyNames;
    propertyNames = getPoses(agentName, true);
    if (propertyNames.contains(descriptor)) {
      return "pose";
    }
    return "";
  }

  //Hacky impl of getting japanese version of descriptors used in action scripts for TTS
  @TRADEService
  public String getJapaneseVersionOfDescriptor(Symbol descriptor) {
    return getJapaneseVersionOfDescriptor(descriptor.getName());
  }
  //TODO: use actor
  @TRADEService
  public String getJapaneseVersionOfDescriptor(String descriptorString) {
    List<String> englishDescriptors = getPoses("robotone",true);
    List<String> japaneseDescriptors = getPoses("robotone",false);
    int index = englishDescriptors.indexOf(descriptorString);
    if (index != -1 && index < japaneseDescriptors.size()) {
      return japaneseDescriptors.get(index);
    }
    return descriptorString;
  }

  @TRADEService
  @Action
  public boolean ignoreTentativeAccept(Predicate goalPred){
    log.info("[ignoreTentativeAccept] goalPred: "+goalPred);
    if(goalPred != null) {
      switch (goalPred.getName()) {
        case "reportAllPackingParameters":
        case "reportFullyPackedCrateConfiguration":
        case "reportMaxNumberOfPackedItemsPerLayer":
        case "reportMaxNumberOfPackedLayers":
        case "reportMaxNumberOfPackedItems":
        case "reportCurrentPackedCrateConfiguration":
        case "reportCurrentNumberOfLayersToPack":
        case "reportCurrentNumberOfItemsToPack":
        case "reportCrateDimensions":
        case "reportCrateHeight":
        case "reportCrateWidth":
        case "reportCrateLength":
        case "reportPackedItemDimensions":
        case "reportPackedItemHeight":
        case "reportPackedItemWidth":
        case "reportPackedItemLength":
        case "setNumberOfItemsToPackPerCrate":
        case "repeatLastQuestion":
        case "describeCurrentGoal":
        case "reportTaughtActions":
        case "reportInstructableActions":
        case "describeActionSteps":
        case "reportIfCanPerform":
        case "reportKioskSettings":
        case "sayText":
        case "say":
          return true;
        default:
          return false;
      }
    }
    return false;
  }

  //TODO: Able to move at least some of this to the base class?
  @TRADEService
  public List<String> getInstructableActions() {
    return new ArrayList<>(exposedActionList.stream().map(a -> a.signature.split("\\(")[0]).toList());
  }

  private String generateLLMActionListString(boolean nlu) {
    StringBuilder sb = new StringBuilder();
    for (LLMActionEntry entry : exposedActionList) {
      sb.append(entry.toString(nlu));
    }
    return sb.toString();
  }

  private void generateExposedActionList() {
    exposedActionList = new ArrayList<>();
    LLMActionEntry entry;
    entry = new LLMActionEntry("packItemsIntoCrates()",
            "You begin the task of packing items into crates",
            "Start packing",
            "packItemsIntoCrates()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("configurePackingParameters()",
            "You engage in a dialogue interaction with the user to set all required parameters for the packing task",
            "I would like to set the packing parameters",
            "configurePackingParameters()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportPackedItemLength()",
            "You say how long you believe the food items to be packed are",
            "How long is one of the food containers",
            "reportPackedItemLength()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportPackedItemWidth()",
            "You say how wide you believe the food items to be packed are",
            "How wide is one of the food containers",
            "reportPackedItemWidth()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportPackedItemHeight()",
            "You say how tall you believe the food items to be packed are",
            "How tall is one of the food containers",
            "reportPackedItemHeight()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportPackedItemDimensions()",
            "You say what the length, width, and height of the food items to be packed are",
            "What are the dimensions of the packed items",
            "reportPackedItemDimensions()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportCrateLength()",
            "You say how long you believe one of the crates into which items are packed into is",
            "How long is the crate",
            "reportCrateLength()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportCrateWidth()",
            "You say how wide you believe one of the crates into which items are packed into is",
            "How wide is the crate",
            "reportCrateWidth()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportCrateHeight()",
            "You say how tall you believe one of the crates into which items are packed into is",
            "How tall is the crate",
            "reportCrateHeight()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportCrateDimensions()",
            "You say what the length, width, and height of the crate into which items are being packed into are",
            "What are the dimensions of the crate",
            "reportCrateDimensions()",
            10
    );
    exposedActionList.add(entry);
    //TODO: below three just merged to always answer as if the 3rd question was asked?
    entry = new LLMActionEntry("reportCurrentNumberOfItemsToPack()",
            "You say how many items you are currently configured to pack into each crate",
            "How many items are you packing into a crate",
            "reportCurrentNumberOfItemsToPack()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportCurrentNumberOfLayersToPack()",
            "You say how many layers of items would be present in a packed crate under your current packing parameters",
            "How many layers would exist in a packed crate",
            "reportCurrentNumberOfLayersToPack()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportCurrentPackedCrateConfiguration()",
            "You report the currently configured number of items, layers, and items per layer to be packed in each crate.",
            "How fully are you currently packing crates",
            "reportCurrentPackedCrateConfiguration()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportMaxNumberOfPackedItems()",
            "You say the maximum number of items you can pack into each crate",
            "How many items can you fit into a crate",
            "reportMaxNumberOfPackedItems()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportMaxNumberOfPackedLayers()",
            "You say how many layers of items would be present in a crate if you packed it fully",
            "How many layers would exist in a fully packed crate",
            "reportMaxNumberOfPackedLayers()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportMaxNumberOfPackedItemsPerLayer()",
            "You say how many items would be in a single, fully packed layer",
            "How many items would be in a full layer",
            "reportMaxNumberOfPackedItemsPerLayer()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportFullyPackedCrateConfiguration()",
            "You report the maximum number of items, layers and items per layer which could fit into a crate if you packed it as full as possible.",
            "What would a fully packed crate look like",
            "reportFullyPackedCrateConfiguration()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("reportAllPackingParameters()",
            "You report all relevant packing parameters you are aware of; The dimensions of food items, the dimensions of the crates, your current packed crate configuration, and the fully packed crate configuration.",
            "Tell me your overall packing configuration",
            "reportAllPackingParameters()",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("setNumberOfItemsToPackPerCrate(?numItems)",
            "You directly configure how many items you will pack into each crate",
            "You should pack ten items at a time",
            "setNumberOfItemsToPackPerCrate(10)",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("setPackedItemDimensions(?itemLength, ?itemWidth, ?itemHeight)",
            "You directly configure all three dimensions of the food items you are packing. Each dimension is measured in millimeters. This is only a valid semantic form if all three dimensions are given.",
            "The packed food items are 0.5 meter by 400 by 200 millimeters",
            "setPackedItemDimensions(500,400,200)",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("setCrateDimensions(?crateLength, ?crateWidth, ?crateHeight)",
            "You directly configure all three dimensions of the crate you are packing items into. Each dimension is measured in millimeters. This is only a valid semantic form if all three dimensions are given.",
            "The crates are 2 meters long, 1.5 meters wide, and 400 mm tall",
            "setPackedItemDimensions(2000,1500,400)",
            10
    );
    exposedActionList.add(entry);
    entry = new LLMActionEntry("describeCurrentGoal()",
            "You report what your current task is",
            "what are you doing now",
            "describeCurrentGoal()", 10);
    exposedActionList.add(entry);
    entry = new LLMActionEntry("cancelSystemGoals()",
            "You stop and fully discard your current task",
            "Stop packing",
            "cancelSystemGoals()", 10);
    exposedActionList.add(entry);
    entry = new LLMActionEntry("suspendSystemGoalsBlocking()",
            "You pause your current task, which can be later resumed",
            "Pause packing",
            "suspendSystemGoalsBlocking()", 10);
    exposedActionList.add(entry);
    entry = new LLMActionEntry("resumeSystemGoals()",
            "You resume the currently paused task",
            "Continue packing",
            "resumeSystemGoals()", 10);
    exposedActionList.add(entry);
    //TODO: report how to pack?
//    entry = new LLMActionEntry("",
//            "",
//            "",
//            "",
//            10
//    );
//    exposedActionList.add(entry);
  }

  TRADEServiceInfo getServiceForAgent(String agentName, String serviceName, Class<?>... argTypes) {
    HashSet<TRADEServiceInfo> services = (HashSet<TRADEServiceInfo>) TRADE.getAvailableServices(new TRADEServiceConstraints().name(serviceName).argTypes(argTypes));
    for (TRADEServiceInfo tsi : services) {
      Collection<String> groups = tsi.getGroups();
      for (String group : groups) {
        if (group.contains(agentName)) {
          return tsi;
        }
      }
    }
    log.warn("[getServiceForAgent] could not find service {} for agent {}", serviceName, agentName);
    return null;
  }
}
