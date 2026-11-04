/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.serviceproviders.tr;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.goal.GoalFilter;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.slug.common.Utterance;

import java.util.*;

//TODO: Now copying a lot of shared templating across service providers, probably worth making another subclass layer
public class CratePackingLLMServiceProvider extends TRLLMServiceProvider {
  private boolean usingWithParserTest = false;
  //TODO: All of this information should potentially be spread out to different parts of the system and gathered here
  //  on startup rather than hardcoded. E.g. descriptions from ActionDBEntries (asl pre-existing and new java annotations),
  //  NL examples from various parsers, selection of exposed actions from config, ... This is a fairly large architectural
  //  decision to discuss and therefore will need to wait until after first deployment.
  private class LLMSemanticEntry {
    String signature;
    String description;
    LLMSemanticExample example;

    LLMSemanticEntry(String signature, String description, LLMSemanticExample example) {
      this.signature = signature;
      this.description = description;
      this.example = example;
    }

    LLMSemanticEntry(String signature, String description, String exampleInstructionEnglish, String exampleInstructionJapanese, String exampleTranslation, int exampleConfidence, String type) {
      this.signature = signature;
      this.description = description;
      Map<String,String> exampleInstructions = new HashMap<>();
      exampleInstructions.put("English", exampleInstructionEnglish);
      exampleInstructions.put("Japanese", exampleInstructionJapanese);
      this.example = new LLMSemanticExample(exampleInstructions, exampleTranslation, exampleConfidence, type);
    }

    public String toString(boolean generateExample, String language) {
      String signatureString = signature;
      //if (example.type.equals("INSTRUCT")) {
      //  String[] parts = signatureString.split("\\(",2);
      //  signatureString = parts[0] + "(robotone";
      //  if (parts[1].equals(")")) {
      //    signatureString += parts[1];
      //  } else {
      //    signatureString += "," + parts[1];
      //  }
      //}
      String output = signatureString + "\n" +
              "Description: " + description + "\n";
      if (generateExample) {
        String exampleInstruction = example.instructions.get(language);
        if (exampleInstruction == null) {
          log.warn("[LLMActionEntry] no example found for language: {}", language);
          exampleInstruction = example.instructions.get("English");
        }
        output +=
                "Example Translation: \"" +
                        exampleInstruction +
                        "\" -> {\"Semantics\":\"" +
                        example.semantics +
                        "\",\"Input Type\":\"" +
                        example.type +
                        "\",\"Confidence\":\"" +
                        example.confidence +
                        "\"}\n";
      }
      output += "#\n";
      return output;
    }
  }
  private class LLMSemanticExample {
    Map<String,String> instructions;
    String semantics;
    int confidence;
    String type;

    LLMSemanticExample(Map<String,String> instructions, String semantics, int confidence, String type) {
      this.instructions = instructions;
      this.semantics = semantics;
      this.confidence = confidence;
      this.type = type;
    }
  }
  private List<LLMSemanticEntry> exposedInstructList = new ArrayList<>();
  private List<LLMSemanticEntry> exposedReplyList = new ArrayList<>();
  private String exposedInstructListNLUString;
  private String exposedInstructListNLGString;
  private String exposedReplyListNLUString;
  private String exposedReplyListNLGString;

  private class LLMNLGExample {
    String predString;
    Map<String,String> exampleGenerations;
    String inputType;

    LLMNLGExample(String predString, Map<String,String> exampleGenerations, String inputType) {
      this.predString = predString;
      this.exampleGenerations = exampleGenerations;
      this.inputType = inputType;
    }
    LLMNLGExample(String predString, String englishTranslation, String japaneseTranslation, String inputType) {
      this.predString = predString;
      Map<String,String> generations = new HashMap<>();
      generations.put("English", englishTranslation);
      generations.put("Japanese", japaneseTranslation);
      this.exampleGenerations = generations;
      this.inputType = inputType;
    }

    public String toString(String language, boolean validation) {
      String exampleGeneration = exampleGenerations.get(language);
      if (exampleGeneration == null || exampleGeneration.isEmpty()) {
        log.warn("[LLMNLGExample] no example found for language: {}", language);
        return "";
      }
      if (validation) {
        return "Input:" + inputType + "(robotone,%addressee,mean(%addressee," + predString + "),{})\n" +
                "Output:{\"translation\":\"" + exampleGeneration + "\"}\n" +
                "#\n";
      } else {
        return "Input:" + inputType + "(robotone,%addressee," + predString + ",{})\n" +
                "Output:{\"translation\":\"" + exampleGeneration + "\"}\n" +
                "#\n";
      }
    }
  }
  private List<LLMNLGExample> nlgExampleList = new ArrayList<>();
  private List<LLMNLGExample> validationExampleList = new ArrayList<>();
  private String nlgExampleListString;
  private String validationExampleListString;

  @Override
  protected void init() {
    super.init();
    generateExposedActionList();
    generateExposedReplyList();
    generateNLGExampleList();
    generateValidationExampleList();
    generatePromptExampleLists();
    processMatchingNLUParseLiteralsAsRefs = true;
  }

  public void setLanguage(String language) {
    if (!(language.equals("English") || language.equals("Japanese"))) {
      log.error("[setLanguage] only languages currently supported are \"English\" or \"Japanese\"");
      return;
    }
    this.language = language;
    generatePromptExampleLists();
  }

  private void generatePromptExampleLists() {
    exposedInstructListNLGString = generateLLMInstructListString(false);
    exposedInstructListNLUString = generateLLMInstructListString(true);
    exposedReplyListNLGString = generateLLMReplyListString(false);
    exposedReplyListNLUString = generateLLMReplyListString(true);
    nlgExampleListString = generateLLMNLGExampleListString();
    validationExampleListString = generateLLMValidationExampleListString();
  }

  @Override
  protected Prompt getNluSystemPrompt() {
    return Prompts.getPrompt("cratepacking/nlu/system");
  }

  @Override
  protected Prompt getNluUserPrompt() {
    return Prompts.getPrompt("cratepacking/nlu/user");
  }

  @Override
  protected Prompt getNlgSystemPrompt() {
    return Prompts.getPrompt("cratepacking/nlg/system");
  }

  @Override
  protected Prompt getNlgUserPrompt() {
    return Prompts.getPrompt("cratepacking/nlg/user");
  }

  @Override
  protected Prompt getValidationSystemPrompt() {
    return Prompts.getPrompt("cratepacking/validation/system");
  }

  @Override
  protected Prompt getValidationUserPrompt() {
    return Prompts.getPrompt("cratepacking/validation/user");
  }

  @Override
  protected Map<String, String> populatePromptVars(Utterance utterance, Map<String, String> promptVars, boolean system, boolean parsing, boolean validation) {
    if (parsing) {
      if (system) {
        promptVars.put("%actionList", exposedInstructListNLUString);
        promptVars.put("%semanticList", exposedReplyListNLUString);
        //Double check, I believe this is addressed generally elsewhere now
        ////TODO: revisit, don't have time to fully address - for now removing validation interactions from the conversation
        ////      history in order to avoid biasing future parsing attempts
        ////      Will have floating yes/no responses to validation currently though
        //conversationHistory = conversationHistory.stream().filter(u -> !(u.needsValidation() || (u.getSemantics() != null && u.getSemantics().getName().equals("mean"))))
        //       .toList();
        promptVars.put("%goals", getGoalString(utterance.getAddressee()));
        promptVars.put("%questionList", getCurrentQuestionsString(utterance.getAddressee()));
        promptVars.put("%language", language);
        promptVars.put("%addressee", utterance.getAddressee().getName());
        List<String> poses;
        if (language.equals("Japanese")) {
          poses = getPoses(utterance.getAddressee().getName(),false);
          promptVars.put("%examplePose", poses.isEmpty() ? "試験場所" : poses.get(0));
        } else {
          poses = getPoses(utterance.getAddressee().getName(),true);
          promptVars.put("%examplePose", poses.isEmpty() ? "test location" : poses.get(0));
        }
        promptVars.put("%poseList", String.join("\n", poses));
        //TODO: if have relevant world state, reinclude
        //promptVars.put("%worldState", getWorldStateString(utterance.getAddressee()));
      } else {
        //TODO: build in notion of homophones more explicitly into serviceproviders (or this should really be done in the ASR app)
        //Preprocess utterance (intended for ASR results)
        String utteranceString = utterance.getWordsAsString();
        utteranceString = utteranceString.replaceAll("饅頭","番重");
        utteranceString = utteranceString.replaceAll("まんじゅう","番重");
        utteranceString = utteranceString.replaceAll("バンジー","番重");
        utteranceString = utteranceString.replaceAll("見つけ作業","積み付け作業");
        utteranceString = utteranceString.replaceAll("墨付け","積み付け");
        utteranceString = utteranceString.replaceAll("つみつけ","積み付け");
        utteranceString = utteranceString.replaceAll("素材パック","惣菜パック");
        promptVars.put("%utterance", utteranceString);
      }
    } else {
      if (system) {
        promptVars.put("%language", language);
        promptVars.put("%actionList", exposedInstructListNLGString);
        promptVars.put("%semanticList", exposedReplyListNLGString);
        if (validation) {
          //TODO: gen once per swapping language,
          promptVars.put("%exampleList", validationExampleListString);
          List<String> poses;
          if (language.equals("Japanese")) {
            poses = getPoses(utterance.getAddressee().getName(),false);
            promptVars.put("%examplePose", poses.isEmpty() ? "試験場所" : poses.get(0));
          } else {
            poses = getPoses(utterance.getAddressee().getName(),true);
            promptVars.put("%examplePose", poses.isEmpty() ? "test location" : poses.get(0));
          }
          promptVars.put("%addressee", utterance.getAddressee().getName());
        } else {
          promptVars.put("%exampleList", nlgExampleListString);
          promptVars.put("%addressee", utterance.getAddressee().getName());
        }
      } else {
        if (validation) {
          //Quick fix - change sizeInMM to lengthInMM/widthInMM/heightInMM based on current outstanding question
          String utteranceString = utterance.toString();
          String relevantQuestion = getFirstQuestion(utterance.getSpeaker());
          if (relevantQuestion.contains("sizeInMM")) {
            if (relevantQuestion.contains("long") || relevantQuestion.contains("長さ")) {
              utteranceString = utteranceString.replaceAll("sizeInMM","lengthInMM");
            } else if (relevantQuestion.contains("wide") || relevantQuestion.contains("幅")) {
              utteranceString = utteranceString.replaceAll("sizeInMM","widthInMM");
            } else if (relevantQuestion.contains("tall") || relevantQuestion.contains("高さ")) {
              utteranceString = utteranceString.replaceAll("sizeInMM","heightInMM");
            }
          }
          promptVars.put("%utterance", utteranceString);
        } else {
          promptVars.put("%utterance", utterance.toString());
        }
      }
    }
    return promptVars;
  }

  @Override
  protected List<String> getRefTypes() {
    return List.of("pose");
  }

  @Override
  //TODO: fix overlapping cases - also in cache:
  //      case where sizeInMM conflicts with direct setLength, etc based on whether a matching question exists
  //      case where sizeInMM conflicts with amount when no units are given
  protected Utterance correctCommonNLUMistakes(Utterance utterance) {
    try {
      String actorName = utterance.getAddressee().getName();
      String command = utterance.getSemantics().getName();
      //if (command.startsWith("describe")) {
      //  utterance.setType(UtteranceType.INSTRUCT);
      //  List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
      //  args.add(0, Factory.createSymbol(actorName));
      //  utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
      //} else
      if (command.equals("knowHowTo")) {
        List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
        args.add(0, Factory.createSymbol(actorName));
        Term knownPredicate = (Term) args.remove(1);
        List<Symbol> innerArgs = knownPredicate.getArgs();
        innerArgs.add(0,Factory.createSymbol(actorName));
        args.add(Factory.createPredicate(knownPredicate.getName(), innerArgs));
        utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
      } else if (command.equals("goTo")) {
        List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
        utterance.setSemantics(Factory.createPredicate("goToPoseDirect", args));
      }
    } catch (Exception e) {
      log.warn("[correctCommonNLUMistakes] unexpected semantics format");
      return null;
    }
    return utterance;
  }

  private String getGoalString(Symbol agent) {
    //TODO: also add pending and scheduled?
    String currentGoalString = "<The robot has no current task>\n";
    try {
      List<Predicate> currentGoalPredicate = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalPredicatesForActor").argTypes(GoalFilter.class, Symbol.class))
              .call(List.class, GoalFilter.SYSTEM, agent);
      if (!currentGoalPredicate.isEmpty()) {
        currentGoalString = currentGoalPredicate.get(0).toString() + "\n";
      }
    } catch (TRADEException e) {
      log.error("[getWorldStateString] error getting current goal for agent", e);
    }
    return currentGoalString;
  }

  private String getFirstQuestion(Symbol agent) {
    String output = "";
    try {
      List<String> questionStrings = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllQuestions").argTypes(Symbol.class)).call(List.class,agent);
      if (!questionStrings.isEmpty()) {
        output = questionStrings.get(0);
      }
      List<List<Term>> allExpectedResponseForms = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllQuestionResponseForms").argTypes(Symbol.class)).call(List.class,agent);
      if (!allExpectedResponseForms.isEmpty()) {
        List<Term> expectedResponseForms = allExpectedResponseForms.get(0);
        output += "\nEXPECTED REPLY FORM:\n" + String.join("\n",expectedResponseForms.stream().map(Term::toString).toList());
      }
    } catch (TRADEException e) {
      log.error("[getCurrentQuestionsString] error getting last question for agent");
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

  @Override
  protected String getDescriptorType(String agentName, String descriptor) {
    List<String> propertyNames;
    propertyNames = getPoses(agentName, language.equals("English"));
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

  //TODO: Able to move at least some of this to the base class?
  @TRADEService
  public List<String> getInstructableActions() {
    return new ArrayList<>(exposedInstructList.stream().map(a -> a.signature.split("\\(")[0]).toList());
  }

  private String generateLLMInstructListString(boolean nlu) {
    StringBuilder sb = new StringBuilder();
    for (LLMSemanticEntry entry : exposedInstructList) {
      sb.append(entry.toString(nlu,language));
    }
    return sb.toString();
  }

  private void generateExposedActionList() {
    exposedInstructList = new ArrayList<>();
    LLMSemanticEntry entry;
    entry = new LLMSemanticEntry("packItemsIntoCrates()",
            "You begin the task of packing items into crates",
            "Start packing",
            "積み付け作業をお願いしたいです",
            "packItemsIntoCrates()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("configurePackingParameters()",
            "You engage in a dialogue interaction with the user to set all required parameters for the packing task",
            "I would like to set the packing parameters",
            "積み付けパラメータを設定したいです",
            "configurePackingParameters()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportPackedItemLength()",
            "You say how long you believe the food items to be packed are",
            "How long is one of the food containers",
            "惣菜パックの長さはいくつですか",
            "reportPackedItemLength()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportPackedItemWidth()",
            "You say how wide you believe the food items to be packed are",
            "How wide is one of the food containers",
            "惣菜パックの幅はいくつですか",
            "reportPackedItemWidth()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportPackedItemHeight()",
            "You say how tall you believe the food items to be packed are",
            "How tall is one of the food containers",
            "惣菜パックの高さはいくつですか",
            "reportPackedItemHeight()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportPackedItemDimensions()",
            "You say what the length, width, and height of the food items to be packed are",
            "What are the dimensions of the packed items",
            "惣菜パックのサイズを教えてください",
            "reportPackedItemDimensions()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportCrateLength()",
            "You say how long you believe one of the crates into which items are packed into is",
            "How long is the crate",
            "番重の長さはいくつですか",
            "reportCrateLength()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportCrateWidth()",
            "You say how wide you believe one of the crates into which items are packed into is",
            "How wide is the crate",
            "番重の幅はいくつですか",
            "reportCrateWidth()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportCrateHeight()",
            "You say how tall you believe one of the crates into which items are packed into is",
            "How tall is the crate",
            "積み付け対象の番重の高さはいくつですか",
            "reportCrateHeight()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportCrateDimensions()",
            "You say what the length, width, and height of the crate into which items are being packed into are",
            "What are the dimensions of the crate",
            "番重のサイズを教えてください",
            "reportCrateDimensions()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    //TODO: below three just merged to always answer as if the 3rd question was asked?
    entry = new LLMSemanticEntry("reportConfiguredNumberOfItemsToPack()",
            "You say how many items you are currently configured to pack into each crate",
            "How many items are you packing into a crate",
            "1つの番重にいくつの惣菜パックを積み付けるよう設定されていますか",
            "reportConfiguredNumberOfItemsToPack()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportCurrentNumberOfLayersToPack()",
            "You say how many layers of items would be present in a packed crate under your current packing parameters",
            "How many layers would exist in a packed crate",
            "番重には何層の惣菜パックが積み重なりますか",
            "reportCurrentNumberOfLayersToPack()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportCurrentPackedCrateConfiguration()",
            "You report the currently configured number of items, layers, and items per layer to be packed in each crate.",
            "How fully are you currently packing crates",
            "現在、番重はどの程度まで積み付けられていますか",
            "reportCurrentPackedCrateConfiguration()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportMaxNumberOfPackedItems()",
            "You say the maximum number of items you can pack into each crate",
            "How many items can you fit into a crate",
            "1つの番重に最大でいくつ入りますか",
            "reportMaxNumberOfPackedItems()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportMaxNumberOfPackedLayers()",
            "You say how many layers of items would be present in a crate if you packed it fully",
            "How many layers would exist in a fully packed crate",
            "番重を満杯にした場合、何層になりますか",
            "reportMaxNumberOfPackedLayers()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportMaxNumberOfPackedItemsPerLayer()",
            "You say how many items would be in a single, fully packed layer",
            "How many items would be in a full layer",
            "各層にはいくつの惣菜パックが入りますか",
            "reportMaxNumberOfPackedItemsPerLayer()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportFullyPackedCrateConfiguration()",
            "You report the maximum number of items, layers and items per layer which could fit into a crate if you packed it as full as possible.",
            "What would a fully packed crate look like",
            "満杯になった番重の状態を教えてください",
            "reportFullyPackedCrateConfiguration()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportAllPackingParameters()",
            "You report the state of all relevant packing parameters you are aware of; The dimensions of food items, the dimensions of the crates, your current packed crate configuration, and the fully packed crate configuration.",
            "Tell me your overall packing configuration",
            "全体の積み付け設定を教えてください",
            "reportAllPackingParameters()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportNumberOfItemsInCrate()",
            "You report the number of items which have been packed so far into the current or last crate",
            "How many items are in the crate?",
            "番重の中にいくつの惣菜パックがありますか？",
            "reportNumberOfItemsInCrate()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportWhatPackingParametersExist()",
            "You report the total list of packing parameters which exist and can be configured",
            "What parameters can be set before packing?",
            "番重の積付けに必要な設定を教えて下さい？",
            "reportWhatPackingParametersExist()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reportUnsetPackingParameters()",
            "You report the list of packing parameters which the user has not yet configured and still needs to before beginning to pack",
            "What do I need to set before packing?",
            "番重の積み付けのために、他に必要な設定はなんですか？ OR 何が必要ですか?",
            "reportUnsetPackingParameters()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("setNumberOfItemsToPackPerCrate(?numItems)",
            "You directly configure how many items you will pack into each crate",
            "You should pack ten items at a time",
            "一度に10個ずつ積み付けてください",
            "setNumberOfItemsToPackPerCrate(10)",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("setPackedItemDimensions(?itemLength, ?itemWidth, ?itemHeight)",
            "You directly configure all three dimensions of the food items you are packing. Each dimension is measured in millimeters. This is only a valid semantic form if all three dimensions are given.",
            "The packed food items are 0.5 meter by 400 by 200 millimeters",
            "惣菜パックのサイズは、長さ0.5メートル、幅400mm、高さ200mmです",
            "setPackedItemDimensions(500,400,200)",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("setPackedItemLength(?itemLength)",
            "You directly configure the length of the food items you are packing in millimeters",
            "",
            "",
            "setPackedItemLength(200)",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("setPackedItemWidth(?itemWidth)",
            "You directly configure the width of the food items you are packing in millimeters",
            "",
            "",
            "setPackedItemWidth(200)",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("setPackedItemHeight(?itemHeight)",
            "You directly configure the height of the food items you are packing in millimeters",
            "",
            "",
            "setPackedItemHeight(200)",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("setCrateDimensions(?crateLength, ?crateWidth, ?crateHeight)",
            "You directly configure all three dimensions of the crate you are packing items into. Each dimension is measured in millimeters. This is only a valid semantic form if all three dimensions are given.",
            "The crates are 2 meters long, 1.5 meters wide, and 400 mm tall",
            "番重のサイズは、長さ2メートル、幅1.5メートル、高さ400mmです",
            "setPackedItemDimensions(2000,1500,400)",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("setCrateLength(?itemLength)",
            "You directly configure the length of the crate you are packing into in millimeters",
            "",
            "",
            "setCrateLength(200)",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("setCrateWidth(?itemWidth)",
            "You directly configure the width of the crate you are packing into in millimeters",
            "",
            "",
            "setCrateWidth(200)",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("setCrateHeight(?itemHeight)",
            "You directly configure the height of the crate you are packing into in millimeters",
            "",
            "",
            "setCrateHeight(200)",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("describeCurrentGoal()",
            "You report what your current task is",
            "what are you doing now",
            "今、何をしていますか",
            "describeCurrentGoal()", 10,
            "INSTRUCT");
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("cancelSystemGoals()",
            "You stop and fully discard your current task",
            "Stop packing",
            "停止してください",
            "cancelSystemGoals()", 10,
            "INSTRUCT");
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("suspendSystemGoalsBlocking()",
            "You pause your current task, which can be later resumed",
            "Pause packing",
            "一時停止してください",
            "suspendSystemGoalsBlocking()", 10,
            "INSTRUCT");
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("resumeSystemGoals()",
            "You resume the currently paused task",
            "Continue packing",
            "再開してください",
            "resumeSystemGoals()", 10,
            "INSTRUCT");
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("repeatLastQuestion()",
            "You repeat the last question you asked, which the human still needs to answer",
            "could you repeat that",
            "質問をもう一度お願いします",
            "repeatLastQuestion()", 10,
            "INSTRUCT");
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("goToPoseDirect(?destination:pose)",
            "You go to location ?destination:pose.",
            "please navigate to %examplePose",
            "%examplePose番にしてください",
            "goToPoseDirect(%examplePose:pose)",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reconfigureCrateDimensions()",
            "The user is telling you that they have changed the type of crate you are packing into or that the dimensions of the crate have changed without supplying any actual values",
            "I",
            "番重 のパラメータを設定したいです",
            "reconfigureCrateDimensions()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("reconfigurePackedItemDimensions()",
            "The user is telling you that they have changed the type of items you are packing or that the dimensions of the packed items have changed without supplying any actual values",
            "",
            "惣菜パック のパラメータを設定したいです",
            "reconfigurePackedItemDimensions()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("openGripper()",
            "The user is telling you to let go of an object by turning off the vacuum of the suction gripper",
            "open gripper",
            "グリッパーを開いてください",
            "openGripper()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
    entry = new LLMSemanticEntry("closeGripper()",
            "The user is telling you to latch onto an object by turning on the vacuum of the suction gripper",
            "close gripper",
            "グリッパーを閉めてください",
            "closeGripper()",
            10,
            "INSTRUCT"
    );
    exposedInstructList.add(entry);
//    entry = new LLMActionEntry("",
//            "",
//            "",
//            "",
//            10,
//            "INSTRUCT"
//    );
//    exposedInstructList.add(entry);
  }

  private String generateLLMReplyListString(boolean nlu) {
    StringBuilder sb = new StringBuilder();
    for (LLMSemanticEntry entry : exposedReplyList) {
      sb.append(entry.toString(nlu,language));
    }
    return sb.toString();
  }

  private void generateExposedReplyList() {
    exposedReplyList = new ArrayList<>();
    LLMSemanticEntry entry;
    entry = new LLMSemanticEntry("val(yes)",
            "Affirmative response to a yes/no question",
            "yeah",
            "はい",
            "val(yes)",
            10,
            "REPLY"
    );
    exposedReplyList.add(entry);
    entry = new LLMSemanticEntry("val(no)",
            "Negative response to a yes/no question",
            "no",
            "いいえ",
            "val(no)",
            10,
            "REPLY"
    );
    exposedReplyList.add(entry);
    entry = new LLMSemanticEntry("sizeInMM(?integer)",
            "Answer to a question about the size of a dimension of something, indicated in millimeters",
            "600mm",
            "600mmです",
            "sizeInMM(600)",
            10,
            "REPLY"
    );
    exposedReplyList.add(entry);
    entry = new LLMSemanticEntry("amount(?integer)",
            "Answer to a question about the amount of items which should be packed into a crate",
            "20 items",
            "20個です",
            "amount(20)",
            10,
            "REPLY"
    );
    exposedReplyList.add(entry);
    entry = new LLMSemanticEntry("abort(question)",
            "Conveyed intent to decline answering an asked question",
            "never mind",
            "気にしないでください",
            "abort(question)",
            10,
            "REPLY"
    );
    exposedReplyList.add(entry);
    entry = new LLMSemanticEntry("replaced(packingCrate)",
            "Acknowledgement that a full crate has been replaced with an empty one",
            "\"I have placed a new crate\"",
            "交換が完了しました",
            "replaced(packingCrate)",
            10,
            "REPLY"
    );
    exposedReplyList.add(entry);
    entry = new LLMSemanticEntry("didIt(request)",
            "Acknowledgement that the user completed the system's request",
            "\"I did it\" or \"done\"",
            "",
            "didIt(request)",
            10,
            "REPLY"
    );
    exposedReplyList.add(entry);
  }

  private String generateLLMNLGExampleListString() {
    StringBuilder sb = new StringBuilder();
    for (LLMNLGExample entry : nlgExampleList) {
      sb.append(entry.toString(language,false));
    }
    return sb.toString();
  }

  private void generateNLGExampleList() {
    nlgExampleList = new ArrayList<>();
    LLMNLGExample example = new LLMNLGExample("ack(okay)",
            "okay",
            "わかりました",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("askForNow(relevantInformation)",
            "I will ask you for all the relevant information now",
            "それでは、関連するすべての情報を順にお伺いします",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("packedItemDimensions(500,200,100)",
            "I believe the food items currently are 500 millimeters long, 200 millimeters wide, and 100 millimeters tall",
            "惣菜パックのサイズは、長さ500 mm、幅200 mm、高さ100 mmだと認識しています",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("packedItemDimensions(400,-1,200)",
            "I believe the food items currently are 400 millimeters long and and 200 millimeters tall. I have not been told how wide they are.",
            "惣菜パックは、長さ400mm、高さ200mmだと認識していますが、幅はまだ指定されていません",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("packedItemDimensions(-1,-1,-1)",
            "I have not been given any information about how large the food items are",
            "惣菜パックの大きさに関する情報はありません",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("crateDimensions(500,200,100)",
            "I believe the crates currently are 500 millimeters long, 200 millimeters wide, and 100 millimeters tall",
            "番重のサイズは、長さ500mm、幅200mm、高さ100mmだと認識しています",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("crateDimensions(400,-1,200)",
            "I believe the crates currently are 400 millimeters long and and 200 millimeters tall. I have not been shown how wide they are.",
            "番重は、長さ400mm、高さ200mmだと認識していますが、幅はまだ指定されていません",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("crateDimensions(-1,-1,-1)",
            "I have not been given any information about how large the crates are",
            "番重の大きさに関する情報はありません",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("reportPackedItemWidth(100)",
            "I believe the food items are 100 mm wide",
            "惣菜パックの幅は100mmだと認識しています",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("reportCrateHeight(-1)",
            "I have not been given any information about the crates' height",
            "番重の高さに関する情報はありません",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("maxPackableItemsPerCrate(24)",
            "The most items which can fit in a crate is 24",
            "1つの番重には、最大24個の惣菜パックが入ります",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("maxNumberOfPackedLayers(3)",
            "The most layers of items which can fit in a crate is 3",
            "1つの番重には、最大で3層まで積み付け可能です",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("maxNumberOfPackedItemsPerLayer(8)",
            "8 items fit per layer in the crate",
            "番重の各層には、8個の惣菜パックが入ります",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("fullyPackedCrateConfiguration(maxItems(15),maxLayers(3),maxPerLayer(5))",
            "A fully packed crate will hold 15 items total, consisting of 5 items within each of 3 layers",
            "1つの番重には、最大15個の惣菜パックが入ります。これは、各層に5個、高さ方向に3段入る計算です",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("currentPackedItemsPerCrate(10)",
            "I am currently configured to pack 10 items per crate",
            "現在、1つの番重に10個の惣菜パックを積み付ける設定になっています",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("currentPackedLayersPerCrate(numLayers(3),lastLayerFull(true))",
            "I will fully packed 3 layers of items under my current configuration",
            "現在の設定では、惣菜パックを3層にわたって積み付けます",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("currentPackedLayersPerCrate(numLayers(3),lastLayerFull(false))",
            "I will pack 2 layers of items fully and the last one partially under my current configuration",
            "現在の設定では、2層を満杯にし、最上段は一部のみ積み付けます",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("and(not(told(numberOfItemsPerCrate)),assuming(maxPossibleNumber))",
            "I have not explicitly been told the amount of items I should pack into each crate. I will assume the maximum possible amount until told otherwise",
            "積み付け個数が明示されていないため、1つの番重につき最大数で積み付けます",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("currentPackedCrateConfiguration(totalItems(18),maxItemsPerLayer(6))",
            "I am currently configured to pack 18 items per crate, consisting of 6 items within each of 3 layers",
            "現在、1つの番重に18個、ただし、各6個で3層を積み付ける設定になっています",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("currentPackedCrateConfiguration(totalItems(15),maxItemsPerLayer(6))",
            "I am currently configured to pack 15 items per crate, consisting of 2 layers with 6 items each and a last layer holding 3 items",
            "現在、1つの番重に15個、ただし、下2層に各6個、最上段に3個を積み付ける設定になっています",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("currentPackedCrateConfiguration(totalItems(18),maxItemsPerLayer(4))",
            "I am currently configured to pack 18 items per crate, consisting of 4 layers with 4 items each and a last layer holding 2 items",
            "現在、1つの番重に18個、ただし、下4層に各4個、最上段に2個）を積み付ける設定になっています",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("cannot(reportMaxNumberOfPackedItems(robotone),because(not(ranPackingAlgorithmWithUpdatedSettings)))",
            "I cannot tell you the maximum number of items I can pack into a crate because I have not yet run the packing algorithm with up-to-date settings.",
            "まだ配置計算を実行していないため、最大個数をお伝えできません",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("cannot(askForNumberOfItemsToPack(robotone),because(cannot(packNegativeAmountOfObjects))",
            "I could not set the number of items to pack to what you told me because I don't know how to pack a negative amount of objects.",
            "負の個数を積み付けることはできないため、指定された値を設定できません",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("cannot(fitAnyItemsInto(crate),given(currentKnownItemAndCrateDimensions))",
            "I cannot fit any items into the crate given my current understanding of their respective dimensions",
            "現在の寸法設定では、番重に惣菜パックを積み付けることができません",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("packing(newCrate)",
            "Beginning to pack new crate",
            "わかりました。番重への積み付けを開始します",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("therefore(and(cannot(fitNumberOfItemsInCrate(10)),maxPackableItemsPerCrate(8)),proceedWith(maxAmount))",
            "I cannot fit 10 into a single crate, the most I can do is 8. I will proceed with that maximum amount.",
            "1つの番重に10個は入りません。最大の8個で積み付けを行います",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("starting(packingProcedure)",
            "Starting packing procedure now",
            "これより積み付け作業を開始します",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("cannot(packItemsIntoCrates(robotone),because(missing(configurationParameters))",
            "I could not successfully pack items because I have unconfigured task parameters",
            "積み付けパラメータの設定は必須です",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("cannot(packItemsIntoCrates(robotone),because(not(availableResources(robotone)))",
            "I can not follow your command to begin packing because I am already busy doing something",
            "現在、別の作業中のため積み付けを開始できません",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("cannot(configurePackingParameters(robotone),because(not(availableResources(robotone)))",
            "Packing parameters cannot be reconfigured while actively pursuing another task. Please cancel your current task before trying again",
            "作業の実行中は積み付けパラメータを変更できません。現在の作業を中止してから再試行してください",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("cannot(setPackedItemDimensions(robotone),because(not(availableResources(robotone)))",
            "Dimensions of the packed items cannot be changed while actively pursuing another task. Please cancel your current task before trying again",
            "作業の実行中は、惣菜パックの寸法を変更できません。現在の作業を中止してから再試行してください",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("cannot(setCrateDimensions(robotone),because(not(availableResources(robotone)))",
            "Dimensions of the crate cannot be changed while actively pursuing another task. Please cancel your current task before trying again",
            "作業の実行中は、番重の寸法を変更できません。現在の作業を中止してから再試行してください",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("not(have(robotone,pendingQuestions))",
            "I do not currently have any pending questions",
            "",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("then(applyingParameterUpdate(numItemsToPack),packingPerCrate(10))",
            "Putting update to number of items to pack into effect. I will now pack 10 items into each crate.",
            "積み付け個数を更新します。これ以降、各番重に10個を積み付けます",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("report(needMomentToCompute(optimalPackingArrangement))",
            "Please give me a moment to compute the optimal packing arrangement",
            "最適な配置を計算しています",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("fullyUpdated(packingParameters)",
            "packing parameters have been fully updated",
            "積み付けパラメータの更新が完了しました",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("and(report(done),ack(thankYou))",
            "Done! Thank you",
            "配置計算が完了しました",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("report(changeWillTakeEffectForTheNextPackedCrate)",
            "This change will take effect for the next packed crate",
            "この変更は次の番重から適用されます",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("and(couldNot(run(?actor,packingAlgorithm)),because(not(fullyDefined,and(crateDimensions,itemDimensions))))",
            "I could not run the packing algorithm because the crate and item dimensions have not been fully defined",
            "番重および惣菜パックの寸法がすべて定義されていないため、配置計算を実行できません",
            "REPLY");
    nlgExampleList.add(example);
    example = new LLMNLGExample("then(report(prerequisite(packing,configure(unsetParameters))),ask(may(robotone,askFor(them))))",
            "Before commencing, there are unset packing parameters which must be configured. May I ask for them now?",
            "未設定の積み付けパラメータがあります。設定しますか？",
            "QUESTION");
    nlgExampleList.add(example);
    example = new LLMNLGExample("then(report(configuredAmountToPackPerCrate(robotone,8)),ask(want(robotone,changeAmount)))",
            "I am currently configured to pack 8 items into each crate. Do you want to change that amount?",
            "現在、1つの番重に8個を積み付ける設定になっています。この数を変更しますか？",
            "QUESTION");
    nlgExampleList.add(example);
    example = new LLMNLGExample("length(crate)",
            "How long is the crate?",
            "番重の長さはいくつですか？",
            "QUESTION");
    nlgExampleList.add(example);
    example = new LLMNLGExample("width(crate)",
            "How wide is the crate?",
            "番重の幅はいくつですか？",
            "QUESTION");
    nlgExampleList.add(example);
    example = new LLMNLGExample("height(crate)",
            "How tall is the crate?",
            "番重の高さはいくつですか？",
            "QUESTION");
    example = new LLMNLGExample("length(packedItem)",
            "How long is the item being packed?",
            "惣菜パックの長さはいくつですか？",
            "QUESTION");
    nlgExampleList.add(example);
    example = new LLMNLGExample("width(packedItem)",
            "How wide is the item being packed?",
            "惣菜パックの幅はいくつですか？",
            "QUESTION");
    nlgExampleList.add(example);
    example = new LLMNLGExample("height(packedItem)",
            "How tall is the item being packed?",
            "惣菜パックの高さはいくつですか？",
            "QUESTION");
    nlgExampleList.add(example);
    example = new LLMNLGExample("fill(robotone,eachCrate,completely)",
            "Do you want me to fill each crate completely?",
            "それぞれの番重は、完全に満杯にしていいですか？",
            "QUESTION");
    nlgExampleList.add(example);
    example = new LLMNLGExample("number(items,pack(robotone))",
            "How many items would you like me to pack?",
            "それぞれの番重には、いくつ積み付けますか？",
            "QUESTION");
    nlgExampleList.add(example);
    example = new LLMNLGExample("then(report(complete(pack(robotone,crate))),ask(ackWhen(replaced(packedCrate,emptyCrate))))",
            "I have finished packing this crate, please let me know when it has been replaced with an empty one",
            "番重への積み付けを完了しました。からの盤寿への交換が完了したらお知らせください",
            "QUESTION");
    nlgExampleList.add(example);
    example = new LLMNLGExample("then(report(updatedSince(relevantPackingParameters,calculated(optimalCrateArrangement))),ask(may(robotone,rerunAlgorithm),desire(upToDate)))",
            "Relevant packing parameters have been updated since the last time an optimal crate arrangement has been calculated, may I rerun the algorithm to be up-to-date?",
            "前回から、関連する積み付けパラメータが更新されています。最適な配置を再計算してもよろしいですか？",
            "QUESTION");
    nlgExampleList.add(example);
  }

  private String generateLLMValidationExampleListString() {
    StringBuilder sb = new StringBuilder();
    for (LLMNLGExample entry : validationExampleList) {
      sb.append(entry.toString(language,true));
    }
    return sb.toString();
  }

  private void generateValidationExampleList() {
    validationExampleList = new ArrayList<>();
    LLMNLGExample example = new LLMNLGExample("packItemsIntoCrates(robotone)",
            "Did you mean to tell me to begin packing?",
            "積み付けを開始するよう指示されましたか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("configurePackingParameters(robotone)",
            "Did you mean to tell me that you want to begin updating all parameters of the packing task?",
            "積み付け作業のすべてのパラメータを更新したいという意味ですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportPackedItemLength(robotone)",
            "Are you asking me what I think the length of the items to pack are?",
            "積み付け対象の惣菜パックの長さについて、システムが認識している値をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportPackedItemWidth(robotone)",
            "Are you asking me what I think the width of the items to pack are?",
            "積み付け対象の惣菜パックの幅について、システムが認識している値をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportPackedItemHeight(robotone)",
            "Are you asking me what I think the height of the items to pack are?",
            "積み付け対象の惣菜パックの高さについて、システムが認識している値をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportPackedItemDimensions(robotone)",
            "Are you asking me what I think the dimensions of the items to pack are?",
            "システムが把握している惣菜パックのサイズをお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportCrateLength(robotone)",
            "Are you asking me what I think the length of the crates I am packing into are?",
            "積み付け先の番重の長さについて、システムが認識している値をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportCrateWidth(robotone)",
            "Are you asking me what I think the width of the crates I am packing into are?",
            "積み付け先の番重の幅について、システムが認識している値をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportCrateHeight(robotone)",
            "Are you asking me what I think the height of the crates I am packing into are?",
            "積み付け先の番重の高さについて、システムが認識している値をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportCrateDimensions(robotone)",
            "Are you asking me what I think the dimensions of the crates I am packing into are?",
            "システムが把握している番重のサイズをお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportConfiguredNumberOfItemsToPack(robotone)",
            "Are you asking me how many items I am configured to pack into each crate?",
            "1つの番重への積み付けるよう設定されてい個数をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportCurrentNumberOfLayersToPack(robotone)",
            "Are you asking me how many layers will be present in a packed crate given my current configuration?",
            "現在の設定で、番重の中にいくつの層ができるかをお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportCurrentPackedCrateConfiguration(robotone)",
            "Are you asking me what a packed crate will look like under my current configuration?",
            "現在の設定で積み付けた番重がどのようになるかをお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportMaxNumberOfPackedItems(robotone)",
            "Are you asking me what the maximum number of items I could pack into a crate is?",
            "1つの番重に積み付け可能な最大の惣菜パック数をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportMaxNumberOfPackedLayers(robotone)",
            "Are you asking me what the maximum number of layers of items I can pack into a crate is?",
            "番重に積み付け可能な最大の層数をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportMaxNumberOfPackedItemsPerLayer(robotone)",
            "Are you asking me how many items will be packed into a full crate layer?",
            "番重の各層に、いくつの惣菜パックが入るかお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportFullyPackedCrateConfiguration(robotone)",
            "Are you asking me what a fully packed crate would look like?",
            "完全に積み付けられた番重の見た目をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("reportAllPackingParameters(robotone)",
            "Are you asking me to report every configured parameter of the packing task?",
            "積み付け作業に関する全ての設定パラメータを知りたいということですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("setNumberOfItemsToPackPerCrate(robotone,10)",
            "Are you telling me to fit 10 items per crate when packing?",
            "1つの番重に10個の惣菜パックを積み付けるよう指示されていますか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("setPackedItemDimensions(robotone,500,400,80)",
            "Are you telling me the items being packed are 500 millimeters long, 400 millimeters wide, and 80 millimeters tall?",
            "惣菜パックのサイズが、長さ500mm、幅400mm、高さ80mmであると指定されていますか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("setCrateDimensions(robotone,1000,500,200)",
            "Are you telling me the crates being packed into are a meter long, half a meter wide, and 200 millimeters tall?",
            "積み付け先の番重が、長さ1m、幅0.5m、高さ200mmであると指定されていますか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("describeCurrentGoal(robotone)",
            "Are you asking me what my current task is?",
            "現在システムが実行している作業内容をお尋ねですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("cancelSystemGoals(robotone)",
            "Are you asking me to cancel and discard my current task?",
            "現在の作業をキャンセルして破棄するよう指示されていますか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("suspendSystemGoalsBlocking(robotone)",
            "Are you asking me to pause my current task?",
            "現在の作業を一時停止するよう指示されていますか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("resumeSystemGoals(robotone)",
            "Are you asking me to resume my current task?",
            "一時停止した作業を再開するよう指示されていますか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("val(yes)",
            "Did you mean to answer my question by saying yes?",
            "私の質問に「はい」と答えたという理解でよろしいですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("val(no)",
            "Did you mean to answer my question by saying no?",
            "私の質問に「いいえ」と答えたという理解でよろしいですか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("sizeInMM(150)",
            "Did you mean to answer my question by indicating a size of 150 millimeters?",
            "サイズが150mmという意味でお答えいただきましたか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("amount(20)",
            "Did you mean to answer by question by indicating an amount of 20?",
            "数量が20という意味でお答えいただきましたか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("abort(question)",
            "Did you mean to tell me to forget about my last question?",
            "先ほどの質問を忘れるよう指示されていますか？",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("repeatLastQuestion(robotone)",
            "Did you mean to ask me to repeat my last question?",
            "",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("goToPoseDirect(robotone,%examplePose:pose)",
            "Did you mean to ask me to go to the %examplePose pose?",
            "%examplePose番にしてください、という指示でしょうか",
            "QUESTION");
    validationExampleList.add(example);
    example = new LLMNLGExample("didIt(request)",
            "Did you mean to say that you completed the request?",
            "",
            "QUESTION");
    validationExampleList.add(example);
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