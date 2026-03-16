/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.serviceproviders.tr;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.*;

public class PickAndPlace2024LLMServiceProvider extends TRLLMServiceProvider {
    private boolean v2 = true; //v1 no longer supported with class abstraction refactor
    private String learnedActionsString = "";
    private boolean usingWithParserTest = false;

    @Override
    protected void init() {
        super.init();
        processMatchingNLUParseLiteralsAsRefs = true;
    }

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = super.additionalUsageInfo();
        options.add(Option.builder("japanese").desc("use japanese versions of prompts").build());
        return options;
    }

    @Override
    protected void parseArgs (CommandLine cmdLine) {
        super.parseArgs(cmdLine);
        if (cmdLine.hasOption("japanese")) {
            language = "Japanese";
        }
    }

    @Override
    protected List<String> getRefTypes() {
        return List.of("pose", "physobj");
    }

    @Override
    protected Prompt getNluSystemPrompt() {
        return Prompts.getPrompt("pickandplace2024/nlu/systemv3");
    }

    @Override
    protected Prompt getNluUserPrompt() {
        if (language.equals("English")) {
            return Prompts.getPrompt("pickandplace2024/nlu/english/userv3");
        } else {
            return Prompts.getPrompt("pickandplace2024/nlu/japanese/userv3");
        }
    }

    @Override
    protected Prompt getNlgSystemPrompt() {
        return Prompts.getPrompt("pickandplace2024/nlg/system");
    }

    @Override
    protected Prompt getNlgUserPrompt() {
        if (language.equals("English")) {
            return Prompts.getPrompt("pickandplace2024/nlg/english/user");
        } else {
            return Prompts.getPrompt("pickandplace2024/nlg/japanese/user");
        }
    }

    @Override
    protected Prompt getValidationSystemPrompt() {
        return getNlgSystemPrompt();
    }
    @Override
    protected Prompt getValidationUserPrompt() {
        if (language.equals("English")) {
            return Prompts.getPrompt("pickandplace2024/nlg/english/validation");
        } else {
            return Prompts.getPrompt("pickandplace2024/nlg/japanese/validation");
        }
    }

    @Override
    protected Map<String,String> populatePromptVars(Utterance utterance, Map<String,String> promptVars, boolean system, boolean parsing, boolean validation) {
        if (parsing) {
            if (system) {
                List<String> objectsEnglish = getObjects(utterance.getAddressee().getName(),true);
                List<String> objectsJapanese = getObjects(utterance.getAddressee().getName(),false);
                List<String> englishPoses = getPoses(utterance.getAddressee().getName(),true);
                List<String> japanesePoses = getPoses(utterance.getAddressee().getName(),false);
                promptVars.put("%objectList", String.join("\n", objectsEnglish));
                promptVars.put("%locationList", String.join("\n", englishPoses));
                promptVars.put("%japaneseObjList", String.join("\n", objectsJapanese));
                promptVars.put("%japanesePoseList", String.join("\n", japanesePoses));
                promptVars.put("%worldState", getWorldStateString(utterance.getAddressee()));
                promptVars.put("%learnedActions%",learnedActionsString);
                populatePromptVarsWithProperties(promptVars, objectsEnglish, objectsJapanese, englishPoses, japanesePoses);
            } else {
                promptVars.put("%input", utterance.getWordsAsString());
            }
        } else {
            List<String> objectsEnglish = getObjects(utterance.getSpeaker().getName(),true);
            List<String> objectsJapanese = getObjects(utterance.getSpeaker().getName(),false);
            List<String> englishPoses = getPoses(utterance.getSpeaker().getName(),true);
            List<String> japanesePoses = getPoses(utterance.getSpeaker().getName(),false);
            promptVars.put("%utterance", utterance.toString());
            populatePromptVarsWithProperties(promptVars, objectsEnglish, objectsJapanese, englishPoses, japanesePoses);

            promptVars = new LinkedHashMap<>();
            promptVars.put("%language", language);
            promptVars.put("%objectList", String.join("\n", objectsEnglish));
            promptVars.put("%locationList", String.join("\n", englishPoses));
            promptVars.put("%japaneseObjList", String.join("\n", objectsJapanese));
            promptVars.put("%japanesePoseList", String.join("\n", japanesePoses));
        }

        return promptVars;
    }

    //TODO: replace failure cases returning null with same failure semantics
    @Override
    protected Utterance correctCommonNLUMistakes(Utterance utterance) {
        String actorName = utterance.getAddressee().getName();
        try {
            String command = utterance.getSemantics().getName();
            if (command.equals("believeObjectAt")) {
                utterance.setType(UtteranceType.INSTRUCT);
                if (v2) {
                    List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
                    args.add(0, Factory.createSymbol(actorName));
                    utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
                }
            } else if (command.startsWith("describe")) {
                utterance.setType(UtteranceType.INSTRUCT);
                if (v2) {
                    List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
                    args.add(0, Factory.createSymbol(actorName));
                    utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
                }
            } else if (command.equals("goTo")) {
                List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
                utterance.setSemantics(Factory.createPredicate("goToPose", args));
            } else if (v2 && command.equals("knowHowTo")) {
                List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
                args.add(0, Factory.createSymbol(actorName));
                Term knownPredicate = (Term) args.remove(1);
                List<Symbol> innerArgs = knownPredicate.getArgs();
                innerArgs.add(0,Factory.createSymbol(actorName));
                args.add(Factory.createPredicate(knownPredicate.getName(), innerArgs));
                utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
            //} else if (v2 && command.equals("see")) {
            //    List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
            //    args.add(0, Factory.createSymbol("robotone"));
            //    utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
            } else if (command.equals("pose")) {
                Symbol arg = ((Term)utterance.getSemantics()).get(0);
                if (!arg.isVariable() && !arg.getName().equals("none") && !(arg.isVariable() && !arg.getType().equals("pose"))) {
                    log.warn("[correctCommonNLUMistakes] LLM returned pose(?pose) parse where ?pose is not a reference. Not allowing");
                    return null;
                }
            } else if (command.equals("objectType")) {
                Symbol arg = ((Term)utterance.getSemantics()).get(0);
                if (!arg.isVariable() && !arg.getName().equals("none")&& !(arg.isVariable() && !arg.getType().equals("physobj"))) {
                    log.warn("[correctCommonNLUMistakes] LLM returned objectType(?object) parse where ?object is not a reference. Not allowing");
                    return null;
                }
            } else if (command.equals("whichObjectsAtCurrentPose")) {
                List<Symbol> args = ((Predicate) (utterance.getSemantics())).getArgs();
                args.add(0, Factory.createSymbol(actorName));
                utterance.setSemantics(Factory.createPredicate(utterance.getSemantics().getName(), args));
            }
        } catch (Exception e) {
            log.warn("[correctCommonNLUMistakes] unexpected semantics format");
            return null;
        }
        return utterance;
    }

    private Map<String,String> populatePromptVarsWithProperties(Map<String,String> promptVars, List<String> objectsEnglish, List<String> objectsJapanese, List<String> posesEnglish, List<String> posesJapanese) {
        int numObjects = objectsEnglish.size();
        int numObjectsJapanese = objectsJapanese.size();
        if (numObjects != 5 || numObjectsJapanese != 5) {
            log.warn("[populatePromptVarsWithProperties] number of objects received is not 5");
        }

        if (numObjects > 0 && numObjectsJapanese > 0) {
            promptVars.put("the red block", objectsEnglish.get(0));
            promptVars.put("red block", objectsEnglish.get(0));
            promptVars.put("赤いブロック", objectsJapanese.get(0));
            if (numObjects > 1 && numObjectsJapanese > 1) {
                promptVars.put("the green block",objectsEnglish.get(1));
                promptVars.put("green block", objectsEnglish.get(1));
                promptVars.put("緑色のブロック", objectsJapanese.get(1));
                if (numObjects > 2 && numObjectsJapanese > 2) {
                    promptVars.put("the yellow block", objectsEnglish.get(2));
                    promptVars.put("yellow block", objectsEnglish.get(2));
                    promptVars.put("黄色いブロック", objectsJapanese.get(2));
                    if (numObjects > 3 && numObjectsJapanese > 3) {
                        promptVars.put("the blue block", objectsEnglish.get(3));
                        promptVars.put("blue block", objectsEnglish.get(3));
                        promptVars.put("青いブロック", objectsJapanese.get(3));
                        if (numObjects > 4 && numObjectsJapanese > 4) {
                            promptVars.put("the orange block", objectsEnglish.get(4));
                            promptVars.put("orange block", objectsEnglish.get(4));
                            promptVars.put("オレンジ色のブロック", objectsJapanese.get(4));
                        }
                    }
                }
            }
        }
        int numPosesEnglish = posesEnglish.size();
        int numPosesJapanese = posesJapanese.size();
        if (numPosesEnglish > 0 && numPosesJapanese > 0) {
                promptVars.put("location one", posesEnglish.get(0));
                promptVars.put("位置1", posesJapanese.get(0));
            if (numPosesEnglish > 1 && numPosesJapanese > 1) {
                    promptVars.put("location two", posesEnglish.get(1));
                    promptVars.put("位置2", posesJapanese.get(1));
                if (numPosesEnglish > 2 && numPosesJapanese > 2) {
                        promptVars.put("location three", posesEnglish.get(2));
                        promptVars.put("位置3", posesJapanese.get(2));
                    if (numPosesEnglish > 3 && numPosesJapanese > 3) {
                            promptVars.put("location four", posesEnglish.get(3));
                            promptVars.put("位置4", posesJapanese.get(3));
                    }
                }
            }
        }
        return promptVars;
    }
    
    private List<String> getObjects(String actorName, boolean english) {
        try {
            List<String> propertyNames = new ArrayList<>();
            if (english) {
                //TODO: should be based on group
                TRADEServiceInfo tsi = getServiceForAgent(actorName, "getEnglishObjectDescriptors");
                if (tsi != null) {
                    propertyNames = tsi.call(List.class);
                }
                else if (usingWithParserTest) {
                    return Arrays.asList("red block", "orange block", "yellow block", "blue block", "green block");
                }
            } else {
                //TODO: should be based on group
                TRADEServiceInfo tsi = getServiceForAgent(actorName, "getJapaneseObjectDescriptors");
                if (tsi != null) {
                    propertyNames = tsi.call(List.class);
                }
                else if (usingWithParserTest){
                    return Arrays.asList("赤いブロック", "オレンジ色のブロック", "黄色いブロック", "青いブロック", "緑色のブロック");
                }
            }
            return propertyNames;
        } catch (TRADEException e) {
            log.error("[getObjectListString]", e);
        }

        return new ArrayList<>();
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

    private String getWorldStateString(Symbol agent) {
        if (usingWithParserTest) {
            return "Nothing is known by the robot";
        } else {
            StringBuilder builder = new StringBuilder();
            Predicate query = Factory.createPredicate("at", "X", "Y");
            List<Map<Variable, Symbol>> bindings;
            String robotLocationString = "";
            String holdingString = "";
            try {
                bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class, query);
                for (Map<Variable, Symbol> binding : bindings) {
                    Symbol x = binding.get(new Variable("X"));
                    Symbol y = binding.get(new Variable("Y"));
                    TRADEServiceInfo tsi = getServiceForAgent(agent.getName(), "getDescriptorForID", Symbol.class);
                    if (tsi != null) {
                        Symbol object = tsi.call(Symbol.class, x);
                        tsi = getServiceForAgent(agent.getName(), "getDescriptorForPoseID", Symbol.class);
                        if (tsi != null) {
                            Symbol pose = tsi.call(Symbol.class, y);
                            if (x.getName().startsWith(agent.getName())) {
                                if (object.getName().equals("none")) {
                                    robotLocationString = String.format("The robot is at %s. ", pose.getName());
                                } else {
                                    builder.append(String.format("There is a %s at %s. ", object.getName(), pose.getName()));
                                }
                            }
                        }
                    }
                }

                query = Factory.createPredicate("holding", "X", "Y");
                bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class, query);
                for (Map<Variable, Symbol> binding : bindings) {
                    Symbol x = binding.get(new Variable("X"));
                    Symbol y = binding.get(new Variable("Y"));
                    if (x.getName().startsWith(agent.getName())) {
                        TRADEServiceInfo tsi = getServiceForAgent(agent.getName(), "getDescriptorForID", Symbol.class);
                        if (tsi != null) {
                            Symbol object = tsi.call(Symbol.class, y);
                            holdingString = String.format("The robot is holding a %s. ", object);
                        }
                    }
                }
            } catch (TRADEException e) {
                log.error("[getWorldState] ", e);
                return "Nothing is known by the robot";
            }
            if (robotLocationString.isEmpty() && holdingString.isEmpty()) {
                log.warn("[getWorldState] no matching beliefs found for agent {}", agent);
                return "Nothing is known by the robot";
            }
            return robotLocationString + holdingString + builder;
        }
    }

    @Override
    protected String getDescriptorType(String agentName, String descriptor) {
        List<String> propertyNames = new ArrayList<>();
        propertyNames  = getObjects(agentName, true);
        if (propertyNames.contains(descriptor)) {
            return "physobj";
        }

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
        List<String> englishDescriptors = getObjects("robotone",true);
        List<String> japaneseDescriptors = getObjects("robotone",false);
        int index = englishDescriptors.indexOf(descriptorString);
        if (index != -1 && index < japaneseDescriptors.size()) {
            return japaneseDescriptors.get(index);
        }
        englishDescriptors = getPoses("robotone",true);
        japaneseDescriptors = getPoses("robotone",false);
        index = englishDescriptors.indexOf(descriptorString);
        if (index != -1 && index < japaneseDescriptors.size()) {
            return japaneseDescriptors.get(index);
        }
        return descriptorString;
    }

    @Override
    public void handleDatabaseAddition(ActionDBEntry entry) throws TRADEException {
        if (entry.isScript() && entry.getDBFile() == null && !entry.getName().equals("planned")) {
            log.info("[handleDatabaseAddition] have learned action {}", entry.getName());
            learnedActionsString += String.format("%s\nDescription: The user has previously manually taught the definition of this action through the use of the learnNewTask and endActionLearning actions. It will be referred to as \"%s\" when instructed by the user.\n#\n",entry.getSignature(false),entry.getName());
        }
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
