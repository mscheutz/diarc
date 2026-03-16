/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.serviceproviders.tr;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.slug.common.Utterance;

import java.util.*;

public class AvaLLMServiceProvider extends TRLLMServiceProvider {
    //TODO: All of this information should potentially be spread out to different parts of the system and gathered here
    //  on startup rather than hardcoded. E.g. descriptions from ActionDBEntries (asl pre-existing and new java annotations),
    //  NL examples from various parsers, selection of exposed actions from config, ... This is a fairly large architectural
    //  decision to discuss and therefore will need to wait until after first deployment.
    private class LLMActionEntry {
        String signature;
        String description;
        List<LLMNLExample> examples;

        LLMActionEntry(String signature, String description, LLMNLExample example) {
            this.signature = signature;
            this.description = description;
            this.examples = List.of(example);
        }

        LLMActionEntry(String signature, String description, String exampleInstruction, String exampleTranslation, int exampleConfidence) {
            this.signature = signature;
            this.description = description;
            this.examples = List.of(new LLMNLExample(exampleInstruction, exampleTranslation, exampleConfidence));
        }

      LLMActionEntry(String signature, String description, List<String> exampleInstructions, List<String> exampleTranslations, int exampleConfidence) {
        this.signature = signature;
        this.description = description;
        this.examples = new ArrayList<>();
        if (exampleInstructions.size() != exampleTranslations.size()) {
          log.error("[LLMActionEntry] provided instruction and translation lists differ in size");
        } else {
          for (int i = 0; i < exampleInstructions.size(); i++) {
            this.examples.add(new LLMNLExample(exampleInstructions.get(i), exampleTranslations.get(i), exampleConfidence));
          }
        }
      }

        public String toString(boolean nlu) {
            String signatureString = signature;
            if (!nlu) {
                String[] parts = signatureString.split("\\(",2);
                signatureString = parts[0] + "(ava";
                if (parts[1].equals(")")) {
                    signatureString += parts[1];
                } else {
                    signatureString += "," + parts[1];
                }
            }
            String output = signatureString + "\n" +
                  "Description: " + description + "\n";
            if (nlu) {
                for (LLMNLExample example : examples) {
                    output +=
                            "Example Translation: \"" +
                                    example.instruction +
                                    "\" -> {\"Semantics\":\"" +
                                    example.semantics +
                                    "\",\"Input Type\":\"INSTRUCT\",\"Confidence\":\"" +
                                    example.confidence +
                                    "\"}\n";
                  }
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
        processMatchingNLUParseLiteralsAsRefs = true;
    }

  @Override
  protected Prompt getNluSystemPrompt() {
    return Prompts.getPrompt("ava/nlu/system");
  }

  @Override
  protected Prompt getNluUserPrompt() {
    return Prompts.getPrompt("ava/nlu/user");
  }

  @Override
  protected Prompt getNlgSystemPrompt() {
    return Prompts.getPrompt("ava/nlg/system");
  }

  @Override
  protected Prompt getNlgUserPrompt() {
    return Prompts.getPrompt("ava/nlg/user");
  }

  @Override
  protected Prompt getValidationSystemPrompt() {
    return Prompts.getPrompt("ava/validation/system");
  }

  @Override
  protected Prompt getValidationUserPrompt() {
    return Prompts.getPrompt("ava/validation/user");
  }

  //TODO: Certain values here should be maintained and updated over time rather than regenerated
  @Override
  protected Map<String, String> populatePromptVars(Utterance utterance, Map<String, String> promptVars, boolean system, boolean parsing, boolean validation) {
    if (parsing) {
      String questionString = getCurrentQuestionsString(utterance.getAddressee());
      if (system) {
        List<String> locations = getLocations("self", true);
        List<String> patrolPaths = getPatrolPaths();
        List<String> videos = getVideoNames();
        List<String> images = getImageNames();
        String patrolPathList = String.join(":path\n", patrolPaths);
        if (!patrolPathList.isEmpty()) {
          patrolPathList += ":path\n";
        }
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
        promptVars.put("%actionList", generateLLMActionListString(true));
        promptVars.put("%patrolPathList", patrolPathList);
        promptVars.put("%locationList", locationList);
        promptVars.put("%videoList", videoList);
        promptVars.put("%exampleVideo", videos.isEmpty() ? "test" : videos.get(0));
        promptVars.put("%exampleImage", images.isEmpty() ? "test" : images.get(0));
        promptVars.put("%imageList", imageList);
        promptVars.put("%worldState", getWorldStateString(utterance.getAddressee()));
        promptVars.put("%goals", getCurrentGoalsSummary(utterance.getAddressee()));
        promptVars.put("%questionList", questionString);
        promptVars.put("%learnedActions%",learnedActionsString);
        promptVars.put("%exampleLocation", locations.isEmpty() ? "test location" : locations.get(0));
        promptVars.put("%examplePath", patrolPaths.isEmpty() ? "test path" : patrolPaths.get(0));
      } else {
        promptVars.put("%utterance", utterance.getWordsAsString());
        //TODO: Quick pre testing-deployment hack - do this through contextual parallel prompting or dialogue recovery instead
        if (questionString.startsWith("What is the name of the task") || questionString.startsWith("What would you like the action to go on this upcoming tour sequence to be called?")) {
          promptVars.put("%preface","Reminder, the current question from the system is as follows:\n" + questionString + "\nIf the input does not make sense as any other sort of instruction, it is likely directly a form the desired taskName.\n");
        } else {
          promptVars.put("%preface", "");
        }
      }
    } else {
      if (system) {
        List<String> locations = getLocations("self", true);
        List<String> patrolPaths = getPatrolPaths();
        promptVars.put("%language", language);
        promptVars.put("%actionList", generateLLMActionListString(false));
        promptVars.put("%learnedActions%",learnedActionsString);
        promptVars.put("%patrolPathList", String.join("\n", patrolPaths));
        promptVars.put("%locationList", String.join("\n", locations));
        promptVars.put("%addressee", utterance.getAddressee().getName());
      } else {
        promptVars.put("%utterance", utterance.toString());
      }
    }
    return promptVars;
  }

  @Override
  protected List<String> getRefTypes() {
    return List.of("location");
  }

    @Override
    protected Utterance correctCommonNLUMistakes(Utterance utterance) {
        ////TODO: this needs to be caught earlier in the pipeline than here, just copying here to maintain awareness of
        //      the issue without hacking code in the asr app or parser itself.
        //incoming.setWords(incoming.getWords().stream().map(word -> {
        //  word = word.toLowerCase();
        //  if (word.equals("wi-fi")) {
        //    word = "wifi";
        //  }
        //  return word;
        //}).collect(Collectors.toList()));

        Symbol actor = utterance.getAddressee();
        try {
            String command = utterance.getSemantics().getName();
            if (command.equals("location")) {
                Symbol arg = ((Term)utterance.getSemantics()).get(0);
                if (!arg.isVariable() && !arg.getName().equals("none") && !(arg.isVariable() && !arg.getType().equals("location"))) {
                    log.warn("[correctCommonNLUMistakes] LLM returned location(?location) parse where ?location is not a reference. Not allowing");
                    return null;
                }
            } else if (command.equals("taskPredicate")) {
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

                // Needs to have all arguments defined (actor, video name, location)
                List<Symbol> args = semantics.getArgs();
                if (args.size() != 3) {
                  log.error("[correctCommonNLUMistakes] playVideo returned with incorrect number of arguments");
                  //TODO: check if actually supplied a duration
                  if (args.size() == 4) {
                    Utterance.Builder failure = new Utterance.Builder();
                    failure.setSpeaker(utterance.getSpeaker());
                    failure.setAddressee(utterance.getAddressee());
                    failure.setSemantics(Factory.createPredicate("error(" + utterance.getAddressee() + ",cannot(playVideosForSpecificDurations))"));
                    failure.setWords(utterance.getWords());
                    return failure.build();
                  } else {
                  return null;
                  }
                }

                // Check which arguments were set to "default" vs populated explicitly
                //TODO: set up as listener and hold notion of these rather than grabbing each call
                List<String> videoNames = getVideoNames();
                List<String> imageNames = getImageNames();
                boolean videoDefault = false;
                boolean locationDefault = false;
                if (args.get(2).toString().equals("default")) {
                  locationDefault = true;
                }
                if (args.get(1).toString().equals("default")) {
                  videoDefault = true;
                } else {
                  //Make sure the supplied video is actually a video and not hallucinated by the LLM or an image
                  String videoName = getMatchingNameIgnoreCase(args.get(1).getName(), videoNames);
                  if (!args.get(1).getType().equals("video") || videoName.isEmpty()) {
                    // If the supplied video is a known image, attempt to coerce into corresponding display image action
                    String imageName = getMatchingNameIgnoreCase(args.get(1).getName(), imageNames);
                    if (!imageName.isEmpty()) {
                      if (locationDefault) {
                        log.warn("[correctCommonNLUMistakes] replacing playVideo with corresponding displayImage action");
                        utterance.setSemantics(Factory.createPredicate("displayNamedImage", actor, args.get(1).toUntyped()));
                        return utterance;
                      } else {
                        log.warn("[correctCommonNLUMistakes] replacing playVideo with corresponding displayImage action");
                        utterance.setSemantics(Factory.createPredicate("displayNamedImageAtLocation", actor, args.get(1).toUntyped(), args.get(2)));
                        return utterance;
                      }
                    }
                    log.error("[correctCommonNLUMistakes] playVideos' video argument is not a properly typed saved video");
                    Utterance.Builder failure = new Utterance.Builder();
                    failure.setSpeaker(utterance.getSpeaker());
                    failure.setAddressee(utterance.getAddressee());
                    failure.setSemantics(Factory.createPredicate("error("+utterance.getAddressee()+",not(savedVideo("+args.get(1).toUntypedString()+")))"));
                    failure.setWords(utterance.getWords());
                    return failure.build();
                  }
                }

                // Arguments are valid, coerce into the appropriate action
                if (videoDefault && locationDefault) {
                  utterance.setSemantics(Factory.createPredicate("playNextVideo", actor));
                } else if (videoDefault) {
                  utterance.setSemantics(Factory.createPredicate("playNextVideoAtLocation", actor, args.get(2)));
                } else if (locationDefault) {
                  utterance.setSemantics(Factory.createPredicate("playSpecificVideo", actor, args.get(1).toUntyped()));
                } else {
                  utterance.setSemantics(Factory.createPredicate("playSpecificVideoAtLocation", actor, args.get(1).toUntyped(), args.get(2)));
                }
            } else if (command.equals("displayImage")) {
                // Special format returned by LLM for different parameterization of displaying images - make sure output is
                //    valid and coerce into proper specific action semantic form.
                Predicate semantics = (Predicate) utterance.getSemantics();

                // Needs to have all arguments defined (actor, video name, location, duration)
                List<Symbol> args = semantics.getArgs();
                if (args.size() != 4) {
                  log.error("[correctCommonNLUMistakes] displayImage returned with incorrect number of arguments");
                  return null;
                }

                // Check which arguments were set to "default" vs populated explicitly
                //TODO: set up as listener and hold notion of these rather than grabbing each call
                List<String> videoNames = getVideoNames();
                List<String> imageNames = getImageNames();
                boolean durationDefault = false;
                boolean locationDefault = false;

                if (args.get(2).toString().equals("default")) {
                  locationDefault = true;
                }
                if (args.get(3).toString().equals("default")) {
                  durationDefault = true;
                }
                //Check if image gets "default"
                if (args.get(1).toString().equals("default")) {
                  log.error("[correctCommonNLUMistakes] displayImages's image argument set to default - not possible");
                  Utterance.Builder failure = new Utterance.Builder();
                  failure.setSpeaker(utterance.getSpeaker());
                  failure.setAddressee(utterance.getAddressee());
                  failure.setSemantics(Factory.createPredicate("error("+utterance.getAddressee()+",not(supplied(imageName)))"));
                  failure.setWords(utterance.getWords());
                  return failure.build();
                } else {
                  //Make sure the supplied image is actually a image and not hallucinated by the LLM or a video
                  String imageName = getMatchingNameIgnoreCase(args.get(1).getName(), imageNames);
                  if (!args.get(1).getType().equals("image") || imageName.isEmpty()) {
                    // If the supplied image is a known video, attempt to coerce into corresponding play video action
                    String videoName = getMatchingNameIgnoreCase(args.get(1).getName(), videoNames);
                    if (!videoName.isEmpty()) {
                      if (!durationDefault) {
                        log.error("[correctCommonNLUMistakes] displayImage returned with a video argument and a non-default duration");
                        Utterance.Builder failure = new Utterance.Builder();
                        failure.setSpeaker(utterance.getSpeaker());
                        failure.setAddressee(utterance.getAddressee());
                        failure.setSemantics(Factory.createPredicate("error("+utterance.getAddressee()+",cannot(playVideosForSpecificDurations))"));
                        failure.setWords(utterance.getWords());
                        return failure.build();
                      }
                      if (locationDefault) {
                        log.warn("[correctCommonNLUMistakes] replacing displayImage with corresponding playVideo action");
                        utterance.setSemantics(Factory.createPredicate("playSpecificVideo",actor,args.get(1).toUntyped()));
                        return utterance;
                      } else {
                        log.warn("[correctCommonNLUMistakes] replacing displayImage with corresponding playVideo action");
                        utterance.setSemantics(Factory.createPredicate("playSpecificVideoAtLocation",actor,args.get(1).toUntyped(),args.get(2)));
                        return utterance;
                      }
                    }
                    log.error("[correctCommonNLUMistakes] displayImages' image argument is not a properly typed saved image");
                    Utterance.Builder failure = new Utterance.Builder();
                    failure.setSpeaker(utterance.getSpeaker());
                    failure.setAddressee(utterance.getAddressee());
                    failure.setSemantics(Factory.createPredicate("error("+utterance.getAddressee()+",not(savedImage("+args.get(1).toUntypedString()+")))"));
                    failure.setWords(utterance.getWords());
                    return failure.build();
                  }
                }

                // Arguments are valid, coerce into the appropriate action
                if (durationDefault && locationDefault) {
                  utterance.setSemantics(Factory.createPredicate("displayNamedImage",actor, args.get(1).toUntyped()));
                } else if (durationDefault) {
                  utterance.setSemantics(Factory.createPredicate("displayNamedImageAtLocation",actor,args.get(1).toUntyped(),args.get(2)));
                } else if (locationDefault) {
                  utterance.setSemantics(Factory.createPredicate("displayNamedImageForDuration",actor,args.get(1).toUntyped(),args.get(3)));
                } else {
                  utterance.setSemantics(Factory.createPredicate("displayNamedImageForDurationAtLocation",actor,args.get(1).toUntyped(),args.get(2), args.get(3)));
                }
            }
        } catch (Exception e) {
            log.warn("[correctCommonNLUMistakes] unexpected semantics format");
            return null;
        }
        return utterance;
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
            return TRADE.getAvailableService(new TRADEServiceConstraints().name("getVideoNames").argTypes(Boolean.class)).call(List.class,true);
        } catch (TRADEException e) {
            log.error("[getVideoNames]", e);
        }
        return new ArrayList<>();
    }

    private List<String> getImageNames() {
        try {
            return TRADE.getAvailableService(new TRADEServiceConstraints().name("getImageNames").argTypes(Boolean.class)).call(List.class,true);
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
            return refs.values().stream().map(ref -> {
                if (!ref.properties.isEmpty()) {
                    return ref.properties.get(0).getName();
                } else {
                    return "";
                }
            }).toList();
        } catch (TRADEException e) {
            log.error("[getLocations]", e);
        }

        return new ArrayList<>();
    }

    private List<String> getPatrolPaths() {
        List<String> pathNames = new ArrayList<>();
        Predicate query = Factory.createPredicate("collection","X","Y");
        List<Map<Variable, Symbol>> bindings;
        try {
            bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class,query);
            for (Map<Variable, Symbol> binding : bindings) {
                Symbol name = binding.get(new Variable("X"));
                pathNames.add(name.getName());
            }
        } catch (TRADEException e) {
            log.error("[getPatrolPaths] ", e);
        }
        return pathNames;
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
            for (Map<Variable, Symbol> binding : bindings) {
                Symbol y = binding.get(new Variable("Y"));
                //TODO: should be based on group
                PoseReference ref = refs.get(y);
                if (ref != null) {
                    if (!ref.properties.isEmpty()) {
                        robotLocationString = String.format("The robot is at the following location: %s.\n", ref.properties.get(0).getName());
                    }
                }
            }
        } catch (TRADEException e) {
            log.error("[getWorldState] ", e);
            return "Nothing is known by the robot";
        }

        String output = robotLocationString;
        if (output.isEmpty()) {
            log.warn("[getWorldState] no matching beliefs found for agent {}", agent);
            output = "<Nothing is known by the robot>";
        }
        return output;
    }

    @Override
    protected String getDescriptorType(String agentName, String descriptor) {
        List<String> propertyNames  = getLocations("self", true);
        if (propertyNames.contains(descriptor)) {
            return "location";
        }

        return "";
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
                case "describeAllKnownPatrolPaths":
                case "describeAllLocationsOnPatrolPath":
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
        entry = new LLMActionEntry("goToLocation(?destination:location)",
                "You go to location ?destination:location.",
                "please navigate to %exampleLocation",
                "goToLocation(%exampleLocation:location)",
                10
        );
        exposedActionList.add(entry);
        entry = new LLMActionEntry("dock(?destination:location)",
                "You dock at the given dock ?destination:location.",
                "dock at %exampleLocation",
                "dock(%exampleLocation:location)",
                10
        );
      exposedActionList.add(entry);
        entry = new LLMActionEntry("reportBatteryLevel()",
                "You report the current battery level of the robot as a percentage",
                "how much charge do you have",
                "reportBatteryLevel()",
                10
        );
        exposedActionList.add(entry);
        entry = new LLMActionEntry("follow()",
                "You follow the user",
                "follow me",
                "follow()",
                10
        );
        exposedActionList.add(entry);
        entry = new LLMActionEntry("say(?words)",
                "You say ?words",
                "say hello, it's me",
                "say(hello, it's me)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("stopFollow()",
                "You stop following the user",
                "stop following me",
                "stopFollow()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("escortToLocation(?name, ?destination:location)",
                "You escort ?name to location ?destination:location",
                "bring Owen to %exampleLocation",
                "escortToLocation(owen,%exampleLocation:location)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("escortFromLocationToLocation(?name, ?start:location, ?destination:location)",
                "You escort ?name from location ?start:location to location ?destination:location",
                "bring our guest from %exampleLocation to the dock",
                "escortFromLocationToLocation(guest,%exampleLocation:location,dock:location)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("saveLocation(?name)",
                "You save your current location under the name ?name",
                "save this area as location one",
                "saveLocation(location one)", 10);
        exposedActionList.add(entry);
        //entry = new LLMActionEntry("saveLocationsToFile()",
        //        "You backup all your currently saved locations",
        //        "backup your current locations",
        //        "saveLocationsToFile()", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("definePatrolPath(?name)",
        //        "You start learning a new patrol path called ?name",
        //        "I want to teach you a new patrol path named lab circuit",
        //        "definePatrolPath(lab circuit)", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("definePatrolPath()",
        //        "You start learning a new patrol path",
        //        "I want to teach you a new patrol path",
        //        "definePatrolPath()", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("patrol(?pathName:path, ?actionName)",
        //        "You navigate along the patrol path ?pathName:path while performing the action ?actionName at each location",
        //        "perform %examplePath patrol with wifi evaluation",
        //        "patrol(%examplePath:path, wifiEvaluation)", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("wifiEvaluation(?location:location)",
        //        "You measure the strength of the wifi signal at location ?location:location",
        //        "check wifi near %exampleLocation",
        //        "wifiEvaluation(%exampleLocation:location)", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("reportWifiForLocation(?location:location)",
        //        "You report the wifi signal strength at location ?location:location",
        //        "what is the wifi signal quality at %exampleLocation",
        //        "reportWifiForLocation(%exampleLocation:location)", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("reportWifiForPatrol(?pathName:path)",
        //        "You report the wifi signal quality at every location along patrol path ?pathName:path",
        //        "report the wifi signal quality for patrol path %examplePath",
        //        "reportWifiForPatrol(%examplePath:path)", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("takePhoto(?location:location)",
        //        "You take a photo at location ?location:location",
        //        "take a photo of %exampleLocation",
        //        "takePhoto(%exampleLocation:location)", 10);
        //exposedActionList.add(entry);
        entry = new LLMActionEntry("sit()",
                "You lower your screen",
                "sit down",
                "sit()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("stand()",
                "You raise your screen",
                "stand up",
                "stand()", 10);
        exposedActionList.add(entry);
        //entry = new LLMActionEntry("deliver(?objectName, ?location:location)",
        //        "You deliver ?objectName to location ?location:location",
        //        "bring the mug to %exampleLocation",
        //        "deliver(mug,%exampleLocation:location)", 10);
        //exposedActionList.add(entry);
        entry = new LLMActionEntry("goHome()",
                "You go to your charging dock and charge",
                "go back to charge",
                "goHome()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("scheduleGoal()",
                "You begin a dialogue interaction with the user to schedule future execution of a task",
                "I want to schedule a new task",
                "scheduleGoal()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("modifyScheduledGoal()",
                "You begin a dialogue interaction with the user to modify the timing of a future already scheduled task",
                "I want to change the timing of an action",
                "modifyScheduledGoal()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describeScheduledGoals()",
                "You report what your currently scheduled goals are",
                "what are you scheduled to do",
                "describeScheduledGoals()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describeCurrentGoal()",
                "You report what your current task is",
                "what are you doing now",
                "describeCurrentGoal()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describePendingGoals()",
                "You report what your currently pending/upcoming tasks are",
                "what are you doing after your current task",
                "describePendingGoals()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("cancelSystemGoals()",
                "You stop and fully discard your current task",
                "stop your current task",
                "cancelSystemGoals()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("suspendSystemGoalsBlocking()",
                "You pause your current task, which can be later resumed",
                "pause what you're doing",
                "suspendSystemGoalsBlocking()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("resumeSystemGoals()",
                "You resume the currently paused task",
                "resume previous task",
                "resumeSystemGoals()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("cancelAllScheduledGoals()",
                "You cancel all currently scheduled goals which have yet to begin execution",
                "cancel all scheduled tasks",
                "cancelAllScheduledGoals()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("cancelScheduledGoal()",
                "You begin a dialogue interaction with the user to cancel a particular scheduled goal",
                "I want to cancel a scheduled task",
                "cancelScheduledGoal()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("learnNewTask()",
                "You begin a dialogue interaction with the user to learn the steps of a new instructable action",
                "i will teach you a new task",
                "learnNewTask()", 10);
        exposedActionList.add(entry);
        List<String> endLearningExamples = new ArrayList<>();
        //Make version of constructor which takes in one translation and multiple examples
        List<String> endLearningTranslations = new ArrayList<>();
        endLearningExamples.add("those are the steps of the task");
        endLearningExamples.add("im done");
        endLearningTranslations.add("endActionLearning()");
        endLearningTranslations.add("endActionLearning()");
        entry = new LLMActionEntry("endActionLearning()",
                "You conclude learning the steps of a new instructable action",
                endLearningExamples,
                endLearningTranslations, 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describeCurrentLocation()",
                "You report what location you believe you are currently at",
                "what is your current location",
                "describeCurrentLocation()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describeAllKnownLocations()",
                "You list all currently saved locations",
                "what are your saved locations",
                "describeAllKnownLocations()", 10);
        exposedActionList.add(entry);
        //entry = new LLMActionEntry("describeAllKnownPatrolPaths()",
        //        "You list all currently saved patrol paths",
        //        "what paths do you know about",
        //        "describeAllKnownPatrolPaths()", 10);
        //exposedActionList.add(entry);
        //entry = new LLMActionEntry("describeAllLocationsOnPatrolPath(?pathName:path)",
        //        "You list all locations included along the saved patrol path named ?pathName:path",
        //        "what locations are in %examplePath",
        //        "describeAllLocationsOnPatrolPath(%examplePath:path)", 10);
        //exposedActionList.add(entry);
        List<String> questionExamples = new ArrayList<>();
        List<String> questionTranslations = new ArrayList<>();
        questionExamples.add("could you repeat that");
        questionExamples.add("what is the question");
        questionTranslations.add("repeatLastQuestion()");
        questionTranslations.add("repeatLastQuestion()");
        entry = new LLMActionEntry("repeatLastQuestion()",
                "You repeat the last question you asked, which the human still needs to answer",
                questionExamples,
                questionTranslations, 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportTaughtActions()",
                "You list all of the actions which have been taught to you by a user",
                "What actions have you been taught",
                "reportTaughtActions()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("forgetTaughtAction(?actionPredicate)",
                "You forget a previously taught action represented by ?actionPredicate. ?actionPredicate should be replaced by a populated instance of an INSTRUCT Semantics from the previous examples.",
                "forget how to go to %exampleLocation",
                "forgetTaughtAction(goToLocation(%exampleLocation:location))", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportIfCanPerform(?actionPredicate)",
                "You report whether you know how to perform the action represented by ?actionPredicate. ?actionPredicate should be replaced by a populated instance of an INSTRUCT Semantics from the previous examples.",
                "do you know how to escort a patient from %exampleLocation to the dock",
                "reportIfCanPerform(escortFromLocationToLocation(guest,%exampleLocation:location,dock:location))", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportInstructableActions()",
                "You report the list of actions a user is able to instruct you through natural language to perform",
                "what can you be instructed to do",
                "reportInstructableActions())", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("describeActionSteps(?action)",
                "You describe an action by listing out the steps which would be performed when it is executed. ?action can be replaced by a populated instance of an INSTRUCT Semantics from the previous examples OR be just the name of an action if no specific arguments are supplied.",
                "How would you escort someone",
                "describeActionSteps(escort)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("learnTourAction()",
                "You begin a dialogue interaction with the user where they teach you how to perform a new type of tour",
                "I want to teach you how to perform a tour",
                "learnTourAction()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportKioskSettings()",
                "You report the current configuration of kiosk mode; Whether it is enabled and the currently configured kiosk timeout and action.",
                "what is your kiosk configuration",
                "reportKioskSettings()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("updateKioskTimeout()",
                "You begin a dialogue interaction with the user to update the configured kiosk timeout",
                "I would like to update the kiosk timeout",
                "updateKioskTimeout()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("setKioskTimeout(?duration, ?timeunit)",
                "You set the kiosk timeout to ?duration ?timeunits. ?timeunit can take the possible values of the Java TIMEUNIT enum.",
                "Set the kiosk timeout to 30 minutes",
                "setKioskTimeout(30,minutes)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("setKioskModeEnabled(?enabled)",
                "You set whether or not kiosk mode is enabled. ?enabled can take the values true or false",
                "turn on kiosk mode",
                "setKioskModeEnabled(true)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("updateKioskAction()",
                "You begin a dialogue interaction with the user to update the configured kiosk action",
                "I want to change the kiosk action",
                "updateKioskAction()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("setVideoVolume(?volume)",
                "You set the volume of videos played on your screen. Acceptable values are whole numbers between 0 and 100.",
                "Set the video volume to max",
                "setVideoVolume(100)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("listSavedVideos()",
                "You list out the collection of videos you currently have saved",
                "what videos can you play",
                "listSavedVideos()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportVideoVolume()",
                "You report the current volume videos are set to play at",
                "what is the video volume set to",
                "reportVideoVolume()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("reportIsVideoSaved(?videoName:video)",
                "You answer whether you have a video saved under the name ?videoName",
                "Do you have the %exampleVideo video saved",
                "reportIsVideoSaved(?videoName)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("waitForDuration(?seconds)",
                "You wait idly for ?seconds seconds. ?seconds must be a positive whole number.",
                "Then wait for a minute",
                "waitForDuration(60)", 10);
        exposedActionList.add(entry);
        //Experimenting with new format for video/image actions
        List<String> videoInstructions = new ArrayList<>();
        List<String> videoTranslations = new ArrayList<>();
        videoInstructions.add("play video");
        videoTranslations.add("playVideo(default,default)");
        videoInstructions.add("Show the %exampleVideo video");
        videoTranslations.add("playVideo(%exampleVideo:video,default)");
        videoInstructions.add("Play videos at %exampleLocation");
        videoTranslations.add("playVideo(default,%exampleLocation:location)");
        videoInstructions.add("Go to %exampleLocation then play the %exampleVideo video");
        videoTranslations.add("playVideo(%exampleVideo:video,%exampleLocation:location)");
        entry = new LLMActionEntry("playVideo(?videoName:video,?location:location)",
                "You begin playback of a video from your saved collection at some location. This action is special and has different meanings based on how it is parameterized. It has two arguments, the video to be played and the location to play it at. The user can instruct this action with any number of these things defined or not defined. If a user does not supply a specific value for any of these arguments, fill it in with the value \"default\". A value of default for the video means that the video played will be the next video in your saved collection, continuing where you left off. For locations, it means that you will play the video at your current location.",
                videoInstructions,
                videoTranslations, 10);
        exposedActionList.add(entry);
        List<String> imageInstructions = new ArrayList<>();
        List<String> imageTranslations = new ArrayList<>();
        imageInstructions.add("Display the %exampleImage image");
        imageTranslations.add("displayImage(%exampleImage:image,default,default)");
        imageInstructions.add("Display the %exampleImage image at the %exampleLocation location");
        imageTranslations.add("displayImage(%exampleImage:image,%exampleLocation:location,default)");
        imageInstructions.add("Display the %exampleImage image for a minute");
        imageTranslations.add("displayImage(%exampleImage:image,default,60)");
        imageInstructions.add("Display the %exampleImage image at the %exampleLocation location for 45 seconds");
        imageTranslations.add("displayImage(%exampleImage:image,%exampleLocation:location,45)");
        entry = new LLMActionEntry("displayImage(?imageName:image,?location:location,?seconds)",
                "You display an image from your saved collection at some location for a certain duration. This action is also special and has different meanings based on how it is parameterized. It has three arguments, the image to be displayed, the location to display it at, and the duration to display it for. When the user instructs this action, the image must explicitly be defined, but the location and duration may be left undefined. If a user does not supply a specific value for these two arguments, fill it in with the value \"default\". A value of default for the location means that you will display the image at your current location. A default value for the duration means that the image will be displayed indefinitely.",
                imageInstructions,
                imageTranslations, 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("listSavedImages()",
                "You list out the collection of images you currently have saved",
                "what images can you display",
                "listSavedImages()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("urgentPriority(?actionPredicate)",
                "You submit an action represented by ?actionPredicate with urgent priority. This means that this action will interrupt execution of any other currently submitted actions and be immediately prioritized. ?actionPredicate should be replaced by a populated instance of an INSTRUCT Semantics from the previous examples.",
                "go to %exampleLocation right now",
                "urgentPriority(goToLocation(%exampleLocation:location))", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("cancelTaskInQueueByIndex(?index)",
                "You cancel and remove the task in task queue at position ?index if it exists. ?index is a 0-based index.",
                "Remove the first task from the queue",
                "cancelTaskInQueueByIndex(0)", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("enableAdminMode()",
                "Allow certain actions which require administrator privileges to be instructed through voice directly to the robot.",
                "enable admin mode",
                "enableAdminMode()", 10);
        exposedActionList.add(entry);
        entry = new LLMActionEntry("disableAdminMode()",
                "Reject certain actions which require administrator privileges from being instructed through voice directly to the robot.",
                "disable admin mode",
                "disableAdminMode()", 10);
        exposedActionList.add(entry);
      //entry = new LLMActionEntry("toggleDisplay()",
      //        "Change the output on your display between the two possible inputs",
      //        "toggle display",
      //        "toggleDisplay()", 10);
      //exposedActionList.add(entry);
      entry = new LLMActionEntry("switchDisplayToTelepresence()",
              "Change the output on your display to the telepresence stream",
              "start telepresence",
              "switchDisplayToTelepresence()", 10);
      exposedActionList.add(entry);
      entry = new LLMActionEntry("switchDisplayToTRACS()",
              "Change the output on your display to what is shown by TRACS, the default setting",
              "return to TRACS",
              "switchDisplayToTRACS()", 10);
      exposedActionList.add(entry);
    }
}
