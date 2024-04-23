/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity.space_station.llm;

import java.io.*;
import java.util.regex.Pattern;
import java.util.regex.Matcher;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.HashMap;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.net.URL;
import java.net.URI;
import java.net.URISyntaxException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import edu.tufts.hrilab.diarc.DiarcComponent;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import edu.tufts.hrilab.slug.common.Utterance;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import edu.tufts.hrilab.consultant.pose.*;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.slug.parsing.llm.ParserResponse;
import edu.tufts.hrilab.util.resource.Resources;

public class UnitySpaceStationLLM extends DiarcComponent {
  static private Logger log = LoggerFactory.getLogger(UnitySpaceStationLLM.class);

  private static String refsDirectoryPath = "config/edu/tufts/hrilab/map";
  Pattern agentPattern = Pattern.compile("agent:");
  Matcher agentMatcher = agentPattern.matcher("");

  private String refsFile;
  HashMap<String, Symbol> locationRefs;

  public UnitySpaceStationLLM() {
    super();
  }

  /**
   * Simplified Pose Reference class only used for reading/writing JSON files.
   */
  class PoseRefJson {
    public List<PoseReferenceJson> poseReferences;
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("refs").hasArg().required().argName("refs").desc("Use a map file for reference resolution").build());
    return options;
  }

  @Override
  protected void parseArgs (CommandLine cmdLine) {
    if (cmdLine.hasOption("refs")) {
      refsFile = cmdLine.getOptionValue("refs");
    }
  }

  @Override
  protected void init () {
    Gson gson = new Gson();
    String[] parts;
    String refId;
    List<String> properties;

    String filename = Resources.createFilepath(UnitySpaceStationLLM.refsDirectoryPath, refsFile);

    InputStream stream = UnitySpaceStationLLM.class.getResourceAsStream(filename);
    if (stream == null) {
      log.error("Resource not found: " + filename);
      return;
    }

    BufferedReader reader = new BufferedReader((new InputStreamReader(stream)));
    PoseRefJson loadedPoseRefs = gson.fromJson(reader, PoseRefJson.class);
    locationRefs = new HashMap<>();
    for (PoseReferenceJson poseRef : loadedPoseRefs.poseReferences) {
      refId = poseRef.getRefId();
      properties = poseRef.getProperties();
      parts = properties.get(0).split("\\(");
      locationRefs.put(parts[0], Factory.createSymbol(refId));
    }
  }

  @TRADEService
  public void LLMListenSpaceStationFinetune (String message) {
    log.warn("Deprecated for new NLU pipeline");
    /*
    Prompt prompt = Prompts.getPrompt("spaceStationFinetunedLlama");
    Chat chat = new Chat(prompt);
    Object oResponse = null;
    String jsonString = "";
    LlamaSpaceStationResponse lssRes = null;
    String predicateString = "";
    String dialogGoalString = "";
    String actor = getAgent();

    chat.addUserMessage(message);

    try {
      oResponse = TRADE.callThe("chatCompletion", chat);
    } catch (TRADEException e) {
      log.error("Error performing chat completion", e);
    }

    Completion response = (Completion) oResponse;
    jsonString = response.toString();

    if (!jsonString.contains("}")) {
      jsonString += "}";
    }

    try {
      lssRes = LlamaSpaceStationResponse.fromString(jsonString);
    } catch (Exception e) {
      log.error("Error parsing response: ", e);
    }

    if (lssRes != null) {
      predicateString = lssRes.toPredicate(actor);
      dialogGoalString = lssRes.toDialogGoal(actor, predicateString);
      log.info(dialogGoalString);
      Predicate goal = Factory.createPredicate(dialogGoalString);

      try {
        TRADE.callThe("submitGoal", goal);
      } catch (TRADEException e) {
        log.error("Error submitting goal.", e);
      }
    }
    */
  }

  @TRADEService
  public ParserResponse spaceStationLLMParser (String utterance) {
    Prompt prompt = Prompts.getPrompt("spaceStationFinetunedLlama");
    Chat chat = new Chat(prompt);
    String actor = getAgent();
    Completion oResponse = null;
    String jsonString = "";
    LlamaSpaceStationResponse lssRes = null;

    chat.addUserMessage(utterance);

    try {
      oResponse = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion")).call(Completion.class, chat);
    } catch (TRADEException e) {
      log.error("Error performing chat completion", e);
    }

    Completion response = oResponse;
    jsonString = response.toString();

    if (!jsonString.contains("}")) {
      jsonString += "}";
    }
    log.debug("LLM response object: " + jsonString);
    try {
      lssRes = LlamaSpaceStationResponse.fromString(jsonString);
    } catch (Exception e) {
      log.error("Error parsing response: ", e);
    }

    return lssRes.toParserResponse();
  }

  private String getAgent () {
    List<String> groups = getMyGroups();
    for (String group : groups) {
      agentMatcher.reset(group);
      if (agentMatcher.find()) {
        return group.split(":")[1];
      }
    }
    return "robot1";
  }

  public static String toSpoken (String str) {
    return String.join(" ", str.split("(?<=[a-z])(?=[A-Z])"));
  }

  public static String capitalize (String str) {
    return str.substring(0, 1).toUpperCase() + str.substring(1);
  }

  public static Symbol toFullTube (Symbol area, Symbol side, Symbol number) {
    return Factory.createSymbol( area.toString() + capitalize(side.toString()) + capitalize(number.toString()) );
  }

  public static Symbol toTubeNumber (Symbol side, Symbol number) {
    return Factory.createSymbol( capitalize(side.toString()) + capitalize(number.toString()) );
  }

  public static Symbol getTubeSide (Symbol tubeCombined) {
    String[] parts = tubeCombined.toString().split("(?=[A-Z])");
    return Factory.createSymbol( parts[0].toLowerCase() );
  }

  public static Symbol getTubeNumber (Symbol tubeCombined) {
    String[] parts = tubeCombined.toString().split("(?=[A-Z])");
    return Factory.createSymbol( parts[1].toLowerCase() );
  }

  public static List<Symbol> fromFullTube (Symbol fullTube) {
    List <Symbol> symbolList = new ArrayList<>();
    for (String str : fullTube.toString().split("(?<!(^|[A-Z]))(?=[A-Z])|(?<!^)(?=[A-Z][a-z])")) {
      symbolList.add( Factory.createSymbol(str.toLowerCase()) );
    }
    return symbolList;
  }

  @TRADEService
  @Action
  public void stylizeUnitySpeak (Symbol original) {
    Prompt prompt = Prompts.getPrompt("spaceStationSpeechStyle");
    Symbol model = Factory.createSymbol("gpt-3.5-turbo");
    Symbol actor = Factory.createSymbol("robot1");
    Chat chat = new Chat(prompt);
    Completion oResponse = null;

    chat.addUserMessage(original.toString());

    try {
      oResponse = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Symbol.class,Chat.class)).call(Completion.class, model, chat);
    } catch (TRADEException e) {
      log.error("Error getting a completion.", e);
    }

    Completion response = oResponse;
    String strResponse = response.toString().trim();
    Symbol statement = Factory.createSymbol(strResponse);

    log.info("Response: " + strResponse);

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("sayText").argTypes(Symbol.class,String.class)).call(void.class, actor, statement);
    } catch (TRADEException e) {
      log.error("Error calling sayText.", e);
    }
  }

  @TRADEService
  @Action
  public Symbol resolveReferenceLLM (Symbol location) {
    if (locationRefs.containsKey(location.toString())) {
      return locationRefs.get(location.toString());
    }
    return null;
  }
}