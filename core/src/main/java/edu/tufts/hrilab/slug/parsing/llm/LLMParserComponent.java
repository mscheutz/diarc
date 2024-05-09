/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.llm;

import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import com.google.gson.Gson;
import edu.tufts.hrilab.interfaces.NLUInterface;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.util.Http;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LLMParserComponent extends DiarcComponent implements NLUInterface {

  public String endpoint = "http://localhost:8000";
  private String service = null;
  private String prompt = null;

  public LLMParserComponent() {
    super();
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("endpoint").hasArg().argName("string").desc("Set the endpoint for calling the LLM parser service").build());
    options.add(Option.builder("service").hasArg().argName("string").desc("TRADE Service to call in place of parseUtterance").build());
    options.add(Option.builder("prompt").hasArg().argName("string").desc("Prompt to use with an LLM.").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("endpoint")) {
      endpoint = cmdLine.getOptionValue("endpoint");
    } else {
      log.info("Using default endpoint for LLMParserComponent: " + endpoint);
    }
    if (cmdLine.hasOption("service")) {
      service = cmdLine.getOptionValue("service");
    }
    if (cmdLine.hasOption("prompt")) {
      prompt = cmdLine.getOptionValue("prompt");
    }
  }

  /**
   * Sends a POST request to the specified endpoint with the given input as the request body
   * and retrieves the response as a ParserResponse object.
   *
   * @param input The input string to be sent in the request body.
   * @return The ParserResponse object obtained from the response.
   */
  public ParserResponse getResponse (String input) {
    Map<String, String> headers = new HashMap<>();
    Gson gson = new Gson();
    ParserRequestBody requestBody = new ParserRequestBody(input);
    String rawResponse = Http.sendPostRequest(endpoint, requestBody, headers);
    log.debug("Response: " + rawResponse);
    return gson.fromJson(rawResponse, ParserResponse.class);
  }

  /**
   * Parses the input Utterance to obtain semantic information.
   * Optionally use a TRADEService defined in the service property.
   * Default behavior is to use getResponse() to retrieve from parser service.
   *
   * @param input The input Utterance to parse.
   * @return The parsed Utterance with updated semantic information.
   */
  @Override
  public Utterance parseUtterance(Utterance input) {
    String inputString = input.getWordsAsString();
    ParserResponse response = null;

    log.info("parseUtterance: " + inputString);

    if (service == null) {
      try {
        response = getResponse(inputString);
      } catch (Exception ex) {
        log.error("Error getting response from " + endpoint, ex);
      }
    } else {
      try {
        response = TRADE.getAvailableService(new TRADEServiceConstraints().name(service).argTypes(String.class)).call(ParserResponse.class, inputString);
      } catch (TRADEException ex) {
        log.error("Error calling " + service + ".", ex);
      }
    }

    if (response == null) {
      log.error("Error: Failed to get response");
      return null;
    } else {
      log.info("Got response: " + response.toString());
    }
    Utterance.Builder output = new Utterance.Builder(input);
    output.setUtteranceType(UtteranceType.valueOf(response.intention.intent.toUpperCase()));

    // collect referent info by variable name
    Map<String, Referent> referentMap = new HashMap<>();
    List<Variable> variables = new ArrayList<>();
    for (Referent referent : response.referents) {
      referentMap.put(referent.variable_name, referent);
      variables.add(Factory.createVariable(referent.variable_name, referent.type));
    }

    // populate semantics with semantically typed variables
    Proposition prop = response.intention.proposition;
    Predicate semantics;
    if (UtteranceType.valueOf(response.intention.intent.toUpperCase()) == UtteranceType.INSTRUCT) {
      List<Symbol> args = new ArrayList<>();
      args.add(input.getAddressee());
      Arrays.stream(prop.arguments).forEach(arg -> args.add(Factory.createFOL(arg)));
      semantics = Factory.createPredicate(prop.text, args);
    } else {
      semantics = Factory.createPredicate(prop.text, prop.arguments);
    }
    output.setSemantics(semantics.copyWithNewVariableTypes(variables));

    // populate supplemental semantics
    for (Descriptor descriptor : response.descriptors) {
      Predicate descriptorPred = Factory.createPredicate(descriptor.text, descriptor.arguments);
      output.addSupplementalSemantics(descriptorPred.copyWithNewVariableTypes(variables));
    }

    // add tier assignments to supplemental semantics
    variables.forEach(var -> output.addTierAssignment(var, Factory.createSymbol(referentMap.get(var.getName()).toString().toUpperCase())));

    return output.build();
  }

}
