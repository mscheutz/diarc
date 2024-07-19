/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.util.Http;

import java.awt.*;
import java.beans.Transient;
import java.io.InputStreamReader;

import java.util.*;
import java.util.List;

import static java.util.Map.entry;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import edu.tufts.hrilab.diarc.DiarcComponent;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Symbol;

import edu.tufts.hrilab.llm.openai.OpenaiClient;
import edu.tufts.hrilab.llm.openai.request.*;
import edu.tufts.hrilab.llm.openai.response.*;
import edu.tufts.hrilab.llm.llama.LlamaClient;
import edu.tufts.hrilab.llm.llama.request.*;
import edu.tufts.hrilab.llm.llama.response.*;

public class LLMComponent extends DiarcComponent {
  static private Logger log = LoggerFactory.getLogger(LLMComponent.class);

  public String service = "openai";
  public String model = "gpt-4o";
  public float temperature = 0.5f;

  private Tokenizer tokenizer = new Tokenizer();
  private OpenaiClient openai = new OpenaiClient();
  private LlamaClient llama = new LlamaClient();

  /**
   * LLaMA (llama.cpp server)
   */

  private String llamaCompletionEndpoint      = "http://localhost:8080/completion";
  private String llamaChatCompletionEndpoint  = "http://localhost:8080/v1/chat/completions";

  public LLMComponent() {
    super();
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("service").hasArg().argName("string").desc("Set the LLM service to use. [openai, llama]").build());
    options.add(Option.builder("model").hasArg().argName("string").desc("Set the OpenAI LLM model to use. [ " + String.join(", ", OpenaiClient.chatCompletionModels.keySet()) + "]").build());
    options.add(Option.builder("endpoint").hasArg().argName("string").desc("Set the endpoint of the LLM service to use.").build());
    options.add(Option.builder("stopwords").hasArg().argName("string").desc("Set additional stopwords to use with models and services that support them. Comma-separated list.").build());
    options.add(Option.builder("temperature").hasArg().argName("float").desc("Set the temperature of the model").build());
    return options;
  }

  @Override
  protected void parseArgs (CommandLine cmdLine) {
    if (cmdLine.hasOption("service")) {
      service = cmdLine.getOptionValue("service").toLowerCase();
      log.debug("Using service: " + service);
    }

    if (cmdLine.hasOption("endpoint")) {
      setLLMEndpoint(cmdLine.getOptionValue("endpoint"));
    }

    if (cmdLine.hasOption("model")) {
      setLLMModel(cmdLine.getOptionValue("model"));
      log.debug("Using model: " + model);
    }

    if (cmdLine.hasOption("stopwords")) {
      setLLMStopWords(cmdLine.getOptionValue("stopwords").split(","));
    }

    if (cmdLine.hasOption("temperature")) {
      setLLMTemperature(Float.parseFloat(cmdLine.getOptionValue("temperature")));
    }

    log.info("LLMComponent using " + service + " services with the " + model + " model.");
    tokenizer.setEncoding(service, model);
  }

  /**
   * Sets the LLM service to the given value.
   *
   * @param service a string representing the LLM service
   **/
  @TRADEService
  @Action
  public void setLLMService (Symbol service) {
    setLLMService(service.toString());
  }
  /**
   * Sets the LLM service: [openai, llama].
   *
   * @param serviceStr - The LLM service to set
   */
  @TRADEService
  public void setLLMService (String serviceStr) {
    service = serviceStr;
  }
  /**
   * Sets the LLM endpoint (URL).
   *
   * @param endpoint - The endpoint to set
   */
  @TRADEService
  @Action
  public void setLLMEndpoint (Symbol endpoint) {
    setLLMEndpoint(endpoint.toString());
  }
  @TRADEService
  public void setLLMEndpoint (String endpoint) {
    if (service.equals("llama")) {
      llama.setEndpoint(endpoint);
    } else if (service.equals("openai")) {
      openai.setEndpoint(endpoint);
    }
  }

  /**
   * Sets the language model to use for the language learning model service.
   *
   * @param model the name of the model to set
   **/
  @TRADEService
  @Action
  public void setLLMModel (Symbol model) {
    setLLMModel(model.toString());
  }
  @TRADEService
  public void setLLMModel (String modelStr) {
    if (service.equals("openai")) {
      openai.setModel(modelStr);
    }
    model = modelStr;
  }

  /**
   * Sets the stop words for the LLM to terminate token stream.
   *
   * @param stopWords - An array of stop words to set.
   */
  public void setLLMStopWords (String[] stopWords) {
    if (service.equals("llama")) {
      llama.setStopWords(stopWords);
    } else if (service.equals("openai")) {
      log.warn("OpenAI API does not support stop words");
    }
  }

  /**
   * Sets the temperature for the LLM.
   *
   * @param temperatureFloat - The temperature value to set
   */
  public void setLLMTemperature (float temperatureFloat) {
    if (service.equals("llama")) {
      llama.setTemperature(temperatureFloat);
    } else if (service.equals("openai")) {
      log.warn("OpenAI API does not support temperature");
    }
  }

  /**
   *  Completion Services
   **/

  /**
   * Returns a Completion object for the provided prompt based on the specified service.
   * @param prompt the prompt to generate completion for
   * @return a Completion object for the provided prompt
   **/
  @TRADEService
  @Action
  public Completion completion (Prompt prompt) {
    return completion(prompt.toString());
  }
  @TRADEService
  public Completion completion (String prompt) {
    switch (service.toLowerCase()) {
      case "openai" :
        return new Completion(openai.completion(prompt));
      case "llama" :
        return new Completion(llama.completion(prompt));
      default :
        log.error("Service " + service + " not implemented");
        return null;
    }
  }

  /**
   * Returns a Completion object for the provided prompt based on the specified service
   * and model.
   * @param model the model to use when generating a completion
   * @param prompt the prompt to generate completion for
   * @return a Completion object for the provided prompt
   **/

  @TRADEService
  public Completion completion (String model, Prompt prompt) {
    return completion(model, prompt.toString());
  }
  @TRADEService
  public Completion completion (Symbol model, Prompt prompt) {
    return completion(model.toString(), prompt.toString());
  }
  @TRADEService
  public Completion completion (String model, String prompt) {
    switch (service) {
      case "openai" :
        return new Completion(openai.completion(model, prompt));
      case "llama" :
        log.warn("llama.cpp server cannot change model to " + model);
        return new Completion(llama.completion(prompt));
      default :
        log.error("Service " + service + " not implemented");
        return null;
    }
  }

  /**
   *  Chat Services
   **/

 /**
  *  Returns a Completion object for chat-based completions, based on the specified prompt and set service.
  *  @param prompt the prompt to be used for the chat completion
  *  @return a Completion object containing the completed text and any additional data returned by the API
  **/
  @TRADEService
  @Action
  public Completion chatCompletion (Symbol prompt) {
    return chatCompletion(prompt.toString());
  }
  @TRADEService
  public Completion chatCompletion (String prompt) {
    switch (service) {
      case "openai" :
        return new Completion(openai.chatCompletion(prompt));
      case "llama" :
        return new Completion(llama.chatCompletion(prompt));
      default :
        log.error("Service " + service + " not implemented.");
        return null;
    }
  }

 /**
  *  Returns a Completion object for chat-based completions, based on the specified prompt and model 
  *  using the set service.
  *  @param model the model to use for the chat completion request
  *  @param prompt the prompt to be used for the chat completion
  *  @return a Completion object containing the chat completion response
  **/
  @TRADEService
  @Action
  public Completion chatCompletion (Symbol model, Symbol prompt) {
    return chatCompletion(model.toString(), prompt.toString());
  }
  
  @TRADEService
  public Completion chatCompletion (String model, String prompt) {
    switch (service) {
      case "openai" :
        return new Completion(openai.chatCompletion(model, prompt));
      case "llama" :
        return new Completion(llama.chatCompletion(prompt));
      default :
        log.error("Service " + service + " not implemented");
        return null;
    }
  }

 /**
  * Generates a chat completion for a list of messages using the selected service.
  * @param messages the list of previous messages in the conversation
  * @return a Completion object containing the chat completion response
  **/
  @TRADEService
  @Action
  public Completion chatCompletion (List<Message> messages) {
    switch (service) {
      case "openai" :
        return new Completion(openai.chatCompletion(messages));
      case "llama" :
        return new Completion(llama.chatCompletion(messages));
      default :
        log.error("Service " + service + " not implemented");
        return null;
    }
  }

  /**
   * Generates a Completion object by passing a Chat object to the appropriate chat completion 
   * method based on the currently set service.
   * @param chat the Chat object to use for chat completion
   * @return a Completion object containing the chat completion response
   **/
  @TRADEService
  @Action
  public Completion chatCompletion (Chat chat) {
    switch (service) {
      case "openai" :
        return new Completion(openai.chatCompletion(chat));
      case "llama" :
        return new Completion(llama.chatCompletion(chat));
      default :
        log.error("Service " + service + " not implemented");
        return null;
    }
  }

  /**
   * Generates a Completion object by passing a Chat object to the appropriate chat completion 
   * method based on the currently set service.
   * @param model the model to use for the chat completion request
   * @param chat the Chat object to use for chat completion
   * @return a Completion object containing the chat completion response
   **/
  @TRADEService
  @Action
  public Completion chatCompletion (Symbol model, Chat chat) {
    switch (service) {
      case "openai" :
        return new Completion(openai.chatCompletion(model, chat));
      case "llama" :
        log.warn("llama.cpp server cannot change model to " + model);
        return new Completion(llama.chatCompletion(chat));
      default :
        log.error("Service " + service + " not implemented");
        return null;
    }
  }

  @TRADEService
  @Action
  public Completion visionCompletion (Prompt prompt, byte[] image, Dimension imageSize) {
    List<VisionMessage> vmessages = new ArrayList<VisionMessage>();
    vmessages.add(new VisionMessage("user", prompt.toString(), image, imageSize));
    return visionCompletion(vmessages);
  }

  /**
   * Generates a chat completion using a vision model
   * method based on the currently set service.
   * @param vmessages The vision messages to use
   * @return a Completion object containing the chat completion response
   **/
  @TRADEService
  @Action
  public Completion visionCompletion (List <VisionMessage> vmessages) {
    switch (service) {
      case "openai" :
        return new Completion(openai.visionCompletion(vmessages));
      default :
        log.error("Service " + service + " not implemented");
        return null;
    }
  }

  @TRADEService
  @Action
  public Completion visionCompletion (Prompt prompt) {
    Dimension imageSize = null;
    byte[] image = null;
    try {
      imageSize = TRADE.getAvailableService(new TRADEServiceConstraints().name("getImageSize").argTypes()).call(Dimension.class);
    } catch (TRADEException ex) {
      log.error("Error getting image size from vision component");
      return null;
    }
    try {
      image = TRADE.getAvailableService(new TRADEServiceConstraints().name("getFrame")).call(byte[].class);
    } catch (TRADEException ex) {
      log.error("Cannot get frame for vision inference");
      return null;
    }
    return visionCompletion(prompt, image, imageSize);
  }
}
