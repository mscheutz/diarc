/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.llm.azure.AzureClient;
import edu.tufts.hrilab.llm.hf.HFClient;

import java.awt.*;

import java.util.ArrayList;
import java.util.List;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import edu.tufts.hrilab.diarc.DiarcComponent;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Symbol;

import edu.tufts.hrilab.llm.openai.OpenaiClient;
import edu.tufts.hrilab.llm.llama.LlamaClient;

public class LLMComponent extends DiarcComponent {
  static private Logger log = LoggerFactory.getLogger(LLMComponent.class);

  public String service = "openai";
  public String model = "gpt-4o";
  public float temperature = 0.5f;

  private LLMClient client;

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
    setLLMService(service);

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
    if (service.equals("azure")) {
      client = new AzureClient();
    } else if (service.equals("openai")) {
      client = new OpenaiClient();
    } else if (service.endsWith("hf")) {
      client = new HFClient();
    } else if (service.equals("llama")) {
      client = new LlamaClient();
    } else {
      log.warn("Service {} not supported. Defaulting to openai", service);
      service = "openai";
      client = new OpenaiClient();
    }
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
    client.setEndpoint(endpoint);
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
    if (service.equals("llamahf") || service.equals("t5hf")) {
      ((HFClient)client).setModel(service, modelStr);
    } else {
      client.setModel(modelStr);
    }
  }

  /**
   * Sets the stop words for the LLM to terminate token stream.
   *
   * @param stopWords - An array of stop words to set.
   */
  public void setLLMStopWords (String[] stopWords) {
    client.setStopWords(stopWords);
  }

  /**
   * Sets the temperature for the LLM.
   *
   * @param temperatureFloat - The temperature value to set
   */
  public void setLLMTemperature (float temperatureFloat) {
    client.setTemperature(temperatureFloat);
  }

  /**
   *  Completion Services
   **/

  /**
   * Returns a Completion object for the provided prompt based on the specified service.
   * @param userPrompt the user prompt to generate completion for
   * @return a Completion object for the provided prompt
   **/
  @TRADEService
  @Action
  public Completion completion (String userPrompt) {
    return client.completion(userPrompt);
  }

  /**
   * Returns a Completion object for the provided prompt based on the specified service
   * and model.
   * @param userPrompt the user prompt to generate completion for
   * @param model the model to use when generating a completion
   * @return a Completion object for the provided prompt
   **/
  @TRADEService
  public Completion completion (String userPrompt, String model) {
    return client.completion(userPrompt, model);
  }

  /**
   * Returns a Completion object for the provided prompt based on the specified service
   * and model.
   * @param userPrompt the user prompt to generate completion for
   * @param model the model to use when generating a completion
   * @param systemPrompt the system prompt to generate completion for
   * @return a Completion object for the provided prompt
   **/
  @TRADEService
  public Completion completion (String userPrompt, String model, String systemPrompt) {
    return client.completion(userPrompt,model,systemPrompt);
  }

  /**
   *  Chat Services
   **/

 /**
  *  Returns a Completion object for chat-based completions, based on the specified prompt and set service.
  *  @param userPrompt the user prompt to be used for the chat completion
  *  @return a Completion object containing the completed text and any additional data returned by the API
  **/
  @TRADEService
  public Completion chatCompletion (String userPrompt) {
    return client.chatCompletion(userPrompt);
  }

 /**
  *  Returns a Completion object for chat-based completions, based on the specified prompt and model 
  *  using the set service.
  *  @param model the model to use for the chat completion request
  *  @param userPrompt the user prompt to be used for the chat completion
  *  @return a Completion object containing the chat completion response
  **/
  @TRADEService
  public Completion chatCompletion (String userPrompt, String model) {
    return client.chatCompletion(userPrompt, model);
  }

  /**
   * Returns a Completion object for chat-based completion, based on the specified prompts and model.
   * @param userPrompt the user prompt to generate completion for
   * @param model the model to use when generating a completion
   * @param systemPrompt the system prompt to generate completion for
   * @return a Completion object for the provided prompt
   **/
  @TRADEService
  public Completion chatCompletion (String userPrompt, String model, String systemPrompt) {
    return client.chatCompletion(userPrompt, model, systemPrompt);
  }

 /**
  * Generates a chat completion for a list of messages using the selected service.
  * @param messages the list of previous messages in the conversation
  * @return a Completion object containing the chat completion response
  **/
  @TRADEService
  @Action
  public Completion chatCompletion (List<Message> messages) {
    return client.chatCompletion(messages);
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
    return client.chatCompletion(chat);
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
  public Completion chatCompletion (Chat chat, String model) {
    return client.chatCompletion(chat, model);
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
        return new Completion(((OpenaiClient)client).visionCompletion(vmessages));
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
