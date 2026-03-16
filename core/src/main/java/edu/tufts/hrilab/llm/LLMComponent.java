/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.llm.azure.AzureClient;
import edu.tufts.hrilab.llm.gson.tool.ToolDefinition;
import edu.tufts.hrilab.llm.hf.HFClient;

import java.awt.*;

import java.util.ArrayList;
import java.util.Arrays;
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

  public List<String> services;
  public List<String> models;
  public List<Float> temperatures;

  private List<LLMClient> clients;

  public LLMComponent() {
    super();
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("service").hasArgs().argName("string").desc("Set the LLM services to use (in supplied order until non-null response). [openai, llama, hf, azure]").build());
    options.add(Option.builder("model").hasArgs().argName("string").desc("Set the LLM models to use.").build());
    options.add(Option.builder("endpoint").hasArgs().argName("string").desc("Set the endpoints of the LLM services to use.").build());
    options.add(Option.builder("stopwords").hasArgs().argName("string").desc("Set additional stopwords to use with models and services that support them. Comma-separated lists.").build());
    options.add(Option.builder("temperature").hasArgs().argName("float").desc("Set the temperatures of the models").build());
    options.add(Option.builder("tool").desc("The provided model/service pair supports tool calling").build());
    return options;
  }

  @Override
  protected void parseArgs (CommandLine cmdLine) {
    if (cmdLine.hasOption("service")) {
      setLLMServices(Arrays.asList(cmdLine.getOptionValues("service")));
      log.debug("Using services: " + services);
    } else {
      setLLMService("openai");
    }

    if (cmdLine.hasOption("endpoint")) {
      setLLMEndpoints(cmdLine.getOptionValues("endpoint"));
    }

    if (cmdLine.hasOption("model")) {
      setLLMModels(Arrays.asList(cmdLine.getOptionValues("model")));
      log.debug("Using model: " + models);
    } else {
      setLLMModel("gpt-4o");
    }

    if (cmdLine.hasOption("stopwords")) {
      String[] stopwordsets = cmdLine.getOptionValues("stopwords");
      List<String[]> stopwords = new ArrayList<>();
      for (String set : stopwordsets) {
        stopwords.add(set.split(","));
      }
      setAllLLMStopWords(stopwords);
    }

    if (cmdLine.hasOption("temperature")) {
       String[] tempStrings = cmdLine.getOptionValues("temperature");
       temperatures = new ArrayList<>();
       for (String temp : tempStrings) {
         temperatures.add(Float.parseFloat(temp));
       }
      setLLMTemperatures(temperatures);
    } else {
      setLLMTemperature(0.5f);
    }

//    if (cmdLine.hasOption("tool")) {
//      for (LLMClient client : clients) {
//        client.setToolEnabled(true);
//      }
//    }
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
    setLLMServices(List.of(serviceStr));
  }
  private void setLLMServices(List<String> serviceStrs) {
    clients = new ArrayList<>();
    services = new ArrayList<>();
    for (String serviceStr : serviceStrs) {
      LLMClient client;
      if (serviceStr.equals("azure")) {
        client = new AzureClient();
      } else if (serviceStr.equals("openai")) {
        client = new OpenaiClient();
      } else if (serviceStr.endsWith("hf")) {
        client = new HFClient();
      } else if (serviceStr.equals("llama")) {
        client = new LlamaClient();
      } else {
        log.warn("Service {} not supported. Defaulting to openai", serviceStr);
        serviceStr = "openai";
        client = new OpenaiClient();
      }
      clients.add(client);
      services.add(serviceStr);
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
    if (clients.isEmpty()) {
      log.error("[setLLMEndpoint] no client exists");
      return;
    }
    clients.get(0).setEndpoint(endpoint);
  }
  private void setLLMEndpoints(String[] endpoints) {
    if (endpoints.length > clients.size()) {
      log.error("[setLLMEndpoints] more supplied endpoints than configured clients");
      return;
    }
    for (int i = 0; i < endpoints.length; i++) {
      clients.get(i).setEndpoint(endpoints[i]);
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
    setLLMModels(List.of(modelStr));
  }
  private void setLLMModels(List<String> modelStrs) {
    if (modelStrs.size() > services.size() || modelStrs.size() > clients.size()) {
      log.error("[setLLMModels] more models supplied than configured clients or services");
      return;
    }
    for (int i = 0; i < modelStrs.size(); i++) {
      if (services.get(i).endsWith("hf")) {
        ((HFClient)clients.get(i)).setModel(services.get(i), modelStrs.get(i));
      } else {
        clients.get(i).setModel(modelStrs.get(i));
      }
    }
    models = modelStrs;
  }


  /**
   * Sets the stop words for the LLM to terminate token stream.
   *
   * @param stopWords - An array of stop words to set.
   */
  public void setLLMStopWords (String[] stopWords) {
    List<String[]> stopWordsSets = new ArrayList<>();
    stopWordsSets.add(stopWords);
    setAllLLMStopWords(stopWordsSets);
  }
  private void setAllLLMStopWords(List<String[]> stopWords) {
    if (stopWords.size() > clients.size()) {
      log.error("[setAllLLMStopWords] more stopword sets supplied than clients present");
      return;
    }
    for (int i = 0; i < stopWords.size(); i++) {
      clients.get(i).setStopWords(stopWords.get(i));
    }
  }

  /**
   * Sets the temperature for the LLM.
   *
   * @param temperatureFloat - The temperature value to set
   */
  public void setLLMTemperature (float temperatureFloat) {
    setLLMTemperatures(List.of(temperatureFloat));
  }
  private void setLLMTemperatures(List<Float> temperatureFloats) {
    if (temperatureFloats.size() > clients.size()) {
      log.error("[setLLMTemperatures] more temperatures supplied than clients present");
      return;
    }
    for (int i = 0; i < temperatureFloats.size(); i++) {
      clients.get(i).setTemperature(temperatureFloats.get(i));
    }
    temperatures = temperatureFloats;
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
    Completion completion = null;
    for (LLMClient client : clients) {
      completion = client.completion(userPrompt);
      if (completion != null) {
        break;
      }
    }
    return completion;
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
    Completion completion = null;
    for (LLMClient client : clients) {
      completion = client.completion(userPrompt, model);
      if (completion != null) {
        break;
      }
    }
    return completion;
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
    Completion completion = null;
    for (LLMClient client : clients) {
      completion = client.completion(userPrompt, model, systemPrompt);
      if (completion != null) {
        break;
      }
    }
    return completion;
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
    Completion completion = null;
    for (LLMClient client : clients) {
      completion = client.chatCompletion(userPrompt);
      if (completion != null) {
        break;
      }
    }
    return completion;
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
    Completion completion = null;
    for (LLMClient client : clients) {
      completion = client.chatCompletion(userPrompt, model);
      if (completion != null) {
        break;
      }
    }
    return completion;
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
    Completion completion = null;
    for (LLMClient client : clients) {
      completion = client.chatCompletion(userPrompt, model, systemPrompt);
      if (completion != null) {
        break;
      }
    }
    return completion;
  }

 /**
  * Generates a chat completion for a list of messages using the selected service.
  * @param messages the list of previous messages in the conversation
  * @return a Completion object containing the chat completion response
  **/
  @TRADEService
  @Action
  public Completion chatCompletion (List<Message> messages) {
    Completion completion = null;
    for (LLMClient client : clients) {
      completion = client.chatCompletion(messages);
      if (completion != null) {
        break;
      }
    }
    return completion;
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
    Completion completion = null;
    for (LLMClient client : clients) {
      completion = client.chatCompletion(chat);
      if (completion != null) {
        break;
      }
    }
    return completion;
  }

  //TODO: revisit holistic LLM API and tool support - refactor based on standards (MCP, openai-api) rather than models or
  // inference providers
  @TRADEService
  @Action
  public Completion chatCompletion (Chat chat, List<ToolDefinition> tools) {
    Completion completion = null;
    for (LLMClient client : clients) {
      if (client instanceof OpenaiClient) {
        completion = ((OpenaiClient)client).chatCompletion(chat, tools);
      } else if(client instanceof HFClient && ((HFClient) client).service.equals("hf")) {
        completion = ((HFClient)client).chatCompletion(chat, tools);
      } else {
        completion = client.chatCompletion(chat);
      }
      if (completion != null) {
        break;
      }
    }
    return completion;
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
    Completion completion = null;
    for (LLMClient client : clients) {
      completion = client.chatCompletion(chat, model);
      if (completion != null) {
        break;
      }
    }
    return completion;
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
    for (int i = 0; i < services.size(); i++) {
      if (services.get(i).equals("openai")) {
        return new Completion(((OpenaiClient)clients.get(i)).visionCompletion(vmessages));
      }
    }
    log.error("No configured services implement vision completions");
    return null;
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
