/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.llama;

import edu.tufts.hrilab.llm.LLMClient;
import edu.tufts.hrilab.llm.message.TextMessage;
import edu.tufts.hrilab.llm.llama.request.*;
import edu.tufts.hrilab.llm.llama.response.*;

import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.message.Message;
import edu.tufts.hrilab.util.Http;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import com.google.gson.Gson;

public class LlamaClient implements LLMClient {

  static private Logger log = LoggerFactory.getLogger(LlamaClient.class);

  private String completionEndpoint      = "http://localhost:8080/completion";

  private float temperature = 0.5f;
  private String[] stopWords = {};

  public LlamaClient () {

  }
  /**
   * Sets the endpoint for the llama.cpp server.
   *
   * @param endpoint - The base endpoint to set
   */
  @Override
  public void setEndpoint (String endpoint) {
    completionEndpoint = endpoint + "/completion";
    //chatCompletionEndpoint = endpoint + "/v1/chat/completions";
    log.info("Using llama.cpp endpoint: " + endpoint);
  }

  @Override
  public void setModel(String model) {
    log.info("[setModel] not supported for LlamaClient");
  }

  /**
   * Sets the stop words array for terminating token stream on matching words.
   *
   * @param stopWordsArr - An array of stop words to set
   */
  @Override
  public void setStopWords (String[] stopWordsArr) {
    stopWords = stopWordsArr;
    log.info("Using stop words: " + String.join(", ", stopWordsArr));
  }

  /**
   * Sets the temperature of the model.
   *
   * @param temperatureFloat - The temperature value to set
   */
  @Override
  public void setTemperature (float temperatureFloat) {
    temperature = temperatureFloat;
    log.info(String.format("Using temperature: %s", temperature));
  }

  @Override
  public Completion completion(String userPrompt) {
    return completion(userPrompt, null);
  }

  /**
   * Generates a completion of the given prompt using the llama.cpp API service.
   * @param userPrompt the user prompt for which to generate the completion
   * @return a LlamaCompletionResponse containing the generated completion and additional information
   **/
  @Override
  public Completion completion(String userPrompt, String model) {
    if (model != null) {
      log.error("[completion] Llama client does not currently support multiple models");
      return null;
    }
    return completion(userPrompt, model, "You are a helpful assistant.");
  }

  @Override
  public Completion completion(String userPrompt, String model, String systemPrompt) {
    if (model != null) {
      log.error("[completion] Llama client does not currently support multiple models");
      return null;
    }
    LlamaCompletionRequestBody requestBody = new LlamaCompletionRequestBody(userPrompt);
    if (stopWords.length > 0) {
      requestBody.addStopWords(stopWords);
    }
    requestBody.setTemperature(temperature);
    return new Completion(completion(requestBody));
  }

  /**
   * Generates a chat completion using the LLaMA service and a prompt string.
   * @param userPrompt the prompt string provided by the user
   * @return the LlamaCompletionResponse containing the completed chat.
   **/@Override
  public Completion chatCompletion(String userPrompt) {
    return chatCompletion(userPrompt, null);
  }

  @Override
  public Completion chatCompletion(String userPrompt, String model) {
    if (model != null) {
      log.error("[chatCompletion] Llama client does not currently support multiple models");
      return null;
    }
    return chatCompletion(userPrompt, model, "You are a helpful assistant.");
  }

  @Override
  public Completion chatCompletion(String userPrompt, String model, String systemPrompt) {
    if (model != null) {
      log.error("[chatCompletion] Llama client does not currently support multiple models");
      return null;
    }

    List<Message> messages = new ArrayList<>();
    messages.add(new TextMessage("system", systemPrompt));
    messages.add(new TextMessage("user", userPrompt));
    return chatCompletion(messages);
  }

  /**
   * Performs an LLaMA chat completion using the specified model and the given list of messages.
   * This method creates a new LlamaRequestBody object with the provided model and messages.
   * @param messages a list of messages to be sent for chat completion
   * @return an LlamaCompletionResponse object representing the response from LLaMA
   **/
  @Override
  public Completion chatCompletion(List<Message> messages) {
    return chatCompletion(new Chat(messages));
  }

  @Override
  public Completion chatCompletion(Chat chat) {
    return chatCompletion(chat, null);
  }

  @Override
  public Completion chatCompletion(Chat chat, String model) {
    if (model != null) {
      log.error("[chatCompletion] Llama client does not currently support multiple models");
      return null;
    }
    return new Completion(chatCompletionInner(chat));
  }

  /**
   * Sends a Llama completion request with the given model and request body, and returns the completion response.
   * https://github.com/ggerganov/llama.cpp/tree/master/examples/server
   * @param requestBody the request body containing the prompt and other parameters for the completion request
   * @return the completion response from the Llama API
   **/
  private LlamaCompletionResponse completion (LlamaCompletionRequestBody requestBody) {
    String endpoint = completionEndpoint;
    Map<String, String> headers = new HashMap<>();
    Gson gson = new Gson();

    Http.HTTPRequestResponse httpResponse = Http.sendPostRequest(endpoint, requestBody, headers);
    String response = httpResponse == null ? null : httpResponse.responseString;
    log.debug("Response: " + response);
    return gson.fromJson(response, LlamaCompletionResponse.class);
  }

  /**
   * Performs a chat completion using LLaMA with the set model and the given chat messages.
   * @param chat the chat containing the messages to use for the completion
   * @return an LlamaCompletionResponse object representing the response from LLaMA
   **/
  public LlamaCompletionResponse chatCompletionInner (Chat chat) {
    LlamaCompletionRequestBody requestBody = new LlamaCompletionRequestBody(chat);
    String userStop = chat.getUserRole();
    String robotStop = chat.getRobotRole();
    String stop[] = new String[2 + stopWords.length];
    stop[0] = userStop;
    stop[1] = robotStop;
    if (stopWords.length > 0) {
      for (int i = 0; i < stopWords.length; i++) {
        stop[i + 2] = stopWords[i];
      }
    }
    requestBody.addStopWords(stop);
    requestBody.setTemperature(temperature);
    return chatCompletion(requestBody);

  }
  private LlamaCompletionResponse chatCompletion (LlamaCompletionRequestBody requestBody) {
    String endpoint = completionEndpoint;
    Map<String, String> headers = new HashMap<>();
    Gson gson = new Gson();
    Http.HTTPRequestResponse httpResponse = Http.sendPostRequest(endpoint, requestBody, headers);
    String response = httpResponse == null ? null : httpResponse.responseString;
    log.debug("Response: " + response);
    return gson.fromJson(response, LlamaCompletionResponse.class);
  }
}