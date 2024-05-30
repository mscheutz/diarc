/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.llama;

import edu.tufts.hrilab.llm.openai.request.*;
import edu.tufts.hrilab.llm.openai.response.*;
import edu.tufts.hrilab.llm.llama.request.*;
import edu.tufts.hrilab.llm.llama.response.*;

import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Message;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.util.Http;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import static java.util.Map.entry;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import io.github.cdimascio.dotenv.Dotenv;
import io.github.cdimascio.dotenv.DotenvException;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import java.io.InputStreamReader;

public class LlamaClient {

  static private Logger log = LoggerFactory.getLogger(LlamaClient.class);

  private String completionEndpoint      = "http://localhost:8080/completion";
  //use OpenaiClient
  //private String chatCompletionEndpoint  = "http://localhost:8080/v1/chat/completions";

  private float temperature = 0.5f;
  private String[] stopWords = {};

  public LlamaClient () {

  }
  /**
   * Sets the endpoint for the llama.cpp server.
   *
   * @param endpoint - The base endpoint to set
   */
  public void setEndpoint (String endpoint) {
    completionEndpoint = endpoint + "/completion";
    //chatCompletionEndpoint = endpoint + "/v1/chat/completions";
    log.info("Using llama.cpp endpoint: " + endpoint);
  }

  /**
   * Sets the stop words array for terminating token stream on matching words.
   *
   * @param stopWordsArr - An array of stop words to set
   */
  public void setStopWords (String[] stopWordsArr) {
    stopWords = stopWordsArr;
    log.info("Using stop words: " + String.join(", ", stopWordsArr));
  }

  /**
   * Sets the temperature of the model.
   *
   * @param temperatureFloat - The temperature value to set
   */
  public void setTemperature (float temperatureFloat) {
    temperature = temperatureFloat;
    log.info(String.format("Using temperature: %s", temperature));
  }

  /**
   * Generates a completion of the given prompt using the llama.cpp API service.
   * @param prompt the prompt for which to generate the completion
   * @return a LlamaCompletionResponse containing the generated completion and additional information
   **/
  public LlamaCompletionResponse completion (Prompt prompt) {
    return completion(prompt.toString());
  }
  public LlamaCompletionResponse completion (Symbol prompt) {
    return completion(prompt.toString());
  }

  /**
   * Generates a completion of the given prompt and model using the llama.cpp API service.
   * @param prompt the prompt for which to generate the completion
   * @return a LlamaCompletionResponse containing the generated completion and additional information
   **/
  public LlamaCompletionResponse completion (Symbol model, Symbol prompt) {
    return completion(prompt.toString());
  }
  public LlamaCompletionResponse completion(String prompt) {
    LlamaCompletionRequestBody requestBody = new LlamaCompletionRequestBody(prompt);
    if (stopWords.length > 0) {
      requestBody.addStopWords(stopWords);
    }
    requestBody.setTemperature(temperature);
    return completion(requestBody);
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
    String response = Http.sendPostRequest(endpoint, requestBody, headers);
    log.debug("Response: " + response);
    return gson.fromJson(response, LlamaCompletionResponse.class);
  }

  /**
   * Generates a chat completion using the LLaMA service and a prompt string.
   * @param prompt the prompt string provided by the user
   * @return the LlamaCompletionResponse containing the completed chat.
   **/
  public LlamaCompletionResponse chatCompletion (Symbol prompt) {
    return chatCompletion(prompt.toString());
  }
  public LlamaCompletionResponse chatCompletion (String prompt) {
    List<Message> messages = new ArrayList<>();
    messages.add(new Message("system", "You are a helpful assistant."));
    messages.add(new Message("user", prompt));
    return chatCompletion(messages);
  }

  /**
   * Performs an LLaMA chat completion using the specified model and the given list of messages.
   * This method creates a new LlamaRequestBody object with the provided model and messages.
   * @param messages a list of messages to be sent for chat completion
   * @return an LlamaCompletionResponse object representing the response from LLaMA
   **/
  public LlamaCompletionResponse chatCompletion (List<Message> messages) {
    Chat chat = new Chat(messages);
    return chatCompletion(chat);
  }

  /**
   * Performs a chat completion using LLaMA with the set model and the given chat messages.
   * @param chat the chat containing the messages to use for the completion
   * @return an LlamaCompletionResponse object representing the response from LLaMA
   **/
  public LlamaCompletionResponse chatCompletion (Chat chat) {
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
    String response = Http.sendPostRequest(endpoint, requestBody, headers);
    log.debug("Response: " + response);
    return gson.fromJson(response, LlamaCompletionResponse.class);
  }
}