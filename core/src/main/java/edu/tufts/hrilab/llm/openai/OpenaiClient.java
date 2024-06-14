/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai;

import edu.tufts.hrilab.llm.openai.request.*;
import edu.tufts.hrilab.llm.openai.response.*;
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

public class OpenaiClient {
  static private Logger log = LoggerFactory.getLogger(OpenaiClient.class);

  private String completionEndpoint     = "https://api.openai.com/v1/completions";
  private String chatCompletionEndpoint = "https://api.openai.com/v1/chat/completions";

  //if alternate is true, use OpenAI API without API Key and limitations on /v1/completions
  private boolean canonical = false;

  private String apiKey = null;
  private int maxTokens;
  private String model = "gpt-3.5-turbo";

  // https://platform.openai.com/docs/models
  // 2024-02-09
  public static Map<String, Integer> chatCompletionModels = Map.ofEntries(
    entry("gpt-4o", 128000),
    entry("gpt-4o-2024-05-13", 128000),
    entry("gpt-3.5-turbo", 4096),
    entry("gpt-3.5-turbo-0125", 16385),
    entry("gpt-3.5-turbo-0301", 4096),
    entry("gpt-3.5-turbo-0613", 4096),
    entry("gpt-3.5-turbo-1106", 16385),
    entry("gpt-3.5-turbo-16k", 16385),
    entry("gpt-3.5-turbo-16k-0613", 16385),
    entry("gpt-3.5-turbo-instruct", 4096),
    entry("gpt-3.5-turbo-instruct-0914", 4096),
    entry("gpt-4", 8192),
    entry("gpt-4-0125-preview", 128000),
    entry("gpt-4-0314", 8192),
    entry("gpt-4-0613", 8192),
    entry("gpt-4-1106-preview", 128000),
    entry("gpt-4-32k-0314", 32768)
  );

  public OpenaiClient () {
    try {
      Dotenv dotenv = Dotenv.load();
      apiKey = dotenv.get("OPENAI_API_KEY");
    } catch (DotenvException ex) {
      log.error("Error loading .env file and retreiving API key");
    }
    if (apiKey == null) {
      log.error("OpenAI requires an API key to use their services.");
    }
  }

  public void setEndpoint (String endpoint) {
    chatCompletionEndpoint = endpoint + "/v1/chat/completions";
    canonical = false;
    log.warn("Using alternate endpoint for OpenAI client: " + endpoint);
  }

  public void setModel (String modelStr) {
    if (chatCompletionModels.containsKey(model)) {
      maxTokens = chatCompletionModels.get(model);
      log.debug("Model context window: " + maxTokens);
    } else {
      log.error("OpenAI does not support chat completion model: " + model);
      return;
    }
    model = modelStr;
  }

  /**
   * Calls the OpenAI API to generate a completion based on the given prompt.
   * @param prompt the prompt used to generate the completion
   * @return an OpenaiCompletionResponse object representing the response from the OpenAI completion endpoint
   **/
  public OpenaiCompletionResponse completion (Prompt prompt) {
    return completion(model, prompt.toString());
  }
  public OpenaiCompletionResponse completion (Symbol prompt) {
    return completion(model, prompt.toString());
  }
  public OpenaiCompletionResponse completion (String prompt) {
    return completion(model, prompt);
  }

  /**
   * Calls the OpenAI API to generate a completion based on the given model and prompt.
   * @param modelSym the model to use when generating a completion
   * @param prompt the prompt used to generate the completion
   * @return an OpenaiCompletionResponse object representing the response from the OpenAI completion endpoint
   **/

  public OpenaiCompletionResponse completion (Symbol modelSym, Prompt prompt) {
    return completion(modelSym.toString(), prompt.toString());
  }
  public OpenaiCompletionResponse completion (Symbol modelSym, Symbol prompt) {
    return completion(modelSym.toString(), prompt.toString());
  }
  public OpenaiCompletionResponse completion (String modelStr, String prompt) {
    OpenaiCompletionRequestBody requestBody = new OpenaiCompletionRequestBody(modelStr, prompt);
    //options
    //requestBody.temperature = temperature;
    requestBody.max_tokens = maxTokens;
    return completion(requestBody);
  }

  /**
   * Sends a POST request to the OpenAI completion endpoint with the given request body and API key.
   * https://platform.openai.com/docs/api-reference/completions
   * @param requestBody the request body to send to the OpenAI completion endpoint
   * @return an OpenaiCompletionResponse object representing the response from the OpenAI completion endpoint
   **/
  public OpenaiCompletionResponse completion (OpenaiCompletionRequestBody requestBody) {
    if (apiKey == null) {
      log.error("Cannot complete request without an API key for OpenAI.");
      return null;
    }

    log.error("OpenAI has deprecated the completion endpoint.");
    return null;
  }


  /**
   * Generates a chat completion using the OpenAI service and a prompt string.
   * @param prompt the prompt string provided by the user
   * @return the OpenaiChatCompletionResponse containing the completed chat.
   **/
  public OpenaiChatCompletionResponse chatCompletion (Symbol prompt) {
    return chatCompletion(prompt.toString());
  }
  public OpenaiChatCompletionResponse chatCompletion (String prompt) {
    List<Message> messages = new ArrayList<>();
    messages.add(new Message("system", "You are a helpful assistant."));
    messages.add(new Message("user", prompt));
    return chatCompletion(messages);
  }

  /**
   * Generates a chat completion response using OpenAI's API with the specified model and prompt.
   * Creates a list of two messages, one from the system and one from the user, to use as input for
   * the chat completion.
   * @param model the model to use for the chat completion
   * @param prompt the text prompt to use for the chat completion
   * @return an OpenaiChatCompletionResponse object representing the response from the OpenAI API
   **/
  public OpenaiChatCompletionResponse chatCompletion (Symbol model, Symbol prompt) {
    return chatCompletion(model.toString(), prompt.toString());
  }
  public OpenaiChatCompletionResponse chatCompletion (String model, String prompt) {
    List<Message> messages = new ArrayList<Message>();
    messages.add(new Message("system", "You are a helpful assistant."));
    messages.add(new Message("user", prompt));
    return chatCompletion(model, messages);
  }

  /**
   * Performs an OpenAI chat completion using the default model and the given list of messages.
   * This method creates a new OpenaiChatRequestBody object with the default model and the provided messages.
   * @param messages a list of messages to be sent for chat completion
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI.
   **/
  public OpenaiChatCompletionResponse chatCompletion (List<Message> messages) {
    return chatCompletion(model, messages);
  }

  /**
   * Performs a chat completion using OpenAI with the set model and the given chat messages.
   * @param chat the chat containing the messages to use for the completion
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI
   **/
  public OpenaiChatCompletionResponse chatCompletion (Chat chat) {
    List<Message> messages = chat.toPrompt();
    return chatCompletion(model, messages);
  }

  /**
   * Performs a chat completion using OpenAI with a provided model and the messages in the given chat.
   * @param model the model to use for the completion
   * @param chat the chat containing the messages to use for the completion
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI
   **/
  public OpenaiChatCompletionResponse chatCompletion (Symbol model, Chat chat) {
    return chatCompletion(model.toString(), chat);
  }
  public OpenaiChatCompletionResponse chatCompletion (String model, Chat chat) {
    List<Message> messages = chat.toPrompt();
    return chatCompletion(model, messages);
  }

  /**
   * Performs an OpenAI chat completion using the specified model and the given list of messages.
   * This method creates a new OpenaiChatRequestBody object with the provided model and messages.
   * @param model the model to use for the chat completion
   * @param messages a list of messages to be sent for chat completion
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI
   **/
  public OpenaiChatCompletionResponse chatCompletion (Symbol model, List<Message> messages) {
    return chatCompletion(model.toString(), messages);
  }
  public OpenaiChatCompletionResponse chatCompletion (String model, List<Message> messages) {
    OpenaiChatRequestBody requestBody = new OpenaiChatRequestBody(model, messages);
    return chatCompletion(requestBody);
  }

  /**
   * Completes a chat prompt using the OpenAI API.
   * https://platform.openai.com/docs/api-reference/chat
   * @param requestBody an OpenaiChatRequestBody object containing the chat model and messages
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI
   **/
  private OpenaiChatCompletionResponse chatCompletion (OpenaiChatRequestBody requestBody) {
    if (canonical && apiKey == null) {
      log.error("Cannot complete request without an API key for OpenAI.");
      return null;
    }
    if (canonical && !chatCompletionModels.containsKey(requestBody.model)) {
      log.error("Model " + model + " cannot be used with the OpenAI chat completion endpoint.");
      return null;
    }
    /*tokenizer.setEncoding("openai", requestBody.model);
    int tokens = 0;
    for (Message message : requestBody.messages) {
      tokens += tokenizer.count(message.content);
    }
    log.warn("Tokens in request: " + tokens);
    if (tokens > maxTokens ) {
      log.warn("prompt contains: " + tokens + " tokens. Max tokens: " + maxTokens + " for model: " + model);
    }*/
    Map<String, String> headers = new HashMap<>();
    if (canonical) {
      headers.put("Authorization", "Bearer " + apiKey);
    }
    Gson gson = new Gson();
    String response = Http.sendPostRequest(chatCompletionEndpoint, requestBody, headers);
    log.debug("Response: " + response);
    return gson.fromJson(response, OpenaiChatCompletionResponse.class);
  }
}